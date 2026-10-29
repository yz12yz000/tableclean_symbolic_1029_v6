"""Checked commands for the installed hand_control driver.

That driver exposes raw serial replies on /rosout but no state topic/service.
Read GPP register blocks through that existing interface; do not open its serial
port a second time. The observed FC protocol replies contain a two-byte register
address after 0x55, followed by register data (not a Dynamixel error byte).
Unknown layouts and persistent feedback loss fail closed. Commands confirm the
current target state, without requiring a particular previous control state.
"""

import argparse
from collections import deque
from dataclasses import dataclass
import fcntl
import json
import math
import os
from pathlib import Path
import sys
import time


def crc16(data):
    crc = 0
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x8005 if crc & 0x8000 else 0)) & 0xffff
    return crc


@dataclass(frozen=True)
class JawState:
    device_id: int
    enabled: int
    control: int
    position: int


def decode_state(raw):
    """Accept only a CRC-valid address-0, 62-register response seen on this unit."""
    if len(raw) != 74 or raw[:4] != b'\xff\xff\xfc\x00' or raw[7] != 0x55:
        raise ValueError('gripper_unrecognized_state_packet')
    if int.from_bytes(raw[5:7], 'little') != len(raw) - 7:
        raise ValueError('gripper_invalid_packet_length')
    if crc16(raw[:-2]) != int.from_bytes(raw[-2:], 'little'):
        raise ValueError('gripper_invalid_packet_crc')
    if raw[8:10] != b'\x00\x00':
        raise ValueError('gripper_unexpected_register_address')
    registers = raw[10:-2]
    if registers[6] != raw[4] or registers[39] not in (0, 1) or registers[41] not in (0, 1):
        raise ValueError('gripper_unrecognized_register_layout')
    return JawState(raw[4], registers[39], registers[41],
                    int.from_bytes(registers[56:58], 'little'))


# Raw encoder counts for this installed gripper: repeated open captures 676..684.
# This is an aperture check, not evidence that an object was gripped.
OPEN_POSITION = 680
OPEN_TOLERANCE = 10
ALREADY_OPEN_SETTLE_TIME = .4
COMMAND_QUERY_DELAY = .2
QUERY_RETRY_INTERVAL = .5
POLL_INTERVAL = .15
MAX_SAMPLE_GAP = 1.


def is_open(state):
    return (state.enabled == 1 and state.control == 0
            and abs(state.position - OPEN_POSITION) <= OPEN_TOLERANCE)


class Completion:
    """Confirm the requested present state and stability, independent of history."""
    def __init__(self, command, before, settle_time, started):
        if before.enabled != 1:
            raise RuntimeError('gripper_not_enabled')
        self.command, self.before, self.settle_time = command, before, settle_time
        self.started, self.window = started, deque()
        self.expected = 1 if command == 'Close' else 0
        self.last = before
        self.last_sample_time = None

    def at_target(self, state):
        if self.command == 'Open':
            return is_open(state)
        return state.control == 1 and 0 <= state.position < OPEN_POSITION - OPEN_TOLERANCE

    def update(self, state, now):
        self.last = state
        if state.device_id != self.before.device_id or state.enabled != 1:
            raise RuntimeError('gripper_identity_or_enable_changed')
        # Missing observations must not count as continuous stationary feedback.
        if self.last_sample_time is not None and now - self.last_sample_time > MAX_SAMPLE_GAP:
            self.window.clear()
        self.last_sample_time = now
        if not self.at_target(state):
            self.window.clear()
            return False
        self.window.append((now, state.position))
        while len(self.window) > 1 and self.window[1][0] <= now - self.settle_time:
            self.window.popleft()
        positions = [p for _, p in self.window]
        return (now - self.started >= self.settle_time and len(positions) >= 3
                and now - self.window[0][0] >= self.settle_time
                and max(positions) - min(positions) <= 1)

    def failure(self):
        if self.last.control != self.expected:
            return 'gripper_command_not_applied'
        if not self.at_target(self.last):
            return 'gripper_target_position_not_reached'
        return 'gripper_motion_not_settled'


class DriverClient:
    def __init__(self):
        import rclpy
        from rcl_interfaces.msg import Log
        from hand_control.msg import HandControlCmd
        self.rclpy, self.message_type = rclpy, HandControlCmd
        self.node = rclpy.create_node('checked_gripper_command')
        self.publisher = self.node.create_publisher(HandControlCmd, '/handControlCmd', 10)
        self.latest = None
        self.barrier = 0
        self.buffer = bytearray()
        self.subscription = self.node.create_subscription(Log, '/rosout', self.on_log, 100)

    def on_log(self, msg):
        if msg.name != 'hand_control_node' or not msg.msg.startswith('recv raw data: '):
            return
        stamp = msg.stamp.sec * 10**9 + msg.stamp.nanosec
        if stamp < self.barrier:
            return
        try:
            raw = bytes(int(v, 16) for v in msg.msg.removeprefix('recv raw data: ').split())
        except ValueError:
            return
        self.buffer.extend(raw)
        while len(self.buffer) >= 7:
            index = self.buffer.find(b'\xff\xff\xfc\x00')
            if index < 0:
                self.buffer[:] = self.buffer[-3:]
                return
            del self.buffer[:index]
            if len(self.buffer) < 7:
                return
            length = int.from_bytes(self.buffer[5:7], 'little') + 7
            # Only the known 74-byte state reply is useful here. A damaged
            # length must not block later valid replies behind it.
            if length != 74:
                del self.buffer[0]
                continue
            if len(self.buffer) < length:
                return
            packet = bytes(self.buffer[:length])
            try:
                state = decode_state(packet)
            except ValueError:
                # A truncated frame may contain the next valid header.
                del self.buffer[0]
                continue
            del self.buffer[:length]
            self.latest = (stamp, state)

    def spin(self):
        self.rclpy.spin_once(self.node, timeout_sec=.02)

    def connect(self, deadline):
        while time.monotonic() < deadline:
            drivers = [i for i in self.node.get_subscriptions_info_by_topic('/handControlCmd')
                       if i.node_name == 'hand_control_node']
            if len(drivers) > 1:
                raise RuntimeError('multiple_gripper_drivers')
            if len(drivers) == 1 and self.publisher.get_subscription_count() >= 1:
                return
            self.spin()
        raise TimeoutError('gripper_driver_not_discovered')

    def send(self, command):
        self.publisher.publish(self.message_type(msgtype=command, value=1))

    def read(self, deadline):
        self.latest = None
        barrier = self.node.get_clock().now().nanoseconds
        self.barrier = barrier
        self.buffer.clear()
        next_query = 0.
        while time.monotonic() < deadline:
            if time.monotonic() >= next_query:
                self.send('GPP')  # Retry only read requests, never Open/Close.
                next_query = time.monotonic() + QUERY_RETRY_INTERVAL
            self.spin()
            if self.latest is not None:
                stamp, state = self.latest
                age = (self.node.get_clock().now().nanoseconds - stamp) / 1e9
                if stamp >= barrier and 0 <= age <= .5:
                    return state
        raise TimeoutError('gripper_state_feedback_timeout:require_GPP_raw_register_reply')

    def pause(self, duration, deadline):
        until = min(deadline, time.monotonic() + duration)
        while time.monotonic() < until:
            self.spin()

    def run(self, command, timeout, settle_time):
        if command not in ('Check', 'Open', 'Close'):
            raise ValueError('invalid_gripper_command')
        deadline = time.monotonic() + timeout
        self.connect(deadline)
        before = self.read(min(deadline, time.monotonic() + 4.))
        if before.enabled != 1:
            raise RuntimeError('gripper_not_enabled')
        if command == 'Check':
            return {'command': command, 'position': before.position, 'read_only': True}
        command_sent = command == 'Close' or not is_open(before)
        confirmation_time = settle_time if command_sent else ALREADY_OPEN_SETTLE_TIME
        completion = Completion(command, before, confirmation_time, time.monotonic())

        def issue_once():
            if deadline - time.monotonic() < settle_time + COMMAND_QUERY_DELAY + .5:
                raise TimeoutError('gripper_insufficient_time_to_confirm_command')
            self.send(command)
            # Give the serial write time to complete before a register read.
            self.pause(COMMAND_QUERY_DELAY, deadline)

        if command_sent:
            issue_once()
        else:
            completion.update(before, time.monotonic())
            self.pause(POLL_INTERVAL, deadline)
        while time.monotonic() < deadline:
            state = self.read(deadline)
            complete = completion.update(state, time.monotonic())
            if not command_sent and not is_open(state):
                # The initial open sample was transitional. Ensure Open once,
                # then demand the full settling interval before descending.
                completion = Completion(command, before, settle_time, time.monotonic())
                issue_once()
                command_sent = True
                continue
            if complete:
                return {'command': command, 'position_before': before.position,
                        'position_after': state.position, 'command_sent': command_sent,
                        'feedback': ('target_state_confirmed_and_settled' if command_sent
                                     else 'already_open_confirmed'),
                        'grasp_verified': False}
            self.pause(POLL_INTERVAL, deadline)
        raise TimeoutError(completion.failure())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('command', choices=['Check', 'Open', 'Close'])
    parser.add_argument('--timeout', type=float, required=True)
    parser.add_argument('--settle-time', type=float, default=3.)
    args = parser.parse_args()
    if not all(math.isfinite(v) and v > 0 for v in (args.timeout, args.settle_time)):
        parser.error('timeout and settle-time must be finite and positive')
    domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    path = Path('/tmp') / f'robot-grasp-{os.getuid()}-domain-{domain}.gripper-command.lock'
    fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    client = None
    import rclpy
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        rclpy.init()
        client = DriverClient()
        print(json.dumps(client.run(args.command, args.timeout, args.settle_time)), flush=True)
        return 0
    except Exception as exc:
        print(str(exc), file=sys.stderr, flush=True)
        return 1
    finally:
        if client is not None:
            client.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        os.close(fd)


if __name__ == '__main__':
    sys.exit(main())
