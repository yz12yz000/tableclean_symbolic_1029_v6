from types import SimpleNamespace as NS

import pytest

from grasp_executor.gripper_command import Completion, DriverClient, JawState, crc16, decode_state
from grasp_executor.config import GraspConfig
from grasp_executor.grasp import Pose, execute_grasp_place


# Actual GPP response captured on 2026-09-28, no gripper motion requested.
CAPTURE = bytes.fromhex(
    'FF FF FC 00 03 43 00 55 00 00 14 01 D0 00 00 00 03 02 02 00 '
    '00 00 00 00 00 00 00 00 00 00 84 03 20 03 B8 0B DC 05 00 00 '
    '0B 00 01 00 00 00 04 00 01 01 01 00 58 02 0F 00 AA 02 90 01 '
    '0F 00 00 00 00 00 A9 02 44 00 17 00 1B 0E')


def test_actual_register_reply_has_address_not_error_byte():
    assert decode_state(CAPTURE) == JawState(3, 1, 0, 681)


@pytest.mark.parametrize('index', [4, 5, 8, 40, 65, 73])
def test_corrupted_packet_never_becomes_feedback(index):
    raw = bytearray(CAPTURE)
    raw[index] ^= 1
    with pytest.raises(ValueError):
        decode_state(raw)


def test_unrecognized_layout_even_with_correct_crc_is_rejected():
    raw = bytearray(CAPTURE)
    raw[8] = 6
    raw[-2:] = crc16(raw[:-2]).to_bytes(2, 'little')
    with pytest.raises(ValueError, match='register_address'):
        decode_state(raw)


def client():
    c = DriverClient.__new__(DriverClient)
    c.barrier, c.buffer, c.latest = 10**9, bytearray(), None
    return c


def message(data, sec=2, name='hand_control_node'):
    return NS(name=name, msg='recv raw data: ' + data.hex(' '), stamp=NS(sec=sec, nanosec=0))


def test_fragmented_serial_reads_are_reassembled():
    c = client()
    for part in [CAPTURE[:2], CAPTURE[2:15], CAPTURE[15:]]:
        c.on_log(message(part))
    assert c.latest == (2 * 10**9, JawState(3, 1, 0, 681))


def test_stale_and_foreign_logs_are_not_feedback():
    c = client()
    c.on_log(message(CAPTURE, sec=0))
    c.on_log(message(CAPTURE, name='other_node'))
    assert c.latest is None


def test_close_command_readback_without_movement_never_allows_lift():
    progress = Completion('Close', JawState(3, 1, 0, 681), 3., 0.)
    for t in range(12):
        assert not progress.update(JawState(3, 1, 1, 681), float(t))
    assert progress.failure() == 'gripper_target_position_not_reached'


def test_close_waits_until_position_has_stayed_stable_for_full_duration():
    progress = Completion('Close', JawState(3, 1, 0, 600), 3., 0.)
    for t, p in [(0., 600), (1., 620), (2., 630), (3., 640), (4., 640), (5., 640)]:
        assert not progress.update(JawState(3, 1, 1, p), t)
    assert progress.update(JawState(3, 1, 1, 640), 6.)


def test_continued_slow_motion_does_not_count_as_settled():
    progress = Completion('Close', JawState(3, 1, 0, 600), 3., 0.)
    for t in range(12):
        assert not progress.update(JawState(3, 1, 1, 605 + t), float(t))


@pytest.mark.parametrize('state', [JawState(4, 1, 1, 640), JawState(3, 0, 1, 640)])
def test_wrong_device_or_disabled_gripper_is_fatal(state):
    progress = Completion('Close', JawState(3, 1, 0, 600), 3., 0.)
    with pytest.raises(RuntimeError):
        progress.update(state, 4.)


def test_release_requires_movement_when_before_was_closed():
    progress = Completion('Open', JawState(3, 1, 1, 640), 3., 0.)
    for t in range(12):
        assert not progress.update(JawState(3, 1, 0, 640), float(t))


def test_already_open_can_be_confirmed_before_descent():
    progress = Completion('Open', JawState(3, 1, 0, 680), 3., 0.)
    for t in range(3):
        assert not progress.update(JawState(3, 1, 0, 680), float(t))
    assert progress.update(JawState(3, 1, 0, 680), 3.)


def test_feedback_failure_stops_task_before_lift_or_release():
    events = []
    def close(timeout):
        events.append('close')
        raise TimeoutError('gripper_no_position_change')
    def move(pose, timeout):
        events.append('move')
        return True
    interface = NS(move_to_pose=move, move_linear=move, is_pose_reached=lambda timeout: True,
                   open_gripper=lambda timeout: True, close_gripper=close,
                   stop_motion=lambda timeout: events.append('stop') or True)
    def pose(z):
        return Pose((.4, .1, z), (0., 1., 0., 0.), 'base_link', 'grasp_center')
    result = execute_grasp_place(pose(.3), pose(.2), pose(.3), pose(.2), interface, GraspConfig())
    assert result.status == 'failed' and result.failed_step == 'closing_gripper'
    assert result.reason == 'gripper_no_position_change'
    assert events == ['move', 'move', 'close', 'stop']


def test_close_accepts_already_closed_without_prior_open_or_movement():
    progress = Completion('Close', JawState(3, 1, 1, 9), 3., 0.)
    for t in range(3):
        assert not progress.update(JawState(3, 1, 1, 9), float(t))
    assert progress.update(JawState(3, 1, 1, 9), 3.)


def test_open_register_alone_is_not_open_aperture():
    progress = Completion('Open', JawState(3, 1, 0, 9), 3., 0.)
    for t in range(6):
        assert not progress.update(JawState(3, 1, 0, 9), float(t))
    assert progress.failure() == 'gripper_target_position_not_reached'


def test_missing_feedback_does_not_count_as_stability():
    progress = Completion('Close', JawState(3, 1, 1, 9), 3., 0.)
    for t in [0., 1., 2., 4.]:
        assert not progress.update(JawState(3, 1, 1, 9), t)
    assert not progress.update(JawState(3, 1, 1, 9), 5.)
    assert not progress.update(JawState(3, 1, 1, 9), 6.)
    assert progress.update(JawState(3, 1, 1, 9), 7.)


@pytest.mark.parametrize('command,before,after,sent', [
    ('Open', JawState(3, 1, 0, 680), JawState(3, 1, 0, 680), []),
    ('Open', JawState(3, 1, 0, 9), JawState(3, 1, 0, 680), ['Open']),
    ('Open', JawState(3, 1, 1, 9), JawState(3, 1, 0, 680), ['Open']),
    ('Close', JawState(3, 1, 1, 9), JawState(3, 1, 1, 9), ['Close']),
    ('Close', JawState(3, 1, 0, 680), JawState(3, 1, 1, 9), ['Close']),
])
def test_command_policy_uses_current_state(monkeypatch, command, before, after, sent):
    from grasp_executor import gripper_command as module
    clock = [0.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    c = client()
    commands = []
    readings = [before]
    c.connect = lambda deadline: None
    c.send = commands.append
    c.read = lambda deadline: readings.pop() if readings else after
    def spin():
        clock[0] += .05
    c.spin = spin
    result = c.run(command, 12., 3.)
    assert commands == sent
    assert result['command_sent'] == bool(sent)
    assert not result['grasp_verified']


@pytest.mark.parametrize('respond', [True, False])
def test_missing_reply_retries_read_only_within_deadline(monkeypatch, respond):
    from grasp_executor import gripper_command as module
    clock = [0.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    c = client()
    c.node = NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=int((1+clock[0])*1e9))))
    commands = []
    c.send = commands.append
    def spin():
        clock[0] += .05
        if respond and len(commands) >= 2:
            c.latest = (int((1+clock[0])*1e9), JawState(3, 1, 0, 680))
    c.spin = spin
    if respond:
        assert c.read(2.).position == 680
    else:
        with pytest.raises(TimeoutError, match='gripper_state_feedback_timeout'):
            c.read(2.)
    assert 2 <= len(commands) <= 4
    assert set(commands) == {'GPP'}
    assert clock[0] <= 2.1


@pytest.mark.parametrize('prefix', [
    CAPTURE[:5] + b'\x00\x08' + CAPTURE[7:20],  # corrupt length could wait for 2055 bytes
    CAPTURE[:30],  # lost suffix followed by a complete replacement response
    CAPTURE[:-1] + bytes([CAPTURE[-1] ^ 1]),  # bad CRC
    b'\xff\xff\xfc\x00\x03\x04\x00\x55\x00\x00\x00',  # write reply
])
def test_damaged_frame_does_not_swallow_next_valid_response(prefix):
    c = client()
    c.on_log(message(prefix + CAPTURE))
    assert c.latest == (2 * 10**9, JawState(3, 1, 0, 681))
    assert not c.buffer


def timed_client(monkeypatch, states):
    from grasp_executor import gripper_command as module
    clock = [0.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    c = client()
    commands, reads = [], []
    states = iter(states)
    current = [None]
    c.connect = lambda deadline: None
    c.send = lambda cmd: commands.append((cmd, clock[0]))
    def read(deadline):
        reads.append(clock[0])
        current[0] = next(states, current[0])
        return current[0]
    c.read = read
    def spin():
        clock[0] += .05
    c.spin = spin
    return c, clock, commands, reads


def test_already_open_requires_multiple_stable_samples_without_open_command(monkeypatch):
    c, clock, commands, reads = timed_client(monkeypatch, [JawState(3, 1, 0, 680)])
    result = c.run('Open', 16., 3.)
    assert result['feedback'] == 'already_open_confirmed'
    assert not commands and len(reads) >= 3 and clock[0] >= .4


def test_open_sample_then_closed_sends_open_once_and_waits_full_interval(monkeypatch):
    c, clock, commands, reads = timed_client(monkeypatch, [
        JawState(3, 1, 0, 680), JawState(3, 1, 1, 640), JawState(3, 1, 0, 680)])
    result = c.run('Open', 16., 3.)
    assert result['command_sent'] and [cmd for cmd, _ in commands] == ['Open']
    assert reads[2] - commands[0][1] >= .2
    assert clock[0] - commands[0][1] >= 3.


def test_close_write_is_separated_from_first_query(monkeypatch):
    c, clock, commands, reads = timed_client(monkeypatch, [
        JawState(3, 1, 0, 680), JawState(3, 1, 1, 640)])
    c.run('Close', 16., 3.)
    assert [cmd for cmd, _ in commands] == ['Close']
    assert reads[1] - commands[0][1] >= .2


def test_too_little_time_never_sends_control_command(monkeypatch):
    c, _, commands, _ = timed_client(monkeypatch, [JawState(3, 1, 0, 680)])
    with pytest.raises(TimeoutError, match='insufficient_time'):
        c.run('Close', 2., 3.)
    assert not commands


def test_close_never_treated_as_success_while_aperture_remains_open(monkeypatch):
    c, clock, commands, reads = timed_client(monkeypatch, [
        JawState(3, 1, 0, 680), JawState(3, 1, 1, 680)])
    with pytest.raises(TimeoutError, match='target_position_not_reached'):
        c.run('Close', 6., 3.)
    assert [cmd for cmd, _ in commands] == ['Close']
    assert clock[0] <= 6.1


@pytest.mark.parametrize('failure,move_count,expected_step', [
    ('open1', 1, 'opening_gripper'),
    ('close', 2, 'closing_gripper'),
    ('open2', 5, 'releasing_gripper'),
])
def test_gripper_failure_blocks_next_motion_and_never_retries(failure, move_count, expected_step):
    events = []
    opens = [0]
    def move(pose, timeout):
        events.append('move')
        return True
    def open_gripper(timeout):
        opens[0] += 1
        event = f'open{opens[0]}'
        events.append(event)
        if failure == event:
            raise TimeoutError('gripper_state_feedback_timeout')
        return True
    def close_gripper(timeout):
        events.append('close')
        if failure == 'close':
            raise TimeoutError('gripper_state_feedback_timeout')
        return True
    interface = NS(move_to_pose=move, move_linear=move, is_pose_reached=lambda timeout: True,
                   open_gripper=open_gripper, close_gripper=close_gripper,
                   stop_motion=lambda timeout: events.append('stop') or True)
    def pose(z):
        return Pose((.4, .1, z), (0., 1., 0., 0.), 'base_link', 'grasp_center')
    result = execute_grasp_place(pose(.3), pose(.2), pose(.3), pose(.2), interface,
                                GraspConfig(place_dwell_time=.001))
    assert result.status == 'failed' and result.failed_step == expected_step
    assert events.count('move') == move_count
    assert events[-2:] == [failure, 'stop']
