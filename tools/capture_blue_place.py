#!/usr/bin/env python3
"""Capture a stationary current tool pose as a release point; derive ready +Z."""

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

from robot_control import JOINTS, joint_values, position_window_stationary
from collections import deque


ROOT = Path(__file__).resolve().parents[1]
def pose(transform):
    p = transform.transform.translation
    q = transform.transform.rotation
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in values) or abs(math.hypot(q.x, q.y, q.z, q.w) - 1) > .01:
        raise ValueError('invalid_transform')
    return {'position': dict(zip('xyz', values[:3])),
            'orientation': dict(zip('xyzw', values[3:])),
            'stamp_ns': transform.header.stamp.sec * 10**9 + transform.header.stamp.nanosec}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target-zone', default='blue')
    parser.add_argument('--output', type=Path, default=ROOT / 'docs' / 'records' / 'blue_release_position.json')
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    rclpy.init()
    node = rclpy.create_node('place_release_pose_capture')
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    samples = deque()
    observed = {'since': None, 'last': None, 'joint': None, 'error': None}

    def on_joint(msg):
        now = time.monotonic()
        try:
            stamp, positions = joint_values(msg, node.get_clock().now().nanoseconds)
            if observed['last'] is None or now - observed['last'] > .5:
                samples.clear()
                observed['since'] = None
            if position_window_stationary(samples, stamp, positions):
                observed['since'] = observed['since'] or now
                observed['joint'] = dict(zip(JOINTS, positions))
                observed['error'] = None
            else:
                observed['since'] = None
        except (ValueError, IndexError) as exc:
            samples.clear()
            observed['since'] = None
            observed['error'] = str(exc)
        observed['last'] = now

    sub = node.create_subscription(JointState, '/joint_states', on_joint, qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            now = time.monotonic()
            if observed['since'] is None or now - observed['since'] < .4 or now - observed['last'] > .5:
                continue
            try:
                raw = buffer.lookup_transform('base', 'tool0', Time())
                program = buffer.lookup_transform('base_link', 'grasp_center', Time())
                base_frame = buffer.lookup_transform('base_link', 'base', Time())
            except Exception as exc:
                observed['error'] = str(exc)
                continue
            poses = [pose(t) for t in (raw, program, base_frame)]
            now_ns = node.get_clock().now().nanoseconds
            if any(p['stamp_ns'] <= 0 or not -.2 <= (now_ns-p['stamp_ns'])/1e9 <= 1 for p in poses[:2]):
                observed['error'] = f'stale_tf: ages={[round((now_ns-p["stamp_ns"])/1e9, 3) for p in poses]}'
                continue
            # +Z in base must map to +Z in base_link for this direct offset.
            q = poses[2]['orientation']
            z_axis = 1 - 2*(q['x']**2 + q['y']**2)
            if z_axis < .9999:
                raise RuntimeError('base_and_base_link_z_axes_not_aligned')
            ready = []
            for release in poses[:2]:
                derived = {'position': dict(release['position']),
                           'orientation': dict(release['orientation'])}
                derived['position']['z'] += .1
                ready.append(derived)
            record = {
                'target': args.target_zone,
                'captured_at': datetime.now().astimezone().isoformat(),
                'capture_method': 'read-only fresh TF and stationary joint feedback',
                'joint_positions': observed['joint'],
                'release_base_tool0': poses[0],
                'descent_start_base_tool0': ready[0],
                'release_base_link_grasp_center': poses[1],
                'descent_start_base_link_grasp_center': ready[1],
                'descent_distance_m': .1,
                'motion_commands_sent': False,
            }
            temp = args.output.with_suffix('.json.tmp')
            temp.write_text(json.dumps(record, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
            temp.replace(args.output)
            print(json.dumps(record, ensure_ascii=False, indent=2))
            return
        raise TimeoutError(observed['error'] or 'stationary_fresh_joint_and_tf_timeout')
    finally:
        node.destroy_subscription(sub)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
