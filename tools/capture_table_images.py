#!/usr/bin/env python3
"""Subscribe to fresh camera RGB only; no publishers or robot commands."""
import argparse
import json
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--topic', default='/camera/color/image_raw')
    parser.add_argument('--count', type=int, default=3)
    parser.add_argument('--interval', type=float, default=1.)
    parser.add_argument('--timeout', type=float, default=30.)
    args = parser.parse_args()
    import math
    if (args.count < 1 or not math.isfinite(args.interval) or args.interval < 0
            or not math.isfinite(args.timeout) or args.timeout <= 0):
        parser.error('count/interval/timeout invalid')
    if args.output_dir.exists():
        parser.error('output directory already exists')
    import cv2
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    rclpy.init()
    node = rclpy.create_node('visual_prompt_image_recorder')
    frames, records = [], []
    subscription = node.create_subscription(Image, args.topic, frames.append, qos_profile_sensor_data)
    deadline, next_capture, previous_stamp = time.monotonic()+args.timeout, 0., 0
    try:
        args.output_dir.mkdir(parents=True)
        while len(records) < args.count:
            if time.monotonic() >= deadline:
                raise TimeoutError('fresh_camera_image_timeout')
            rclpy.spin_once(node, timeout_sec=.05)
            if not frames:
                continue
            message = frames[-1]
            frames.clear()
            stamp = message.header.stamp.sec*10**9+message.header.stamp.nanosec
            age = (node.get_clock().now().nanoseconds-stamp)/1e9
            if stamp <= previous_stamp or not 0 <= age < 2. or time.monotonic() < next_capture:
                continue
            image = CvBridge().imgmsg_to_cv2(message, desired_encoding='bgr8')
            name = f'frame_{len(records)+1:03d}.png'
            if not cv2.imwrite(str(args.output_dir/name), image):
                raise OSError('image_write_failed')
            records.append(dict(image=name, source_stamp_ns=stamp, receipt_age_s=age,
                                frame_id=message.header.frame_id, width=message.width, height=message.height))
            previous_stamp, next_capture = stamp, time.monotonic()+args.interval
            print(name, stamp, flush=True)
    finally:
        if args.output_dir.exists():
            (args.output_dir/'capture.json').write_text(json.dumps(dict(
                topic=args.topic, hardware_motion=False, frames=records), indent=2)+'\n')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
