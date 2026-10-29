#!/usr/bin/env python3
"""Actual YOLOE node with recorded RGB-D in an empty localhost DDS domain.

Only the vision launch is started. No driver, locator, planner, executor or
controller is created. The recorded depth/TF are test inputs, not live data.
"""
import json
import os
from pathlib import Path
import random
import signal
import subprocess
import tempfile
import time
import uuid

from isolated_ros import IsolatedRosTestLock


def run():
    root = Path(__file__).resolve().parents[1]
    old_domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    domain = random.SystemRandom().choice([v for v in range(150, 220) if v != old_domain])
    os.environ.update(ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1', PYTHONNOUSERSITE='1')
    for key in ('ROS_DISCOVERY_SERVER', 'FASTRTPS_DEFAULT_PROFILES_FILE',
                'FASTDDS_DEFAULT_PROFILES_FILE', 'CYCLONEDDS_URI'):
        os.environ.pop(key, None)
    directory = Path(tempfile.mkdtemp(prefix='single-scene-ros-'))
    os.environ['ROS_LOG_DIR'] = str(directory / 'ros_logs')
    print('Test output:', directory, 'ROS_DOMAIN_ID:', domain, flush=True)
    import numpy as np
    import rclpy
    from cv_bridge import CvBridge
    from geometry_msgs.msg import TransformStamped
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2
    from std_msgs.msg import String
    from tf2_ros import StaticTransformBroadcaster
    from rim_locator.pointcloud import read_xyz_object_ids
    from test_live_place_pipeline import stop_owned_processes

    rclpy.init()
    node = Node('single_scene_test_coordinator')
    process, stream = None, None
    result = dict(passed=False, domain=domain, hardware_started=False, live_camera_started=False,
                  motion_targets_published=0, checks=[])
    try:
        deadline = time.monotonic() + 1.
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        if any(name != node.get_name() for name in node.get_node_names()):
            raise RuntimeError('test_domain_not_empty')
        namespace = '/single_scene_' + uuid.uuid4().hex[:8]
        config = json.loads((root / 'src/yolo_vision/config/clear_table_vision.json').read_text())
        config.update(rgb_topic=namespace+'/rgb', depth_image_topic=namespace+'/depth',
                      depth_info_topic=namespace+'/info', output_topic=namespace+'/objects',
                      model_path=str(root/'src/yolo_vision'/config['model_path']),
                      tray_config_path=str(directory/'tray.json'))
        tray = json.loads((root / 'src/yolo_vision/config/clear_table_live.json').read_text())
        tray['yolo']['model_path'] = config['model_path']
        (directory/'vision.json').write_text(json.dumps(config))
        (directory/'tray.json').write_text(json.dumps(tray))
        statuses, clouds = [], []
        retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, namespace+'/yolo_vision/status',
                                 lambda msg: statuses.append(json.loads(msg.data)), retained)
        node.create_subscription(PointCloud2, config['output_topic'], clouds.append, retained)
        publishers = [node.create_publisher(kind, config[key], qos_profile_sensor_data)
                      for kind, key in ((Image, 'rgb_topic'), (Image, 'depth_image_topic'),
                                        (CameraInfo, 'depth_info_topic'))]
        recording = root / 'debug_output/clear_table_live/20261010_pCnsvN/after_reset/observation.npz'
        data = np.load(recording, allow_pickle=False)
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'base', 'test_camera_optical'
        tx, ty, tz = data['translation']
        qx, qy, qz, qw = data['quaternion']
        transform.transform.translation.x = float(-tx)
        transform.transform.translation.y = float(-ty)
        transform.transform.translation.z = float(tz)
        rotation = transform.transform.rotation
        rotation.x, rotation.y, rotation.z, rotation.w = map(float, [-qy, qx, qw, -qz])
        broadcaster = StaticTransformBroadcaster(node)
        broadcaster.sendTransform(transform)
        stream = (directory/'vision.log').open('x')
        process = subprocess.Popen([str(root/'run_yolo_vision.sh'),
            'config_path:='+str(directory/'vision.json'), 'namespace:='+namespace,
            'task_id:=recorded_test', 'observation_id:=recorded_test/1'],
            cwd=root, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic()+40.
        while not any(s['status'] == 'ready' for s in statuses):
            if process.poll() is not None or any(s['status'] == 'failed' for s in statuses):
                raise RuntimeError('vision_startup_failed:' + str(statuses[-1:]))
            if time.monotonic() >= deadline:
                raise TimeoutError('vision_startup_timeout')
            rclpy.spin_once(node, timeout_sec=.05)
        bridge = CvBridge()
        rgb = bridge.cv2_to_imgmsg(data['bgr'], encoding='bgr8')
        depth = bridge.cv2_to_imgmsg(data['xyz'][:, :, 2].copy(), encoding='32FC1')
        info = CameraInfo(height=800, width=1280, k=data['k'].tolist(), d=[0.]*5)
        stamp = node.get_clock().now().to_msg()
        for message in (rgb, depth, info):
            message.header.frame_id = 'test_camera_optical'
            message.header.stamp = stamp
        next_publish = 0.
        while not any(s['status'] in ('success', 'failed', 'no_targets') for s in statuses):
            if time.monotonic() >= deadline or process.poll() is not None:
                raise TimeoutError('vision_recorded_inference_timeout')
            if time.monotonic() >= next_publish:
                for publisher, message in zip(publishers, (rgb, depth, info)):
                    publisher.publish(message)
                next_publish = time.monotonic()+.5
            rclpy.spin_once(node, timeout_sec=.05)
        terminal = statuses[-1]
        result['status'] = terminal
        assert terminal['status'] == 'success', terminal
        assert terminal['perception']['mode'] == 'single_yoloe_scene'
        assert terminal['perception']['model_inferences'] == 1
        assert terminal['perception']['class_counts']['tray'] == 1
        assert terminal['detected_count'] == terminal['perception']['class_counts']['bowl'] >= 1
        result['checks'].append('one_scene_inference_returns_both_classes')
        assert terminal['source_stamp_ns'] == stamp.sec*10**9+stamp.nanosec
        assert terminal['task_id'] == 'recorded_test' and terminal['observation_id'] == 'recorded_test/1'
        result['checks'].append('sensor_stamp_and_observation_identity_preserved')
        until = time.monotonic()+3.
        while not clouds and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
        assert len(clouds) == 1
        points, ids = read_xyz_object_ids(clouds[0])
        accepted = {obj['object_id'] for obj in terminal['objects']}
        assert set(ids) == accepted
        assert len(points) == sum(obj['point_count'] for obj in terminal['objects'])
        assert clouds[0].header.stamp == stamp
        result['checks'].append('only_bowl_points_published_in_original_camera_frame')
        assert not any(name in ('rim_locator', 'grasp_executor', 'controller_manager', 'move_group')
                       for name in node.get_node_names())
        result['checks'].append('no_motion_nodes_or_controllers_started')
        result['passed'] = True
    except Exception as exc:
        result['error'] = type(exc).__name__ + ':' + str(exc)
    finally:
        if process is not None:
            errors = stop_owned_processes({'vision': process})
            result['cleanup_errors'] = errors
            if errors:
                result['passed'] = False
        if stream is not None:
            stream.close()
        node.destroy_node()
        rclpy.shutdown()
        (directory/'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    with IsolatedRosTestLock():
        raise SystemExit(run())
