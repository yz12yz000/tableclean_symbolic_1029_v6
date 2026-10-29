"""One-shot ROS 2 node. Successful output retains the sensor acquisition stamp."""

import json
from pathlib import Path
import signal
from threading import Event
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .config import load_config, resolve_model_path
from .pointcloud import extract_object_points, make_cloud, read_xyz, read_aligned_depth_xyz
from .segmentation import Segmenter, TraySegmentationConfig
from .sensor_input import SerializedSensor, raw_subscriber
from .diagnostics import detection_debug, projection_metadata
from .tray_utils import (detect_tray, estimate_object_position, estimate_obstacle_position,
                         load_tray_config)


def retained_qos():
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class VisionNode(Node):
    def __init__(self):
        super().__init__("yolo_vision")
        self.state = "initializing"
        self.task_id = ""
        self.observation_id = ""
        self.source_not_before_ns = 0
        self.status_pub = self.create_publisher(String, "~/status", retained_qos())
        try:
            self.task_id = self.declare_parameter("task_id", "").value
            self.observation_id = self.declare_parameter("observation_id", "").value
            self.source_not_before_ns = self.declare_parameter("source_not_before_ns", 0).value
            for name in ("task_id", "observation_id"):
                value = getattr(self, name)
                if not isinstance(value, str) or (value and not value.strip()):
                    raise ValueError("invalid_observation_parameter:" + name)
            if bool(self.task_id) != bool(self.observation_id):
                raise ValueError("incomplete_observation_identity")
            if type(self.source_not_before_ns) is not int or self.source_not_before_ns < 0:
                raise ValueError("invalid_observation_parameter:source_not_before_ns")
            share = Path(get_package_share_directory("yolo_vision"))
            path = self.declare_parameter("config_path", str(share / "config/config.json")).value
            self.config = load_config(path)
            self.tray_config = None
            self.tray_tf_error = None
            if self.config.tray_config_path:
                tray_path = Path(self.config.tray_config_path).expanduser()
                if not tray_path.is_absolute():
                    tray_path = share / tray_path
                self.tray_config = load_tray_config(tray_path, require_calibrated=True)
                self.tf_buffer = Buffer(node=self)
                self.tf_listener = TransformListener(self.tf_buffer, self)
            self.publisher = self.create_publisher(PointCloud2, self.config.output_topic, retained_qos())
            self.bridge = CvBridge()
            self.rgb_pub = self.create_publisher(Image, "~/camera_rgb", retained_qos())
            if self.config.publish_debug:
                self.mask_pub = self.create_publisher(Image, "~/mask", retained_qos())
                self.overlay_pub = self.create_publisher(Image, "~/overlay", retained_qos())
            self.tray_segmenter = None
            if self.config.scene_classes is not None:
                if (self.tray_config is None or
                        self.tray_config['yolo']['target_class'] not in self.config.scene_classes
                        or self.tray_config['yolo']['target_class'] == self.config.target_class
                        or self.tray_config['yolo']['target_class'] in self.config.class_aliases):
                    raise ValueError('scene_inference_requires_distinct_bowl_and_tray_classes')
                # One checkpoint and one model instance for the complete table.
                if resolve_model_path(TraySegmentationConfig(**self.tray_config['yolo']), share).resolve() != resolve_model_path(self.config, share).resolve():
                    raise ValueError('scene_tray_model_must_match_vision_model')
                tray_class = self.tray_config['yolo']['target_class']
                tray_threshold = self.config.class_confidences.get(tray_class, self.config.confidence)
                if (self.tray_config['yolo']['confidence'] != tray_threshold
                        or any(self.tray_config['yolo'][name] != getattr(self.config, name)
                               for name in ('imgsz', 'device'))):
                    raise ValueError('scene_tray_inference_settings_must_match_vision')
            elif self.tray_config is not None:
                tray_model_config = TraySegmentationConfig(**self.tray_config['yolo'])
                self.tray_segmenter = Segmenter(
                    resolve_model_path(tray_model_config, share), tray_model_config,
                    require_target=True)
            self.segmenter = Segmenter(resolve_model_path(self.config, share), self.config)
            self.inputs = {}
            sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            accept = lambda: self.state == "ready"
            on_error = lambda exc: self.report("failed", failed_step="camera_input", reason=str(exc))
            self.rgb_sub = raw_subscriber(self, Image, self.config.rgb_topic, sensor_qos, accept, on_error)
            spatial_type = Image if self.config.depth_image_topic else PointCloud2
            spatial_topic = self.config.depth_image_topic or self.config.pointcloud_topic
            self.cloud_sub = raw_subscriber(self, spatial_type, spatial_topic, sensor_qos, accept, on_error)
            self.rgb_sub.registerCallback(lambda message: self.observe_input("rgb", message))
            self.cloud_sub.registerCallback(lambda message: self.observe_input("cloud", message))
            inputs = [self.rgb_sub, self.cloud_sub]
            if self.config.depth_image_topic:
                self.info_sub = Subscriber(self, CameraInfo, self.config.depth_info_topic, qos_profile=sensor_qos)
                self.info_sub.registerCallback(lambda message: self.observe_input("depth_info", message))
                inputs.append(self.info_sub)
            self.synchronizer = ApproximateTimeSynchronizer(
                inputs, self.config.sync_queue_size,
                self.config.sync_slop, allow_headerless=False,
            )
            self.synchronizer.registerCallback(self.on_pair)
            self.wait_started = time.monotonic()
            self.timer = self.create_timer(0.1, self.check_timeout, clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.report("ready")
        except Exception as exc:
            self.report("failed", failed_step="initializing", reason=str(exc))

    def report(self, status, **details):
        self.state = status
        payload = json.dumps({"status": status, **details,
                              "task_id": self.task_id,
                              "observation_id": self.observation_id,
                              "source_not_before_ns": self.source_not_before_ns}, ensure_ascii=False)
        self.status_pub.publish(String(data=payload))
        self.get_logger().info(payload)

    def check_timeout(self):
        if self.state == "ready" and self.deadline_expired():
            VisionNode.report_input_timeout(self)

    def report_input_timeout(self):
        details = {}
        if getattr(self, 'tray_tf_error', None):
            details['tray'] = self.tray_tf_error
        self.report("failed", failed_step="ready", reason="input_timeout",
                    camera=self.camera_details(), **details)

    def camera_to_base(self, header):
        """Use acquisition-time TF, never a latest transform or an assumed frame."""
        target_frame = self.tray_config['frame_id']
        if header.frame_id == target_frame:
            return np.eye(4)
        value = self.tf_buffer.lookup_transform(
            target_frame, header.frame_id, Time.from_msg(header.stamp)).transform
        q = value.rotation
        quaternion = np.asarray([q.x, q.y, q.z, q.w], dtype=float)
        norm = float(np.linalg.norm(quaternion))
        t = value.translation
        translation = np.asarray([t.x, t.y, t.z], dtype=float)
        if not np.isfinite(quaternion).all() or norm < 1e-12 or not np.isfinite(translation).all():
            raise ValueError('invalid_tray_transform')
        x, y, z, w = quaternion / norm
        matrix = np.eye(4)
        matrix[:3, :3] = [
            [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
        ]
        matrix[:3, 3] = translation
        return matrix

    def observe_input(self, name, message):
        previous = self.inputs.get(name, {})
        self.inputs[name] = {"received": previous.get("received", 0) + 1,
                             "width": message.width, "height": message.height,
                             "frame_id": message.header.frame_id,
                             "stamp_ns": message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec}

    def camera_details(self):
        details = {"rgb_topic": self.config.rgb_topic,
                   "spatial_topic": self.config.depth_image_topic or self.config.pointcloud_topic,
                   "depth_info_topic": self.config.depth_info_topic if self.config.depth_image_topic else None,
                   "expected_size": [self.config.expected_width, self.config.expected_height],
                   "sync_slop": self.config.sync_slop, "inputs": getattr(self, "inputs", {})}
        inputs = details["inputs"]
        required = ("rgb", "cloud", "depth_info") if self.config.depth_image_topic else ("rgb", "cloud")
        missing = [name for name in required if name not in inputs]
        details["missing"] = missing
        if not missing:
            details["latest_stamp_delta_s"] = abs(inputs["rgb"]["stamp_ns"] - inputs["cloud"]["stamp_ns"]) / 1e9
        return details

    def deadline_expired(self):
        return time.monotonic() - self.wait_started >= self.config.input_timeout

    def on_pair(self, image, cloud, camera_info=None):
        if self.state != "ready":
            return
        if self.deadline_expired():
            VisionNode.report_input_timeout(self)
            return
        try:
            stamps = [m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec for m in (image, cloud)]
            now_ns = self.get_clock().now().nanoseconds
            if self.source_not_before_ns:
                if now_ns < self.source_not_before_ns:
                    raise ValueError("source_clock_before_observation_barrier")
                # Ignore queued images acquired before this observation. Do
                # not lock the one-shot node or restart its timeout for them.
                # CameraInfo keeps the existing synchronization contract;
                # its timestamp is not an image acquisition freshness gate.
                if any(stamp <= self.source_not_before_ns for stamp in stamps):
                    return
            tray_config = getattr(self, 'tray_config', None)
            if tray_config is None:
                self.report("processing")  # Lock the job BEFORE any decoding/inference.
            if not cloud.header.frame_id.strip():
                raise ValueError("empty_frame_id")
            shape = (image.height, image.width)
            if shape != (cloud.height, cloud.width) or shape != (self.config.expected_height, self.config.expected_width):
                raise ValueError("dimension_mismatch")
            if cloud.height <= 1:
                raise ValueError("cloud_not_organized")
            if image.header.frame_id != cloud.header.frame_id:
                raise ValueError("depth_not_registered_to_rgb_frame")
            if abs(stamps[0] - stamps[1]) > self.config.sync_slop * 1e9:
                raise ValueError("rgb_depth_not_synchronized")
            if any(s <= 0 or not -0.1 <= (now_ns - s)/1e9 <= self.config.max_source_age for s in stamps):
                raise ValueError("source_timestamp_stale_or_future")
            if self.config.depth_image_topic:
                if camera_info is None:
                    raise ValueError("depth_camera_info_missing")
                info_stamp = camera_info.header.stamp.sec * 1_000_000_000 + camera_info.header.stamp.nanosec
                if abs(info_stamp - stamps[1]) > self.config.sync_slop * 1e9:
                    raise ValueError("depth_camera_info_not_synchronized")
            camera_to_base = None
            if tray_config is not None:
                try:
                    camera_to_base = VisionNode.camera_to_base(self, cloud.header)
                except TransformException as exc:
                    # Let the single-threaded executor receive TF and retry with
                    # a new synchronized pair. Do not restart the input budget.
                    self.tray_tf_error = {'status': 'invalid', 'reason': 'tray_tf_unavailable',
                                          'frame_id': tray_config['frame_id'], 'detail': str(exc),
                                          'source_stamp_ns': stamps[1]}
                    # This discarded frame is diagnostic only. A top-level
                    # stamp identifies the final frozen observation and must
                    # not conflict with a later candidate on another topic.
                    self.report('ready', tray=self.tray_tf_error)
                    return
                self.tray_tf_error = None
                self.report('processing')
            if isinstance(image, SerializedSensor):
                image = image.decode()
            if isinstance(cloud, SerializedSensor):
                cloud = cloud.decode()
            bgr = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
            scene = None
            if self.config.scene_classes is not None:
                scene = self.segmenter.predict_scene(bgr)
                results = scene[self.config.target_class]
            else:
                results = self.segmenter.predict_all(bgr)
            if self.deadline_expired():
                raise TimeoutError("input_timeout")
            xyz = (read_aligned_depth_xyz(cloud, camera_info) if self.config.depth_image_topic
                   else read_xyz(cloud))
            tray_details = {}
            if tray_config is not None:
                if self.deadline_expired():
                    raise TimeoutError("input_timeout")
                # Shared mode reuses the SAME model result, even with no bowls.
                # Tray pixels never enter the grasp cloud.
                tray_results = (scene[tray_config['yolo']['target_class']] if scene is not None
                                else self.tray_segmenter.predict_all(bgr))
                if self.deadline_expired():
                    raise TimeoutError("input_timeout")
                # read_aligned_depth_xyz already validates CameraInfo frame,
                # dimensions, rectification and intrinsics for registered RGB-D.
                # Organized clouds carry the same calibrated pixel/XYZ mapping.
                camera_matrix = (np.asarray(camera_info.k, dtype=float).reshape(3, 3)
                                 if camera_info is not None else None)
                occluders = ([item.mask for name, items in scene.items()
                              if name != tray_config['yolo']['target_class'] for item in items]
                             if scene is not None else [result.mask for result in results])
                tray_details['tray'] = detect_tray(
                    bgr, xyz, tray_config, camera_matrix=camera_matrix,
                    camera_to_base=camera_to_base, inference_roi=self.config.inference_roi,
                    detections=tray_results, occluder_masks=occluders)
                if scene is not None:
                    obstacles = []
                    for name, items in scene.items():
                        if name in (self.config.target_class, tray_config['yolo']['target_class']):
                            continue
                        for item in items:
                            obstacles.append(dict(object_id=len(obstacles), class_name=name,
                                confidence=item.confidence,
                                **estimate_obstacle_position(xyz[item.mask], camera_to_base, tray_config)))
                    tray_details['obstacles'] = obstacles
                    tray_details['perception'] = {
                        'mode': 'single_yoloe_scene', 'model_inferences': 1,
                        'roi': self.config.inference_roi,
                        'class_counts': {name: len(items) for name, items in scene.items()}}
            # Publish the exact frozen RGB used for this segmentation, including empty scenes.
            if getattr(self, 'rgb_pub', None) is not None:
                raw_message = self.bridge.cv2_to_imgmsg(bgr, encoding="bgr8")
                raw_message.header = image.header
                self.rgb_pub.publish(raw_message)
            tray_details['rgbd_pair'] = {
                'rgb_stamp_ns': stamps[0], 'depth_stamp_ns': stamps[1],
                'rgb_frame_id': image.header.frame_id, 'depth_frame_id': cloud.header.frame_id,
                'sync_slop_s': self.config.sync_slop,
                'width': int(bgr.shape[1]), 'height': int(bgr.shape[0])}
            if tray_config is not None:
                tray_details['camera_to_base_matrix'] = camera_to_base.tolist()
            try:
                tray_details['camera_projection'] = projection_metadata(
                    xyz, image.header, cloud.header, camera_info)
            except Exception as exc:
                tray_details['camera_projection_error'] = str(exc)
            if not results:
                if self.deadline_expired():
                    raise TimeoutError("input_timeout")
                self.report("no_targets", detected_count=0, object_count=0,
                            reason="empty_detection", source_stamp_ns=stamps[1], **tray_details)
                return
            all_points, all_ids, accepted, rejected = [], [], [], []
            occupied = np.zeros(xyz.shape[:2], dtype=bool)
            for object_id, result in enumerate(results):
                if not isinstance(result.mask, np.ndarray) or result.mask.dtype != np.bool_:
                    raise ValueError("mask_must_be_boolean")
                if result.mask.shape != xyz.shape[:2]:
                    raise ValueError("dimension_mismatch")
                # Highest confidence wins overlaps; confidence never sets grasp order.
                mask = result.mask & ~occupied
                occupied |= result.mask
                mask_pixels = int(mask.sum())
                valid_count = int((mask & np.isfinite(xyz).all(axis=2) & (xyz[:, :, 2] > 0)).sum())
                pixels_y, pixels_x = np.nonzero(result.mask)
                bbox = ([int(pixels_x.min()), int(pixels_y.min()), int(pixels_x.max()), int(pixels_y.max())]
                        if len(pixels_x) else None)
                depth_details = dict(pixel_bbox=bbox, manipulation_class=self.config.target_class,
                                     mask_pixels=mask_pixels, valid_depth_points=valid_count,
                                     valid_depth_ratio=valid_count/max(1, mask_pixels))
                if tray_config is not None:
                    depth_details.update(estimate_object_position(xyz[mask], camera_to_base, tray_config))
                try:
                    points = extract_object_points(xyz, mask, self.config.min_points)
                except ValueError as exc:
                    if str(exc) != "insufficient_pointcloud":
                        raise
                    rejected.append({"object_id": object_id, "reason": str(exc), **depth_details})
                    continue
                all_points.append(points)
                all_ids.append(np.full(len(points), object_id, dtype=np.uint32))
                accepted.append({"object_id": object_id, "point_count": len(points),
                                 "confidence": result.confidence, **depth_details})
            if self.config.publish_debug:
                combined_mask, overlay = detection_debug(bgr, results, xyz)
                if scene is not None:
                    # Preserve bowl IDs in the mask image; draw trays only on
                    # the RGB overlay so grasp diagnostics remain compatible.
                    import cv2
                    for target in tray_results:
                        contours, _ = cv2.findContours(target.mask.astype(np.uint8), cv2.RETR_EXTERNAL,
                                                       cv2.CHAIN_APPROX_SIMPLE)
                        cv2.drawContours(overlay, contours, -1, (255, 200, 0), 2)
                        yy, xx = np.nonzero(target.mask)
                        if len(xx):
                            cv2.putText(overlay, f'tray {target.confidence:.2f}',
                                        (int(xx.min()), max(14, int(yy.min())-5)),
                                        cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 200, 0), 1)
                    for name, items in scene.items():
                        if name in (self.config.target_class, tray_config['yolo']['target_class']):
                            continue
                        for item in items:
                            contours, _ = cv2.findContours(item.mask.astype(np.uint8), cv2.RETR_EXTERNAL,
                                                           cv2.CHAIN_APPROX_SIMPLE)
                            cv2.drawContours(overlay, contours, -1, (220, 80, 220), 2)
                            yy, xx = np.nonzero(item.mask)
                            if len(xx):
                                cv2.putText(overlay, f'{name} {item.confidence:.2f}',
                                            (int(xx.min()), max(14, int(yy.min())-5)),
                                            cv2.FONT_HERSHEY_SIMPLEX, .45, (220, 80, 220), 1)
                mask_message = self.bridge.cv2_to_imgmsg(combined_mask, encoding="mono8")
                mask_message.header = image.header
                overlay_message = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
                overlay_message.header = image.header
                if self.deadline_expired():
                    raise TimeoutError("input_timeout")
                self.mask_pub.publish(mask_message)
                self.overlay_pub.publish(overlay_message)
            if self.deadline_expired():
                raise TimeoutError("input_timeout")
            if not all_points:
                self.report("failed", failed_step="processing", reason="no_valid_depth_for_detected_targets",
                            detected_count=len(results), object_count=0, rejected=rejected,
                            source_stamp_ns=stamps[1], **tray_details)
                return
            output = make_cloud(cloud.header, np.concatenate(all_points), np.concatenate(all_ids))
            self.publisher.publish(output)
            self.report("success", detected_count=len(results), object_count=len(accepted), objects=accepted,
                        class_name=self.config.target_class, rejected=rejected,
                        **tray_details,
                        source_stamp_ns=cloud.header.stamp.sec*1_000_000_000+cloud.header.stamp.nanosec,
                        rgb_depth_delta_s=abs((image.header.stamp.sec-cloud.header.stamp.sec) +
                                              (image.header.stamp.nanosec-cloud.header.stamp.nanosec)/1e9))
        except Exception as exc:
            self.report("failed", failed_step="processing", reason=str(exc))


def main(args=None):
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    executor = SingleThreadedExecutor()
    shutdown_requested = Event()
    previous_sigint = signal.getsignal(signal.SIGINT)
    previous_sigterm = signal.getsignal(signal.SIGTERM)

    def request_shutdown(signum, frame):
        shutdown_requested.set()
        executor.wake()

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    node = None
    try:
        node = VisionNode()
        executor.add_node(node)
        while not shutdown_requested.is_set():
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        shutdown_requested.set()
    finally:
        if node is not None:
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)
