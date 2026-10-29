#!/usr/bin/env python3
"""Plan or execute one independently observed bowl for one selected zone.

The default is plan-only: no controller switching, arm motion or gripper command.
Zone execution includes the six grasp/place segments and a fixed-joint return
home. Only pre-motion candidate exhaustion and explicit geometry failures may
request a bounded fresh observation; a completed zone task exits.
"""
import argparse
from copy import deepcopy
from dataclasses import asdict, replace
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
from threading import Event
import uuid

import rclpy
from moveit_msgs.msg import DisplayTrajectory
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2, JointState
from rclpy.qos import qos_profile_sensor_data
from table_round_io import atomic_json, publish_request, render_overlay
from table_round_protocol import validate_decision, validate_bundle
from yolo_vision.config import load_effective_config
from rim_locator.config import load_config as load_rim_config
from rim_locator.candidate_protocol import decode_candidate_batch, encode_candidate_batch
from grasp_executor.config import load_config as load_grasp_config
from grasp_executor.process_guard import process_start_ticks
from table_agent_protocol import (WORKSPACE_REVISION, load_agent_json, scene_revision,
                                  validate_execution_baseline, validate_round_decision,
                                  validate_task_policy)
from table_runtime_metadata import build_preflight
from place_rounds import (RoundBudget, RoundLimits, context_matches,
                          load_place_zone, load_release_coordinates,
                          recoverable_failure, select_candidate_groups, ClearTableSession,
                          tray_release_config)


def write_json(path, value, source_paths=()):
    path = Path(path)
    if path.resolve() in source_paths:
        raise ValueError('output_overwrites_source_config:' + str(path))
    # A task/round result is immutable once written. Never replace older evidence.
    with path.open('x', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)


def task_lock_path():
    domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    return Path('/tmp') / f'robot-grasp-{os.getuid()}-domain-{domain}.place-task.lock'


class TaskLock:
    def __enter__(self):
        fd = os.open(task_lock_path(), os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BaseException:
            os.close(fd)
            raise RuntimeError('another_place_task_is_running')
        self.fd = fd
        return self

    def __exit__(self, *args):
        os.close(self.fd)


def process_group_alive(pgid):
    """Linux process groups include ros2 launch descendants, even after its exit.

    Zombies cannot send commands and may wait for an external parent to reap them.
    Read /proc rather than considering such zombies a live worker forever.
    """
    for path in Path('/proc').glob('[0-9]*/stat'):
        try:
            fields = path.read_text().rsplit(')', 1)[1].split()
            if int(fields[2]) == pgid and fields[0] not in ('Z', 'X'):
                return True
        except (FileNotFoundError, ProcessLookupError):
            continue
    return False


def stop_owned_processes(processes):
    errors = []
    pending = dict(processes)
    for sig, allowance in ((signal.SIGINT, 20.), (signal.SIGTERM, 3.), (signal.SIGKILL, 3.)):
        if sig == signal.SIGTERM:
            errors.extend('forced_shutdown:' + name for name in pending)
        for name, process in pending.items():
            # The session leader can exit before its descendants; always signal
            # the group that this round created, never unrelated ROS processes.
            try:
                os.killpg(process.pid, sig)
            except ProcessLookupError:
                pass
            except OSError as exc:
                errors.append('process_signal_failed:' + name + ':' + str(exc))
        deadline = time.monotonic() + allowance
        while pending:
            for name, process in list(pending.items()):
                process.poll()  # reap the child leader if it has exited
                try:
                    alive = process_group_alive(process.pid)
                except OSError as exc:
                    errors.append('process_group_check_failed:' + name + ':' + str(exc))
                    alive = True
                if not alive:
                    pending.pop(name)
            if not pending or time.monotonic() >= deadline:
                break
            time.sleep(.05)
        if not pending:
            break
    errors.extend('process_shutdown_unconfirmed:' + name for name in pending)
    return errors


def round_configs(vision, rim, grasp, namespace, timeout, execute):
    return {
        'vision': replace(vision, output_topic=namespace + '/object_cloud'),
        'rim': replace(rim, input_topic=namespace + '/object_cloud',
                       output_topic=namespace + '/rim_point', poses_topic=namespace + '/grasp_poses',
                       candidates_topic=namespace + '/grasp_candidates',
                       input_timeout=max(rim.input_timeout, timeout + 35.)),
        'grasp': replace(grasp, input_mode='candidate_groups', input_topic=namespace + '/selected_candidates',
                         task_preview_topic=namespace + '/task_preview', plan_only=not execute,
                         input_timeout=max(grasp.input_timeout, timeout + 35.)),
    }


class ObservationRound:
    """Own one namespace, one set of processes, and one immutable observation."""
    def __init__(self, node, root, output, task_id, number, configs, deadline,
                 source_not_before_ns, execute, interrupted, source_paths, single_bowl=False,
                 clear_session=None, max_completed_objects=20, task_policy=None,
                 agent_decision_dir=None, agent_round_protocol="v1", previous_action=None, source_guard=None):
        self.node, self.root, self.output = node, root, output
        self.task_id, self.observation_id = task_id, f'{task_id}/round_{number:04d}'
        self.namespace = f'/place_demo_{task_id}/round_{number:04d}'
        self.deadline, self.barrier = deadline, source_not_before_ns
        self.execute, self.interrupted = execute, interrupted
        self.single_bowl = single_bowl
        self.clear_session = clear_session
        self.task_policy = deepcopy(task_policy) if task_policy is not None else None
        self.agent_decision_dir = Path(agent_decision_dir).expanduser().resolve() if agent_decision_dir else None
        self.agent_round_protocol = agent_round_protocol
        self.source_guard = source_guard or {}
        self.previous_action = previous_action or dict(command_attempted=False, motion_completed=False,
            grasp_verified=False, returned_home=None, placement_verified=False, verified_placed_count=0)
        self.joint_feedback = None
        self.home_monitoring = False
        self.max_completed_objects = max_completed_objects
        self.configs, self.source_paths = configs, source_paths
        self.statuses, self.history, self.previews, self.intermediate = {}, [], [], []
        self.processes, self.logs, self.subscriptions = {}, [], []
        self.batch, self.selected, self.callback_error = None, None, None
        self.supervisor_pid = os.getpid()
        self.supervisor_start_ticks = process_start_ticks(self.supervisor_pid)
        self.executor_started = False
        self.publisher = None
        self.active = True
        self.result = {'task_id': task_id, 'observation_id': self.observation_id,
                       'namespace': self.namespace, 'source_not_before_ns': self.barrier,
                       'task_deadline_monotonic_ns': int(deadline * 1e9),
                       'supervisor_pid': self.supervisor_pid, 'supervisor_start_ticks': self.supervisor_start_ticks,
                       'mode': 'execute' if execute else 'plan_only', 'completed_count': 0,
                       'plan_id': self.task_policy.get('plan_id') if self.task_policy else None}
        output.mkdir(parents=True, exist_ok=False)
        self.snapshots = {name: str(output / f'effective_{name}_config.json') for name in configs}
        for name, config in configs.items():
            if name != 'grasp' or clear_session is None:
                write_json(self.snapshots[name], asdict(config), source_paths)
        if clear_session is not None:
            self.snapshots['grasp'] = None  # Chosen only after this round's tray observation.
        self.result['config_snapshots'] = self.snapshots
        self.qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def attach(self):
        def subscribe(kind, topic, callback, qos=None):
            def guarded(message):
                if not self.active:
                    return
                try:
                    callback(message)
                except Exception as exc:
                    self.callback_error = str(exc)
            self.subscriptions.append(self.node.create_subscription(kind, topic, guarded, qos or self.qos))
        for name in ('yolo_vision', 'rim_locator', 'grasp_executor'):
            subscribe(String, self.namespace + '/' + name + '/status',
                      lambda message, name=name: self.on_status(name, message))
        subscribe(String, self.configs['rim'].candidates_topic, self.on_candidates)
        subscribe(DisplayTrajectory, self.configs['grasp'].task_preview_topic, self.on_preview,
                  QoSProfile(depth=10))
        if self.agent_decision_dir is not None and self.agent_round_protocol == 'v2':
            subscribe(Image, self.namespace + '/yolo_vision/camera_rgb', self.save_rgb)
            subscribe(JointState, '/joint_states', self.on_joint, qos_profile_sensor_data)
        if self.configs['vision'].publish_debug:
            subscribe(Image, self.namespace + '/yolo_vision/overlay', self.save_image)
        if self.configs['rim'].publish_debug:
            subscribe(PointCloud2, self.namespace + '/rim_locator/base_cloud', self.save_cloud)
        self.publisher = self.node.create_publisher(String, self.configs['grasp'].input_topic, self.qos)

    def on_status(self, name, message):
        value = json.loads(message.data)
        if not context_matches(value, self.task_id, self.observation_id):
            return  # Foreign retained callbacks must never advance this round.
        stamp = value.get('source_stamp_ns')
        if stamp is not None:
            if type(stamp) is not int or stamp <= self.barrier:
                raise ValueError('status_before_observation_barrier')
            if self.batch is not None and stamp != self.batch['source_stamp_ns']:
                raise ValueError('status_observation_stamp_mismatch')
        if self.statuses.get(name) != value:
            print(name, json.dumps(value, ensure_ascii=False), flush=True)
            self.history.append({'node': name, 'status': value})
        self.statuses[name] = value

    def on_candidates(self, message):
        # Ignore correctly formed messages belonging to another observation.
        payload = decode_candidate_batch(message.data)
        if not context_matches(payload, self.task_id, self.observation_id):
            return
        payload = decode_candidate_batch(message.data, expected_task_id=self.task_id,
            expected_observation_id=self.observation_id, expected_frame_id=self.configs['grasp'].base_frame,
            expected_tool_frame=self.configs['grasp'].tool_frame, source_not_before_ns=self.barrier)
        if self.batch is not None:
            if payload != self.batch:
                raise ValueError('observation_changed_after_publication')
            return
        self.batch = payload
        write_json(self.output / 'candidates.json', payload, self.source_paths)

    def write_scene_snapshot(self, vision, decision=None):
        """Persist one immutable, Agent-readable summary for this round."""
        if self.clear_session is None:
            return
        payload = {
            'schema_version': 1,
            'plan_id': self.task_policy.get('plan_id') if self.task_policy else None,
            'task_id': self.task_id,
            'observation_id': self.observation_id,
            'source_stamp_ns': vision.get('source_stamp_ns'),
            'frame_id': 'base',
            'rgbd_pair': vision.get('rgbd_pair'),
            'tray': vision.get('tray'),
            'objects': vision.get('objects', []),
            'rejected': vision.get('rejected', []),
            'obstacles': vision.get('obstacles', []),
            'grasp_options': self.batch,
            'placement_options': self.clear_session.list_options(),
            'config_hash': scene_revision(self.clear_session.config),
            'selection': (dict(
                object_id=decision['selected']['objects'][0]['object_id'],
                candidate_id=next(candidate['candidate_id']
                                  for candidate in decision['selected']['objects'][0]['candidates']
                                  if not candidate.get('disabled', False)),
                slot_id=decision['slot']['id'])
                          if decision and decision.get('kind') == 'ready' else None),
            'limits': self.task_policy.get('limits') if self.task_policy else None,
        }
        # ClearTableSession computes the revision from the exact tray/object/
        # candidate inputs used for selection.  Keep that digest stable even
        # though this persisted bundle also carries diagnostics and limits.
        payload['scene_revision'] = self.clear_session.current_scene_revision() or scene_revision(payload)
        self.result['scene_revision'] = payload['scene_revision']
        write_json(self.output / 'scene.json', payload, self.source_paths)

    def check_sources(self):
        for name, expected in self.source_guard.items():
            path = Path(name)
            if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
                raise RuntimeError('agent_configuration_changed:' + name)

    def on_joint(self, message):
        self.joint_feedback = (message, time.monotonic())
        if self.home_monitoring:
            self.check_home()

    def check_home(self):
        config = self.configs['grasp']
        home = config.return_home_joint_positions
        if not home or self.joint_feedback is None:
            raise RuntimeError('home_feedback_missing')
        msg, received = self.joint_feedback
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        age = (self.node.get_clock().now().nanoseconds-stamp)/1e9
        if (time.monotonic()-received > config.feedback_timeout or stamp <= 0
                or not -config.future_tolerance <= age <= config.feedback_timeout):
            raise RuntimeError('home_feedback_stale')
        if len(set(msg.name)) != len(msg.name) or len(msg.position) != len(msg.name) or len(msg.velocity) != len(msg.name):
            raise RuntimeError('home_feedback_invalid')
        self.result['home_check'] = dict(tolerance_rad=config.start_joint_tolerance,
            expected=home, actual=dict(zip(msg.name, msg.position)), velocity=dict(zip(msg.name, msg.velocity)))
        for name, target in home.items():
            if name not in msg.name:
                raise RuntimeError('home_joint_missing')
            i = msg.name.index(name)
            if (not math.isfinite(msg.position[i]) or not math.isfinite(msg.velocity[i])
                    or abs(msg.position[i]-target) > config.start_joint_tolerance
                    or abs(msg.velocity[i]) > .02):
                raise RuntimeError('robot_not_stationary_at_home')
        return True

    def refresh_home(self):
        """Receive a fresh joint sample after CPU-bound scene/slot computation."""
        started = time.monotonic()
        deadline = min(self.deadline, started + 5.)
        while self.joint_feedback is None or self.joint_feedback[1] <= started:
            if time.monotonic() >= deadline:
                raise RuntimeError('home_feedback_refresh_timeout')
            self.spin()
        return self.check_home()

    def save_rgb(self, message):
        from PIL import Image as PillowImage
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        if stamp <= self.barrier or 'camera_rgb_header' in self.result:
            return
        if message.encoding not in ('rgb8', 'bgr8'):
            raise ValueError('agent_rgb_encoding')
        image = PillowImage.frombytes('RGB', (message.width, message.height), bytes(message.data),
            'raw', 'BGR' if message.encoding == 'bgr8' else 'RGB', message.step, 1)
        image.save(self.output/'camera_rgb.jpg', quality=95)
        self.result['camera_rgb_header'] = dict(stamp_ns=stamp, frame_id=message.header.frame_id)

    def ensure_rgb(self, vision):
        # All evidence is frozen before the status pointer makes it visible.
        wait_until = min(self.deadline, time.monotonic()+5.)
        while 'camera_rgb_header' not in self.result:
            if time.monotonic() >= wait_until:
                raise RuntimeError('agent_rgb_missing')
            self.spin()
        header, pair = self.result['camera_rgb_header'], vision['rgbd_pair']
        if header != dict(stamp_ns=pair['rgb_stamp_ns'], frame_id=pair['rgb_frame_id']):
            raise RuntimeError('agent_image_scene_mismatch')

    def wait_for_v2(self, decision, vision):
        self.ensure_rgb(vision)
        scene = load_agent_json(self.output/'scene.json')
        previous = dict(self.previous_action,
            placement_verified=self.clear_session.verified_placed_count > 0 and self.clear_session.last_placement is None,
            verified_placed_count=self.clear_session.verified_placed_count)
        request = publish_request(self.output, vision, scene, self.clear_session.list_options(), previous, self.refresh_home())
        pointer = self.agent_decision_dir.parent/'agent_status.json'
        status = dict(state='waiting_agent', plan_id=self.task_policy['plan_id'],
                      task_id=self.task_id, observation_id=self.observation_id,
                      round=self.output.name, scene_revision=request['scene_revision'])
        atomic_json(pointer, status, replace=True)
        path = self.agent_decision_dir / (self.output.name+'.json')
        deadline = min(self.deadline, time.monotonic()+120.)
        while not path.is_file():
            if time.monotonic() >= deadline:
                raise TimeoutError('agent_decision_timeout')
            self.spin()
            self.check_home()
        if time.monotonic() >= deadline:
            raise TimeoutError('agent_decision_timeout')
        self.spin()
        self.check_home()
        age = (self.node.get_clock().now().nanoseconds-vision['source_stamp_ns'])/1e9
        if not 0 <= age <= self.configs['grasp'].max_target_age:
            raise RuntimeError('agent_scene_expired')
        self.check_sources()
        validate_bundle(self.output, request)
        value = validate_decision(load_agent_json(path), request)
        write_json(self.output/'agent_decision.json', value, self.source_paths)
        atomic_json(pointer, dict(status, state='running'), replace=True)
        if value['action'] == 'select':
            return self.clear_session.apply_object_slot(value['selection'])
        if value['action'] == 'reobserve':
            return dict(kind='observe_again', reason='agent_reobserve')
        if value['action'] == 'stop':
            return dict(kind='stop', reason='agent_stop:' + value.get('reason', ''))
        return decision  # finish is legal only for locally confirmed completion

    def wait_for_agent_decision(self, automatic_decision):
        """Wait for one atomic JSON selection, then bind it to this scene."""
        if self.agent_decision_dir is None:
            return automatic_decision
        path = self.agent_decision_dir / f'round_{self.observation_id.rsplit("_", 1)[-1]}.json'
        deadline = min(self.deadline, time.monotonic() + 120.)
        while not path.is_file():
            if time.monotonic() >= deadline:
                raise TimeoutError('agent_decision_timeout')
            self.spin()
            time.sleep(.02)
        try:
            with path.open(encoding='utf-8') as stream:
                value = validate_round_decision(json.load(stream),
                    task_id=self.task_id,
                    plan_id=self.task_policy['plan_id'],
                    observation_id=self.observation_id,
                    scene_revision=self.clear_session.current_scene_revision())
            result = self.clear_session.apply_selection(value)
            write_json(self.output / 'agent_decision.json', value, self.source_paths)
            return result
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
            raise RuntimeError('agent_decision_invalid:' + str(exc)) from exc

    def write_verification(self, status, decision, *, passed):
        """Persist the no-motion planning result used by this round."""
        payload = {
            'schema_version': 1,
            'task_id': self.task_id,
            'observation_id': self.observation_id,
            'scene_revision': self.result.get('scene_revision'),
            'passed': bool(passed),
            'command_attempted': bool(status.get('command_attempted', False)),
            'candidate_failures': status.get('candidate_failures', []),
            'reason': status.get('reason', ''),
            'selected': {
                'object_id': decision.get('source', {}).get('object_id'),
                'candidate_ids': [candidate.get('candidate_id')
                                 for group in decision.get('selected', {}).get('objects', [])
                                 for candidate in group.get('candidates', [])
                                 if not candidate.get('disabled', False)],
                'slot_id': decision.get('slot', {}).get('id'),
            },
        }
        payload['verification_id'] = scene_revision(payload)
        self.result['verification'] = payload
        write_json(self.output / 'verification.json', payload, self.source_paths)

    def on_preview(self, message):
        preview = {'segments': len(message.trajectory),
                   'points': [len(t.joint_trajectory.points) for t in message.trajectory]}
        expected = 7 if self.configs['grasp'].return_home_joint_positions is not None else 6
        (self.previews if preview['segments'] == expected else self.intermediate).append(preview)

    def save_image(self, message):
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        if stamp <= self.barrier or 'detection_overlay' in self.result:
            return
        try:
            from PIL import Image as PillowImage
            if message.encoding not in ('bgr8', 'rgb8'):
                raise ValueError('unsupported_debug_image_encoding')
            image = PillowImage.frombytes('RGB', (message.width, message.height), bytes(message.data),
                'raw', 'BGR' if message.encoding == 'bgr8' else 'RGB', message.step, 1)
            path = self.output / 'detection_overlay.png'
            image.save(path)
            self.result['detection_overlay'] = str(path)
            self.result['detection_image_header'] = dict(stamp_ns=stamp, frame_id=message.header.frame_id)
        except Exception as exc:
            self.result.setdefault('debug_capture_errors', []).append(str(exc))

    def save_cloud(self, message):
        stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
        if stamp <= self.barrier or 'base_cloud' in self.result:
            return
        try:
            import numpy as np
            from rim_locator.pointcloud import read_xyz_object_ids
            points, ids = read_xyz_object_ids(message)
            path = self.output / 'base_cloud.npz'
            np.savez_compressed(path, points=points, object_ids=ids,
                                frame_id=message.header.frame_id, stamp_ns=stamp)
            self.result['base_cloud'] = str(path)
        except Exception as exc:
            self.result.setdefault('debug_capture_errors', []).append(str(exc))

    def save_selected_pose_overlay(self):
        """Diagnostic failures never change a robot task's execution outcome."""
        try:
            from grasp_pose_overlay import selected_grasp_poses, render_grasp_overlay
            poses = selected_grasp_poses(self.selected, self.history, self.configs['grasp'])
            if not poses:
                self.result['grasp_pose_overlay_status'] = 'no_selected_pose'
                return
            camera = self.statuses.get('yolo_vision', {}).get('camera_projection')
            transform = self.statuses.get('rim_locator', {}).get('cloud_transform')
            metadata = dict(task_id=self.task_id, observation_id=self.observation_id,
                            camera_projection=camera, cloud_transform=transform, poses=poses)
            if 'detection_overlay' in self.result:
                import cv2
                image = cv2.imread(self.result['detection_overlay'])
                if image is None:
                    raise ValueError('detection_image_unreadable')
                output, metadata['poses'] = render_grasp_overlay(image, poses, camera, transform,
                    self.result.get('detection_image_header'))
                path = self.output / 'grasp_pose_overlay.png'
                if not cv2.imwrite(str(path), output):
                    raise OSError('grasp_pose_image_write_failed')
                self.result['grasp_pose_overlay'] = str(path)
                self.result['grasp_pose_overlay_status'] = ('annotated' if all(
                    'projection_error' not in pose for pose in metadata['poses']) else 'coordinates_only')
            else:
                self.result['grasp_pose_overlay_status'] = 'image_unavailable'
            path = self.output / 'selected_grasp_pose.json'
            write_json(path, metadata, self.source_paths)
            self.result['selected_grasp_pose'] = str(path)
        except Exception as exc:
            self.result.setdefault('debug_capture_errors', []).append('grasp_pose_overlay:' + str(exc))

    def spin(self):
        if self.agent_decision_dir is not None and (self.agent_decision_dir.parent/'cancel.json').exists():
            self.interrupted.set()
        if self.interrupted.is_set():
            raise RuntimeError('interrupted')
        if time.monotonic() >= self.deadline:
            raise TimeoutError('pipeline_timeout')
        rclpy.spin_once(self.node, timeout_sec=min(.05, max(0., self.deadline-time.monotonic())))
        if self.callback_error:
            raise RuntimeError('invalid_round_message:' + self.callback_error)
        for name, process in self.processes.items():
            if name != 'position_control' and process.poll() is not None:
                raise RuntimeError('child_process_exited:' + name)

    def start(self, name, *extra):
        path = self.root / ('run_grasp_position_control.sh' if name == 'position_control' else f'run_{name}.sh')
        command = [str(path), *map(str, extra)]
        if name != 'position_control':
            config_key = {'rim_locator': 'rim', 'yolo_vision': 'vision', 'grasp_executor': 'grasp'}[name]
            command += [f'config_path:={self.snapshots[config_key]}', f'namespace:={self.namespace}',
                        f'task_id:={self.task_id}', f'observation_id:={self.observation_id}']
            if name in ('yolo_vision', 'grasp_executor'):
                command.append(f'source_not_before_ns:={self.barrier}')
            if name == 'grasp_executor':
                command.append(f'task_deadline_monotonic_ns:={int(self.deadline*1e9)}')
                command.append(f'task_preview_topic:={self.configs["grasp"].task_preview_topic}')
                command += [f'supervisor_pid:={self.supervisor_pid}',
                            f'supervisor_start_ticks:={self.supervisor_start_ticks}']
        log = (self.output / (name + '.log')).open('x')
        self.logs.append(log)
        process = subprocess.Popen(command, cwd=self.root, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        self.processes[name] = process
        return process

    def wait_ready(self, name, status):
        startup_deadline = min(self.deadline, time.monotonic() + 35.)
        while self.statuses.get(name, {}).get('status') != status:
            if any(s.get('status') == 'failed' for s in self.statuses.values()):
                raise RuntimeError('pipeline_startup_failed')
            if time.monotonic() >= startup_deadline:
                raise TimeoutError('pipeline_startup_timeout:' + name)
            self.spin()

    def prepare_controller(self):
        process = self.start('position_control')
        stop = min(self.deadline, time.monotonic() + 240.)
        try:
            while process.poll() is None:
                if time.monotonic() >= stop:
                    raise TimeoutError('controller_preparation_timeout')
                self.spin()
            if process.returncode != 0:
                raise RuntimeError('controller_preparation_failed')
        except Exception:
            path = self.output / 'position_control.log'
            self.result['controller_preparation_log'] = str(path)
            self.result['controller_exit_code'] = process.poll()
            try:
                with path.open('rb') as stream:
                    stream.seek(0, os.SEEK_END)
                    stream.seek(max(0, stream.tell() - 4096))
                    detail = stream.read().decode('utf-8', errors='replace')
                self.result['controller_error_detail'] = '\n'.join(detail.splitlines()[-12:]).strip()
            except OSError as exc:
                self.result['controller_error_detail'] = '控制器日志无法读取：' + str(exc)
            raise

    def run(self, controller_prepared):
        if getattr(self, 'clear_session', None) is not None:
            return self.run_clear_table(controller_prepared)
        self.attach()
        self.start('rim_locator')
        self.wait_ready('rim_locator', 'waiting_input')
        self.start('yolo_vision')
        while True:
            self.spin()
            failed = any(s.get('status') in ('failed', 'candidates_exhausted') for s in self.statuses.values())
            if failed:
                # DDS does not order different topics. Keep the all-rejected
                # protocol message before retiring this round's subscriptions.
                rim_status = self.statuses.get('rim_locator', {})
                if rim_status.get('reason') == 'pose_not_found:no_valid_object':
                    delivery_deadline = min(self.deadline, time.monotonic() + 1.)
                    def evidence_pending():
                        return self.batch is None or (self.configs['rim'].publish_debug
                                                      and 'base_cloud' not in self.result)
                    while evidence_pending() and time.monotonic() < delivery_deadline:
                        self.spin()
                    if self.batch is None:
                        raise RuntimeError('missing_rejected_candidate_batch')
                    if self.configs['rim'].publish_debug and 'base_cloud' not in self.result:
                        self.result.setdefault('debug_capture_errors', []).append('base_cloud_delivery_timeout')
                self.result['kind'] = 'retry' if recoverable_failure(self.statuses) else 'failed'
                self.result['error'] = next((s.get('reason') or s['status'] for s in self.statuses.values()
                    if s.get('status') in ('failed', 'candidates_exhausted')), 'pipeline_failed')
                return controller_prepared
            vision = self.statuses.get('yolo_vision', {})
            if vision.get('status') == 'no_targets':
                if vision.get('detected_count') != 0 or self.batch is not None or self.executor_started:
                    raise RuntimeError('inconsistent_empty_observation')
                if type(vision.get('source_stamp_ns')) is not int:
                    raise RuntimeError('missing_observation_stamp')
                self.result['kind'] = 'empty'
                return controller_prepared
            if (not self.executor_started and self.batch is not None
                    and vision.get('status') == 'success'
                    and self.statuses.get('rim_locator', {}).get('status') == 'success'):
                if not self.batch['objects']:
                    raise RuntimeError('empty_localized_batch')
                stamps = [vision.get('source_stamp_ns'), self.statuses['rim_locator'].get('source_stamp_ns')]
                if any(stamp != self.batch['source_stamp_ns'] for stamp in stamps):
                    raise RuntimeError('perception_stamp_mismatch')
                accepted_ids = {obj['object_id'] for obj in vision.get('objects', [])}
                located_ids = {obj['object_id'] for obj in self.batch['objects'] + self.batch['rejected']}
                if accepted_ids != located_ids or len(accepted_ids) != vision.get('object_count'):
                    raise RuntimeError('perception_object_mismatch')
                if self.single_bowl and (vision.get('detected_count') != 1
                        or vision.get('object_count') != 1
                        or len(self.batch['objects']) != 1 or self.batch['rejected']
                        or vision.get('rejected') or self.statuses['rim_locator'].get('rejected')):
                    raise RuntimeError('single_bowl_required')
                self.selected = select_candidate_groups(self.batch, self.configs['grasp'].grasp_offset, self.execute)
                write_json(self.output / 'selected_candidates.json', self.selected, self.source_paths)
                if self.execute and not controller_prepared:
                    self.prepare_controller()
                    controller_prepared = True
                self.start('grasp_executor', '--execute' if self.execute else '--plan-only')
                self.executor_started = True
                self.wait_ready('grasp_executor', 'waiting_target')
                self.publisher.publish(String(data=encode_candidate_batch(self.selected)))
            status = self.statuses.get('grasp_executor', {})
            if status.get('status') in ('success', 'planned'):
                expected = 'success' if self.execute else 'planned'
                count = len(self.selected['objects']) if self.selected else 0
                if status['status'] != expected or status.get('object_count') != count:
                    raise RuntimeError('executor_result_mismatch')
                expected_completed = 1 if self.execute else 0
                if status.get('completed_count') != expected_completed:
                    raise RuntimeError('completed_count_mismatch')
                self.result['completed_count'] = expected_completed
                # Status and DisplayTrajectory are independent topics; allow delivery to settle.
                drain_until = min(self.deadline, time.monotonic() + .5)
                while time.monotonic() < drain_until:
                    self.spin()
                if len(self.previews) != count:
                    raise RuntimeError('preview_batch_mismatch')
                self.result['kind'] = 'completed' if self.execute else 'planned'
                return controller_prepared

    def run_clear_table(self, controller_prepared):
        """Reuse the one-object executor, selecting its destination after observation."""
        self.attach()
        if self.agent_decision_dir is not None and self.agent_round_protocol == 'v2':
            until = min(self.deadline, time.monotonic()+5.)
            while self.joint_feedback is None and time.monotonic() < until:
                self.spin()
            self.check_home()
            self.check_sources()
            self.home_monitoring = True
        self.start('rim_locator')
        self.wait_ready('rim_locator', 'waiting_input')
        self.start('yolo_vision')
        while True:
            self.spin()
            vision = self.statuses.get('yolo_vision', {})
            rim = self.statuses.get('rim_locator', {})
            # Only documented per-object geometry rejections are scope-filterable.
            # TF, sensor, process, protocol and execution errors stay fatal.
            if vision.get('status') == 'failed' and vision.get('reason') != 'no_valid_depth_for_detected_targets':
                self.result.update(kind='failed', error=vision.get('reason', 'vision_failed'))
                return controller_prepared
            if rim.get('status') == 'failed' and rim.get('reason') != 'pose_not_found:no_valid_object':
                self.result.update(kind='failed', error=rim.get('reason', 'localization_failed'))
                return controller_prepared
            terminal_vision = vision.get('status') in ('success', 'no_targets', 'failed')
            has_accepted = bool(vision.get('objects'))
            if terminal_vision and (not has_accepted or
                    (self.batch is not None and rim.get('status') in ('success', 'failed'))):
                if has_accepted and rim.get('source_stamp_ns') != vision.get('source_stamp_ns'):
                    raise RuntimeError('perception_stamp_mismatch')
                if not has_accepted and self.batch is not None:
                    raise RuntimeError('inconsistent_empty_observation')
                if self.batch:
                    # Full ID/schema integrity remains checked by the session;
                    # ordering alone is inherited from the existing runner.
                    batch = select_candidate_groups(self.batch, self.configs['grasp'].grasp_offset, False)
                else:
                    batch = None
                decision = self.clear_session.observe(vision, batch)
                self.result['clear_table_decision'] = decision
                self.result['tray_state'] = self.clear_session.summary()
                self.write_scene_snapshot(vision, decision)
                if self.agent_decision_dir is not None and self.agent_round_protocol == 'v2':
                    self.ensure_rgb(vision)
                    if decision['kind'] not in ('ready', 'complete'):
                        render_overlay(self.output, vision, [])
                if decision.get('kind') in ('ready', 'complete') and self.agent_decision_dir is not None:
                    if self.agent_round_protocol == 'v2':
                        decision = self.wait_for_v2(decision, vision)
                    elif decision['kind'] == 'ready':
                        decision = self.wait_for_agent_decision(decision)
                    self.result['clear_table_decision'] = decision
                    write_json(self.output / 'agent_selected_decision.json', decision, self.source_paths)
                if decision['kind'] != 'ready':
                    self.result.update(kind={'observe_again': 'retry', 'complete': 'table_complete',
                                             'stop': 'failed'}[decision['kind']], error=decision['reason'])
                    return controller_prepared
                if self.clear_session.motion_completed_count >= self.max_completed_objects:
                    self.result.update(kind='failed', error='max_completed_objects_reached')
                    return controller_prepared
                self.home_monitoring = False
                self.selected = deepcopy(decision['selected'])
                compensation = {}
                visual_compensation = (self.clear_session.config.get('object_geometry') == 'visual_rim'
                                       and self.clear_session.config.get('compensate_bowl_xy'))
                if visual_compensation:
                    from place_rounds import visual_tray_grasp_config
                    self.configs['grasp'] = visual_tray_grasp_config(decision, self.configs['grasp'],
                                                                   self.clear_session.config)
                elif self.clear_session.config.get('compensate_bowl_xy'):
                    # Keep all contacts.  The executor computes the held
                    # centre and release XY separately for every candidate,
                    # so a rejected contact can be replaced safely.
                    source = deepcopy(decision['source'])
                    source['source_stamp_ns'] = self.selected['source_stamp_ns']
                    compensation = dict(source=source)
                if not visual_compensation:
                    self.configs['grasp'] = tray_release_config(decision['slot'],
                                                           self.clear_session.config, self.configs['grasp'],
                                                           **compensation)
                path = self.output / 'effective_grasp_config.json'
                write_json(path, asdict(self.configs['grasp']), self.source_paths)
                self.snapshots['grasp'] = str(path)
                write_json(self.output / 'selected_candidates.json', self.selected, self.source_paths)
                if self.execute and not controller_prepared:
                    self.prepare_controller()
                    controller_prepared = True
                self.start('grasp_executor', '--execute' if self.execute else '--plan-only')
                self.executor_started = True
                self.wait_ready('grasp_executor', 'waiting_target')
                self.publisher.publish(String(data=encode_candidate_batch(self.selected)))
                break
        while True:
            self.spin()
            status = self.statuses.get('grasp_executor', {})
            if status.get('status') in ('failed', 'candidates_exhausted'):
                # A returned pre-motion exhaustion may reobserve; every attempted
                # command failure stops just as it does in the old single-bowl path.
                retry = (status.get('status') == 'candidates_exhausted'
                         and status.get('command_attempted') is False)
                if retry:
                    self.clear_session.register_plan_failure(decision, status)
                self.write_verification(status, decision, passed=False)
                if status.get('command_attempted') is True:
                    write_json(self.output / 'execution.json', status, self.source_paths)
                self.result.update(kind='retry' if retry else 'failed',
                                   error=status.get('reason', 'executor_failed'))
                return controller_prepared
            if status.get('status') in ('success', 'planned'):
                expected = 'success' if self.execute else 'planned'
                completed = int(self.execute)
                if (status['status'] != expected or status.get('object_count') != 1
                        or status.get('completed_count') != completed):
                    raise RuntimeError('executor_result_mismatch')
                self.write_verification(status, decision, passed=True)
                if self.execute:
                    write_json(self.output / 'execution.json', status, self.source_paths)
                if self.execute:
                    self.clear_session.record_motion(decision)
                self.result.update(completed_count=completed, tray_state=self.clear_session.summary())
                drain_until = min(self.deadline, time.monotonic() + .5)
                while time.monotonic() < drain_until:
                    self.spin()
                if len(self.previews) != 1:
                    raise RuntimeError('preview_batch_mismatch')
                self.result.update(kind='completed' if self.execute else 'planned', completed_count=completed,
                                   tray_state=self.clear_session.summary())
                return controller_prepared

    def close(self):
        self.active = False
        errors = stop_owned_processes(self.processes)
        for subscription in self.subscriptions:
            try:
                self.node.destroy_subscription(subscription)
            except Exception as exc:
                errors.append('subscription_cleanup_failed:' + str(exc))
        if self.publisher is not None:
            try:
                self.node.destroy_publisher(self.publisher)
            except Exception as exc:
                errors.append('publisher_cleanup_failed:' + str(exc))
        for log in self.logs:
            try:
                log.close()
            except OSError as exc:
                errors.append('log_close_failed:' + str(exc))
        executor = self.statuses.get('grasp_executor', {})
        self.result.update(statuses=self.statuses, history=self.history,
            detected_count=self.statuses.get('yolo_vision', {}).get('detected_count'),
            target_count=len(self.batch['objects']) if self.batch else 0,
            selected_object_ids=[g['object_id'] for g in self.selected['objects']] if self.selected else [],
            preview_batches=self.previews, intermediate_previews=self.intermediate,
            candidate_attempt_count=executor.get('candidate_attempt_count', 0),
            rejected={name: self.statuses.get(name, {}).get('rejected', [])
                      for name in ('yolo_vision', 'rim_locator')})
        self.save_selected_pose_overlay()
        try:
            write_json(self.output / 'candidate_attempts.json', executor.get('candidate_failures', []), self.source_paths)
        except OSError as exc:
            errors.append('candidate_log_failed:' + str(exc))
        if errors:
            self.result.update(kind='failed', cleanup_errors=errors, error=';'.join(errors))
        try:
            write_json(self.output / 'result.json', self.result, self.source_paths)
        except OSError as exc:
            errors.append('round_result_write_failed:' + str(exc))
            self.result.update(kind='failed', cleanup_errors=errors, error=';'.join(errors))
        return errors


def main(execute=False):
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--output-dir', default=('debug_output/place_execution/' if execute else
                         'debug_output/place_validation/') + time.strftime('%Y%m%d_%H%M%S'))
    parser.add_argument('--timeout', type=float, default=None)
    parser.add_argument('--vision-config', default=None)
    parser.add_argument('--target-zone', default=None, help='使用已保存的左上、左下、中间、右上、右下或blue点')
    parser.add_argument('--x', type=float, default=None, help='释放点 base/tool0 X，单位米；须与 --y --z 一起使用')
    parser.add_argument('--y', type=float, default=None, help='释放点 base/tool0 Y，单位米')
    parser.add_argument('--z', type=float, default=None, help='释放点 base/tool0 Z，单位米')
    parser.add_argument('--place-zones-config', default=None)
    parser.add_argument('--legacy-multi', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--clear-table', action='store_true', help='每轮抓放一只托盘外的碗，返回后重新观察')
    parser.add_argument('--tray-config', default=None, help='收桌面托盘定位和放置配置 JSON')
    parser.add_argument('--task-policy', default=None, help='可选的、严格校验的收桌面 Agent 策略 JSON')
    parser.add_argument('--baseline-policy', help='执行策略对应的在线预览策略 JSON')
    parser.add_argument('--baseline-result', help='对应的成功在线预览 result.json；动作前核对配置和模型摘要')
    parser.add_argument('--agent-round-protocol', choices=['v1', 'v2'], default='v1')
    parser.add_argument('--agent-decision-dir', default=None,
                        help='可选：按 round_NNNN.json 接收当前场景的 Agent 引用选择')
    parser.add_argument('--execute', action='store_true', default=execute)
    parser.add_argument('--max-reobservations', type=int, default=2)
    parser.add_argument('--max-observation-rounds', type=int, default=30)
    parser.add_argument('--max-completed-objects', type=int, default=20)
    args = parser.parse_args()
    execute = args.execute
    timeout = args.timeout if args.timeout is not None else (900. if execute else 360.)
    task_policy = None
    policy_path = Path(args.task_policy).expanduser().resolve() if args.task_policy else None
    agent_decision_dir = (Path(args.agent_decision_dir).expanduser().resolve()
                          if args.agent_decision_dir else None)
    output = Path(args.output_dir).expanduser().resolve()
    vision_default = root / 'src/yolo_vision/config' / (
        'clear_table_vision.json' if args.clear_table else 'place_config.json')
    rim_path = root / 'src/rim_locator/config/place_config.json'
    grasp_path = root / 'src/grasp_executor/config/place_config.json'
    zones_path = (Path(args.place_zones_config).expanduser().resolve() if args.place_zones_config
                  else root / 'src/grasp_executor/config/place_zones.json')
    override = Path(args.vision_config).expanduser().resolve() if args.vision_config else None
    single_bowl = not args.legacy_multi and not args.clear_table
    tray_path = (Path(args.tray_config).expanduser().resolve() if args.tray_config else
                 root / 'src/yolo_vision/config/clear_table_live.json') if args.clear_table else None
    clear_session = None
    coordinates = (args.x, args.y, args.z)
    has_coordinates = any(value is not None for value in coordinates)
    try:
        if policy_path is not None:
            if not args.clear_table:
                raise ValueError('task_policy_requires_clear_table')
            task_policy = validate_task_policy(
                load_agent_json(policy_path), expected_mode='execute' if execute else 'preview',
                require_deployment_metadata=True,
                expected_workspace_revision=WORKSPACE_REVISION)
            if args.timeout is None:
                timeout = task_policy['limits']['deadline_s']
        if bool(args.baseline_policy) != bool(args.baseline_result):
            raise ValueError('baseline_requires_policy_and_result')
        if args.baseline_policy and (task_policy is None or not execute):
            raise ValueError('baseline_requires_execute_policy')
        if execute and task_policy is not None and not args.baseline_policy:
            raise ValueError('execute_policy_requires_online_preview_baseline')
        if args.agent_round_protocol == 'v2' and agent_decision_dir is None:
            raise ValueError('v2_requires_agent_decision_directory')
        if agent_decision_dir is not None and task_policy is None:
            raise ValueError('agent_decision_requires_task_policy')
        if args.clear_table and (args.legacy_multi or args.target_zone is not None or has_coordinates):
            raise ValueError('clear_table_conflicts_with_place_target_or_legacy_multi')
        if args.tray_config and not args.clear_table:
            raise ValueError('tray_config_requires_clear_table')
        if args.legacy_multi and (args.target_zone is not None or has_coordinates):
            raise ValueError('legacy_multi_conflicts_with_place_target')
        if single_bowl and args.target_zone is None and not has_coordinates:
            raise ValueError('place_target_required:--target-zone or --x --y --z')
        if has_coordinates and (args.target_zone is not None or any(value is None for value in coordinates)):
            raise ValueError('use_either_target_zone_or_complete_xyz')
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('timeout must be positive and finite')
        policy_limits = task_policy.get('limits', {}) if task_policy else {}
        limits = RoundLimits(policy_limits.get('max_reobservations', args.max_reobservations),
                             policy_limits.get('max_observation_rounds', args.max_observation_rounds),
                             policy_limits.get('max_completed_objects', args.max_completed_objects))
        vision, rim, grasp = load_effective_config(vision_default, override), load_rim_config(rim_path), load_grasp_config(grasp_path)
        if single_bowl:
            grasp = (load_release_coordinates(zones_path, coordinates, grasp) if has_coordinates
                     else load_place_zone(zones_path, args.target_zone, grasp))
        if args.clear_table:
            from yolo_vision.tray_utils import load_tray_config
            tray_config = load_tray_config(tray_path, require_calibrated=True)
            if execute and not tray_config.get('motion_approved', False):
                raise ValueError('tray_motion_not_approved')
            clear_session = ClearTableSession(tray_config, policy=task_policy, agent_mode=agent_decision_dir is not None and args.agent_round_protocol == 'v2')
            vision = replace(vision, tray_config_path=str(tray_path))
            # Blue supplies the SAME orientation, descent and home as the old
            # coordinate mode. Only a chosen slot changes its release XY.
            grasp = load_place_zone(zones_path, 'blue', grasp)
        elif vision.tray_config_path is not None:
            raise ValueError('vision_tray_config_requires_clear_table')
        effective_grasp = replace(grasp, plan_only=not execute)
        preflight = build_preflight(root=root, vision=vision, rim=rim, grasp=effective_grasp,
                                    tray=clear_session.config if clear_session else None,
                                    policy=task_policy)
        if args.baseline_policy:
            validate_execution_baseline(task_policy, load_agent_json(args.baseline_policy),
                                        load_agent_json(args.baseline_result), preflight)
        sources = {p.resolve() for p in (vision_default, rim_path, grasp_path, override,
                                         zones_path if single_bowl or args.clear_table else None,
                                         tray_path, policy_path,
                                         Path(args.baseline_policy) if args.baseline_policy else None,
                                         Path(args.baseline_result) if args.baseline_result else None) if p is not None}
        first_output = output / 'round_0001'
        protected_outputs = [output / 'task_config.json', output / 'result.json',
                             output / 'effective_tray_config.json',
                             *[first_output / f'effective_{name}_config.json' for name in ('vision', 'rim', 'grasp')]]
        source_guard = ({str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
                        if agent_decision_dir is not None and args.agent_round_protocol == 'v2' else {})
        if any(p.resolve() in sources for p in protected_outputs):
            raise ValueError('snapshot_overwrites_source_config')
        if any(p.exists() for p in (output / 'result.json', output / 'task_config.json', first_output)):
            raise ValueError('output_directory_already_used')
    except (OSError, ValueError, TypeError) as exc:
        parser.error('invalid pipeline config: ' + str(exc))
    task_id = uuid.uuid4().hex
    budget = RoundBudget(time.monotonic() + timeout, limits)
    result = {'task_id': task_id, 'mode': 'execute' if execute else 'plan_only',
              'hardware_motion_requested': execute,
              'selection': 'clear_table' if args.clear_table else 'single_bowl' if single_bowl else 'all_detected',
              'plan_id': task_policy.get('plan_id') if task_policy else None,
              'task_policy': str(policy_path) if policy_path else None,
              'target_zone': args.target_zone,
              'release_xyz_base_tool0': dict(zip('xyz', coordinates)) if has_coordinates else None,
              'return_home_requested': grasp.return_home_joint_positions is not None,
              'place_zones_config': str(zones_path) if single_bowl or args.clear_table else None,
              'tray_config': str(tray_path) if tray_path else None,
              'vision_config_sources': {'base': str(vision_default), 'override': str(override) if override else None},
              'pick_region': vision.inference_roi, 'placement_region_separation_verified': False,
              'grasp_verified': False, 'placement_verified': False, 'rounds': [],
              'passed': False, 'outcome': 'failed', 'completed_count': 0,
              'preflight': preflight,
              'runner_protocol_version': preflight['runner_protocol_version'],
              'runner_revision': preflight['runner_revision'],
              'workspace_revision': preflight['workspace_revision'],
              'config_digest': preflight['config_digest'],
              'model_digest': preflight['model_digest']}
    interrupted, previous = Event(), {}
    node, initialized, controller_prepared = None, False, False
    owns_output = False
    try:
        with TaskLock():
            output.mkdir(parents=True, exist_ok=True)
            if clear_session is not None:
                tray_snapshot = output / 'effective_tray_config.json'
                write_json(tray_snapshot, clear_session.config, sources)
                vision = replace(vision, tray_config_path=str(tray_snapshot))
            write_json(output / 'task_config.json', {'task_id': task_id, 'timeout': timeout,
                'target_zone': args.target_zone,
                'release_xyz_base_tool0': result['release_xyz_base_tool0'],
                'limits': asdict(limits), 'vision': asdict(vision), 'rim': asdict(rim),
                'grasp': asdict(effective_grasp),
                'tray': clear_session.config if clear_session else None,
                'task_policy': task_policy,
                'vision_config_sources': result['vision_config_sources'],
                'preflight': preflight}, sources)
            owns_output = True
            description = ('收桌面：每轮选择托盘外一只碗，放入空位并返回后重新观察。' if args.clear_table else
                           '单碗单次抓放，目标：' + (args.target_zone or str(result['release_xyz_base_tool0'])) if single_bowl else
                           '抓放每次只处理最近一只；完成退回后重新观测。预览只规划一份快照。')
            print(description, '取物 ROI:', vision.inference_roi, flush=True)
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            initialized = True
            for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
                previous[sig] = signal.getsignal(sig)
                signal.signal(sig, lambda signum, frame: interrupted.set())
            node = Node('live_place_pipeline_test')
            discovery_until = min(budget.deadline, time.monotonic() + 1.)
            while time.monotonic() < discovery_until:
                if interrupted.is_set():
                    raise RuntimeError('interrupted')
                rclpy.spin_once(node, timeout_sec=.05)
            busy = [namespace.rstrip('/') + '/' + name for name, namespace in node.get_node_names_and_namespaces()
                    if name in ('yolo_vision', 'rim_locator', 'grasp_executor')]
            if busy:
                raise RuntimeError('existing project nodes: ' + ', '.join(sorted(busy)))
            while True:
                if agent_decision_dir is not None and (agent_decision_dir.parent/'cancel.json').exists():
                    interrupted.set()
                if interrupted.is_set():
                    raise RuntimeError('interrupted')
                number = budget.begin_round(time.monotonic(), allow_completion_check=args.clear_table)
                barrier = node.get_clock().now().nanoseconds
                if type(barrier) is not int or barrier <= 0:
                    raise RuntimeError('invalid_observation_clock')
                if result['rounds']:
                    last = result['rounds'][-1]
                    prior_stamps = [last['source_not_before_ns']] + [
                        status['source_stamp_ns'] for status in last['statuses'].values()
                        if type(status.get('source_stamp_ns')) is int]
                    if barrier <= max(prior_stamps):
                        raise RuntimeError('observation_clock_did_not_advance')
                namespace = f'/place_demo_{task_id}/round_{number:04d}'
                configs = round_configs(vision, rim, grasp, namespace, timeout, execute)
                observation = ObservationRound(node, root, output / f'round_{number:04d}', task_id, number,
                    configs, budget.deadline, barrier, execute, interrupted, sources, single_bowl,
                    clear_session=clear_session,
                    max_completed_objects=limits.max_completed_objects,
                    task_policy=task_policy,
                    agent_decision_dir=agent_decision_dir, agent_round_protocol=args.agent_round_protocol, source_guard=source_guard,
                    previous_action=next((dict(command_attempted=True, motion_completed=r.get('completed_count') == 1,
                        grasp_verified=r.get('statuses', {}).get('grasp_executor', {}).get('grasp_verified', False),
                        returned_home=r.get('completed_count') == 1)
                        for r in reversed(result['rounds']) if r.get('completed_count') or
                        r.get('statuses', {}).get('grasp_executor', {}).get('command_attempted')), None))
                try:
                    controller_prepared = observation.run(controller_prepared)
                except Exception as exc:
                    observation.result.update(kind='failed', error=str(exc))
                finally:
                    observation.close()
                current = observation.result
                result['rounds'].append(current)
                if 'initial_detected_count' not in result and current['detected_count'] is not None:
                    result['initial_detected_count'] = current['detected_count']
                # Keep these first-observation aliases for existing diagnostics readers.
                if number == 1:
                    result.update(config_snapshots=current['config_snapshots'],
                        effective_vision_config=current['config_snapshots']['vision'],
                        detected_count=current['detected_count'], target_count=current['target_count'])
                kind = current['kind']
                if clear_session is not None:
                    if clear_session.verified_placed_count > budget.completed_count:
                        budget.completed_count = clear_session.verified_placed_count
                        budget.reobservations_without_progress = 0
                    if kind == 'completed':
                        continue  # Must observe after the final bowl too.
                    if kind == 'table_complete':
                        result.update(passed=True, outcome='completed' if budget.completed_count else 'no_targets',
                                      error='', placement_verified=bool(budget.completed_count),
                                      placement_region_separation_verified=True)
                        break
                    if kind == 'planned':
                        result.update(passed=True, outcome='planned', error='')
                        break
                    if kind == 'retry' and current.get('error') in ('confirming_source_empty', 'confirming_placement'):
                        continue  # Required completion observation; not a failure retry.
                    if kind == 'retry' and budget.retry_observation(time.monotonic(), record_failure=False):
                        continue
                    result.update(passed=False, error=current.get('error', 'pipeline_failed'),
                                  outcome='partial' if clear_session.motion_completed_count else 'failed')
                    if kind == 'retry':
                        result['retry_budget_exhausted'] = True
                    break
                has_rejections = any(current['rejected'].values())
                if current['completed_count']:
                    budget.complete_object(has_rejections)
                if kind == 'completed':
                    if single_bowl:
                        result.update(passed=not budget.unresolved_rejections,
                            outcome='completed' if not budget.unresolved_rejections else 'partial',
                            error='' if not budget.unresolved_rejections else 'prior_rejection_unresolved')
                        break
                    continue
                if kind == 'empty':
                    if single_bowl:
                        if budget.retry_observation(time.monotonic(), record_failure=False):
                            print('未检测到碗，重新采集识别（重试 '
                                  f'{budget.reobservations_without_progress}/{limits.max_reobservations}）',
                                  flush=True)
                            continue
                        result.update(passed=False, outcome='no_targets', error='single_bowl_required')
                        break
                    result['passed'], result['outcome'], result['error'] = budget.empty_result()
                    break
                if kind == 'planned':
                    result.update(passed=not has_rejections, outcome='planned' if not has_rejections else 'partial',
                                  error='' if not has_rejections else 'rejected_targets')
                    break
                if kind == 'retry' and budget.retry_observation(time.monotonic()):
                    continue
                result['error'] = ('reobservation_budget_exhausted' if kind == 'retry' else current.get('error', 'pipeline_failed'))
                result['outcome'] = ('partial' if budget.completed_count or
                    (not execute and current['preview_batches']) else 'failed')
                break
    except Exception as exc:
        result.update(passed=False, error=str(exc), outcome='partial' if budget.completed_count else 'failed')
    finally:
        result.update(completed_count=budget.completed_count, observation_count=budget.observation_count,
            returned_home=bool(execute and grasp.return_home_joint_positions is not None
                               and result['passed'] and budget.completed_count == 1),
            reobservations_without_progress=budget.reobservations_without_progress,
            unresolved_rejections=budget.unresolved_rejections,
            candidate_attempt_count=sum(r.get('candidate_attempt_count', 0) for r in result['rounds']),
            preview_batches=[p for r in result['rounds'] for p in r['preview_batches']],
            intermediate_previews=[p for r in result['rounds'] for p in r['intermediate_previews']])
        if clear_session is not None:
            result.update(clear_session.summary())
            # A failed later action must not inherit an earlier return-home claim.
            motions = [r for r in result['rounds'] if r.get('completed_count') or
                       r.get('statuses', {}).get('grasp_executor', {}).get('command_attempted')]
            result['returned_home'] = bool(motions and motions[-1].get('completed_count') == 1)
            result['completed_count'] = clear_session.verified_placed_count
            if not result['passed'] and clear_session.motion_completed_count:
                result['outcome'] = 'partial'
        result['stop_reason'] = result.get('error', '')
        if result['rounds']:
            last_round = result['rounds'][-1]
            for key in ('controller_preparation_log', 'controller_exit_code', 'controller_error_detail',
                        'grasp_pose_overlay', 'grasp_pose_overlay_status', 'selected_grasp_pose'):
                if key in last_round:
                    result[key] = last_round[key]
        if node is not None:
            node.destroy_node()
        if initialized:
            rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        if owns_output and not (output / 'result.json').exists():
            write_json(output / 'result.json', result, sources)
        if agent_decision_dir is not None and owns_output:
            atomic_json(agent_decision_dir.parent/'agent_status.json',
                        dict(state='finished', plan_id=task_policy['plan_id'], task_id=task_id), replace=True)
    if not result['passed']:
        print('任务失败：' + result.get('error', 'unknown_error'), flush=True)
        if result.get('controller_error_detail'):
            print(result['controller_error_detail'], flush=True)
        if result.get('controller_preparation_log'):
            print('控制器准备日志：' + result['controller_preparation_log'], flush=True)
    print('Result:', output / 'result.json', flush=True)
    if result.get('grasp_pose_overlay'):
        print('最终抓取 pose 图片：' + result['grasp_pose_overlay'], flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main('--execute' in os.sys.argv))
