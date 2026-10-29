"""Clear-table status integration with synthetic RGB-D and no ROS graph/hardware."""

from dataclasses import replace
import json
from types import SimpleNamespace as NS

import numpy as np
import pytest
from tf2_ros import TransformException

from test_retry import FakeNode, pair, result
from yolo_vision import main as vision
from yolo_vision.config import VisionConfig


def transform():
    return NS(transform=NS(translation=NS(x=.1, y=.2, z=.3),
                           rotation=NS(x=0., y=0., z=0., w=1.)))


def enable_tray(node, monkeypatch, status='ok'):
    node.config = replace(node.config, tray_config_path='/synthetic/tray.json')
    node.tray_config = {'frame_id': 'base'}
    node.tray_tf_error = None
    node.tf_calls, node.tray_calls, node.tray_model_calls = [], [], []
    node.tray_results = result()

    def predict_tray(bgr):
        node.tray_model_calls.append(bgr)
        return node.tray_results

    node.tray_segmenter = NS(predict_all=predict_tray)

    def lookup(*args):
        node.tf_calls.append(args)
        return transform()

    def detect(bgr, xyz, config, **kwargs):
        node.tray_calls.append((bgr, xyz, config, kwargs))
        return {'status': status, 'frame_id': 'base', 'reason': '' if status == 'ok' else 'tray_yolo_not_found'}

    def position(points, matrix, config):
        valid = np.isfinite(points).all(axis=1) & (points[:, 2] > 0)
        if not valid.any():
            return {'center_xy': None, 'radius': None, 'position_valid': False,
                    'frame_id': 'base', 'position_reason': 'insufficient_depth'}
        center = np.mean(points[valid], axis=0) + matrix[:3, 3]
        return {'center_xy': center[:2].tolist(), 'radius': .1,
                'position_valid': True, 'frame_id': 'base', 'position_reason': ''}

    node.tf_buffer = NS(lookup_transform=lookup)
    monkeypatch.setattr(vision, 'detect_tray', detect)
    monkeypatch.setattr(vision, 'estimate_object_position', position)
    return node


def test_legacy_observation_does_not_call_tray_or_change_payload(monkeypatch):
    monkeypatch.setattr(vision, 'detect_tray', lambda *a, **k: pytest.fail('legacy detected tray'))
    monkeypatch.setattr(vision, 'estimate_object_position', lambda *a, **k: pytest.fail('legacy estimated tray position'))
    node = FakeNode(monkeypatch, [result()])
    node.on_pair(*pair())
    assert node.state == 'success'
    assert 'tray' not in node.reports[-1]
    assert 'center_xy' not in node.reports[-1]['objects'][0]


@pytest.mark.parametrize('status', ['ok', 'not_found', 'invalid'])
def test_empty_bowls_still_report_same_frame_tray(monkeypatch, status):
    node = enable_tray(FakeNode(monkeypatch, [[]]), monkeypatch, status)
    frames = pair()
    node.on_pair(*frames)
    payload = node.reports[-1]
    assert node.state == 'no_targets' and not node.messages
    assert payload['detected_count'] == 0
    assert payload['tray']['status'] == status
    assert payload['source_stamp_ns'] == 99_000_000_042
    bgr, xyz, config, kwargs = node.tray_calls[0]
    assert xyz is node.xyz and config is node.tray_config
    assert kwargs['camera_matrix'] is None
    assert kwargs['inference_roi'] == node.config.inference_roi
    assert kwargs['detections'] is node.tray_results
    assert len(node.tray_model_calls) == 1 and node.tray_model_calls[0] is bgr
    assert kwargs['camera_to_base'][:3, 3] == pytest.approx([.1, .2, .3])
    target, source, stamp = node.tf_calls[0]
    assert (target, source) == ('base', 'camera')
    assert stamp.nanoseconds == 99_000_000_042


def test_tray_model_uses_same_frozen_rgb_and_never_adds_tray_to_bowl_cloud(monkeypatch):
    mask = np.zeros((3, 4), dtype=bool)
    mask[0, :2] = True
    node = enable_tray(FakeNode(monkeypatch, [result(mask)]), monkeypatch)
    bowl_frames = []
    original_predict = node.segmenter.predict_all

    def predict_bowls(bgr):
        bowl_frames.append(bgr)
        return original_predict(bgr)

    node.segmenter.predict_all = predict_bowls
    node.on_pair(*pair())
    assert node.state == 'success'
    assert bowl_frames[0] is node.tray_model_calls[0]
    from rim_locator.pointcloud import read_xyz_object_ids
    points, ids = read_xyz_object_ids(node.messages[0])
    assert len(points) == 2 and ids.tolist() == [0, 0]
    assert node.reports[-1]['detected_count'] == 1


def test_tray_inference_still_obeys_total_observation_deadline(monkeypatch):
    node = enable_tray(FakeNode(monkeypatch, [[]]), monkeypatch)

    def late(bgr):
        node.clock = 10.
        return node.tray_results

    node.tray_segmenter.predict_all = late
    node.on_pair(*pair())
    assert node.state == 'failed'
    assert node.reports[-1]['reason'] == 'input_timeout'
    assert not node.messages and not node.tray_calls


def test_expired_depth_decode_does_not_start_tray_inference(monkeypatch):
    node = enable_tray(FakeNode(monkeypatch, [[]]), monkeypatch)

    def late_depth(cloud):
        node.clock = 10.
        return node.xyz

    monkeypatch.setattr(vision, 'read_xyz', late_depth)
    node.on_pair(*pair())
    assert node.state == 'failed'
    assert node.reports[-1]['reason'] == 'input_timeout'
    assert not node.tray_model_calls and not node.tray_calls


def test_status_enriches_rejected_objects_without_changing_cloud(monkeypatch):
    mask = np.zeros((3, 4), dtype=bool)
    mask[:, :2] = True
    node = enable_tray(FakeNode(monkeypatch, [result(mask) + result(~mask)]), monkeypatch)
    node.xyz[:, :2] = 0.
    node.on_pair(*pair())
    payload = node.reports[-1]
    assert node.state == 'success'
    assert payload['detected_count'] == 2 and payload['object_count'] == 1
    accepted, rejected = payload['objects'][0], payload['rejected'][0]
    assert accepted['object_id'] == 1 and accepted['position_valid']
    assert accepted['center_xy'] == pytest.approx([1.1, 1.2])
    assert rejected['object_id'] == 0 and not rejected['position_valid']
    assert rejected['center_xy'] is None and rejected['radius'] is None
    assert rejected['reason'] == 'insufficient_pointcloud'
    assert rejected['position_reason'] == 'insufficient_depth'
    from rim_locator.pointcloud import read_xyz_object_ids
    points, object_ids = read_xyz_object_ids(node.messages[0])
    assert points == pytest.approx(np.ones((6, 3)))
    assert object_ids.tolist() == [1] * 6
    assert node.messages[0].header.frame_id == 'camera'
    assert node.messages[0].header.stamp == pair()[1].header.stamp


def test_all_invalid_depth_keeps_detected_objects_and_tray(monkeypatch):
    node = enable_tray(FakeNode(monkeypatch, [result()]), monkeypatch)
    node.xyz[:] = 0.
    node.on_pair(*pair())
    payload = node.reports[-1]
    assert node.state == 'failed' and not node.messages
    assert payload['reason'] == 'no_valid_depth_for_detected_targets'
    assert payload['tray']['status'] == 'ok'
    assert payload['detected_count'] == 1
    assert len(payload['rejected']) == 1
    assert not payload['rejected'][0]['position_valid']


def test_missing_tf_retries_fresh_pair_without_extending_budget(monkeypatch):
    node = enable_tray(FakeNode(monkeypatch, [result()]), monkeypatch)
    good_lookup = node.tf_buffer.lookup_transform

    def unavailable(*args):
        raise TransformException('no acquisition-time transform yet')

    node.tf_buffer.lookup_transform = unavailable
    node.on_pair(*pair())
    assert node.state == 'ready' and node.calls == 0
    assert not node.messages and not node.tray_calls and not node.tray_model_calls
    assert node.reports[-1]['tray']['reason'] == 'tray_tf_unavailable'
    assert node.reports[-1]['tray']['source_stamp_ns'] == 99_000_000_042
    assert 'source_stamp_ns' not in node.reports[-1]
    assert node.wait_started == 0.
    node.clock = 1.
    node.tf_buffer.lookup_transform = good_lookup
    node.on_pair(*pair())
    assert node.state == 'success' and node.calls == 1
    assert node.tray_tf_error is None and node.wait_started == 0.


def test_missing_tf_timeout_is_not_empty_table(monkeypatch):
    node = enable_tray(FakeNode(monkeypatch, [[]]), monkeypatch)

    def unavailable(*args):
        raise TransformException('missing TF')

    node.tf_buffer.lookup_transform = unavailable
    node.on_pair(*pair())
    node.clock = 10.
    node.check_timeout()
    assert node.state == 'failed' and node.calls == 0
    assert node.reports[-1]['reason'] == 'input_timeout'
    assert node.reports[-1]['tray']['reason'] == 'tray_tf_unavailable'
    assert not node.messages


def test_acquisition_transform_rotation_and_exact_frame():
    tf = transform()
    tf.transform.rotation.z = np.sqrt(.5)
    tf.transform.rotation.w = np.sqrt(.5)
    node = NS(tray_config={'frame_id': 'base'},
              tf_buffer=NS(lookup_transform=lambda *args: tf))
    matrix = vision.VisionNode.camera_to_base(node, pair()[0].header)
    assert matrix @ np.array([1., 0., 0., 1.]) == pytest.approx([.1, 1.2, .3, 1.])
    header = pair()[0].header
    header.frame_id = 'base'
    node.tf_buffer = NS(lookup_transform=lambda *args: pytest.fail('identity requested TF'))
    assert vision.VisionNode.camera_to_base(node, header) == pytest.approx(np.eye(4))


@pytest.mark.parametrize('value', ['', ' ', 1, False, []])
def test_optional_tray_path_requires_nonempty_string(value):
    assert VisionConfig().tray_config_path is None
    with pytest.raises(ValueError, match='tray_config_path'):
        VisionConfig(tray_config_path=value)


def test_uncalibrated_tray_fails_before_model_and_sensors(monkeypatch, tmp_path):
    config_path = tmp_path / 'vision.json'
    config_path.write_text(json.dumps({'tray_config_path': 'config/tray.json'}))
    messages = []
    monkeypatch.setattr(vision.Node, '__init__', lambda *args: None)
    monkeypatch.setattr(vision.Node, 'create_publisher', lambda *args, **kw: NS(publish=messages.append))
    monkeypatch.setattr(vision.Node, 'declare_parameter', lambda self, name, default: NS(
        value=str(config_path) if name == 'config_path' else default))
    monkeypatch.setattr(vision.Node, 'get_logger', lambda self: NS(info=lambda payload: None))
    monkeypatch.setattr(vision, 'get_package_share_directory', lambda name: str(tmp_path))
    monkeypatch.setattr(vision, 'Segmenter', lambda *args: pytest.fail('uncalibrated config reached model'))

    def uncalibrated(path, require_calibrated):
        assert path == tmp_path / 'config/tray.json'
        assert require_calibrated is True
        raise ValueError('tray_config_not_calibrated')

    monkeypatch.setattr(vision, 'load_tray_config', uncalibrated)
    node = vision.VisionNode()
    payload = json.loads(messages[-1].data)
    assert node.state == 'failed'
    assert payload['failed_step'] == 'initializing'
    assert payload['reason'] == 'tray_config_not_calibrated'


def test_missing_tray_model_fails_initialization_before_bowl_model_and_sensors(monkeypatch, tmp_path):
    config_path = tmp_path / 'vision.json'
    config_path.write_text(json.dumps({'tray_config_path': 'config/tray.json'}))
    messages, loads = [], []
    monkeypatch.setattr(vision.Node, '__init__', lambda *args: None)
    monkeypatch.setattr(vision.Node, 'create_publisher', lambda *args, **kw: NS(publish=messages.append))
    monkeypatch.setattr(vision.Node, 'declare_parameter', lambda self, name, default: NS(
        value=str(config_path) if name == 'config_path' else default))
    monkeypatch.setattr(vision.Node, 'get_logger', lambda self: NS(info=lambda payload: None))
    monkeypatch.setattr(vision, 'get_package_share_directory', lambda name: str(tmp_path))
    monkeypatch.setattr(vision, 'Buffer', lambda **kwargs: NS())
    monkeypatch.setattr(vision, 'TransformListener', lambda *args: NS())
    monkeypatch.setattr(vision, 'raw_subscriber', lambda *args: pytest.fail('missing tray model reached sensors'))
    monkeypatch.setattr(vision, 'load_tray_config', lambda *args, **kwargs: {
        'yolo': {'model_path': 'models/tray-seg.pt', 'target_class': 'tray',
                 'confidence': .5, 'imgsz': 640, 'device': 'cpu'}})
    real_segmenter = vision.Segmenter

    def load(path, config, **kwargs):
        loads.append((path, config, kwargs))
        return real_segmenter(path, config, **kwargs)

    monkeypatch.setattr(vision, 'Segmenter', load)
    node = vision.VisionNode()
    payload = json.loads(messages[-1].data)
    assert node.state == 'failed' and payload['failed_step'] == 'initializing'
    assert payload['reason'].startswith('segmentation_model_missing:')
    assert len(loads) == 1 and loads[0][0] == tmp_path / 'models/tray-seg.pt'
    assert loads[0][1].device == 'cpu' and loads[0][1].inference_roi is None
    assert loads[0][2] == {'require_target': True}


@pytest.mark.parametrize('case', ['empty', 'valid', 'invalid_depth'])
def test_agent_rgb_is_exact_segmentation_frame_even_without_valid_targets(monkeypatch, case):
    from copy import deepcopy
    node = enable_tray(FakeNode(monkeypatch, [[] if case == 'empty' else result()]), monkeypatch)
    if case == 'invalid_depth':
        node.xyz[:] = 0.
    published, segmented, encoded = [], [], []
    node.rgb_pub = NS(publish=published.append)
    node.bridge.cv2_to_imgmsg = lambda bgr, **kwargs: encoded.append(bgr) or NS(header=None)
    original = node.segmenter.predict_all
    node.segmenter.predict_all = lambda bgr: segmented.append(bgr) or original(bgr)
    frames = pair()
    frames[0].header = deepcopy(frames[0].header)
    frames[0].header.stamp.nanosec += 1000  # Synchronized pairs need not have equal stamps.
    node.on_pair(*frames)
    assert node.state == {'empty':'no_targets', 'valid':'success', 'invalid_depth':'failed'}[case]
    assert len(published) == 1 and encoded[0] is segmented[0]
    assert published[0].header == frames[0].header
    payload = node.reports[-1]
    assert payload['rgbd_pair']['rgb_stamp_ns'] == 99_000_001_042
    assert payload['rgbd_pair']['depth_stamp_ns'] == payload['source_stamp_ns'] == 99_000_000_042
    if case != 'empty':
        obj = payload['objects' if case == 'valid' else 'rejected'][0]
        assert obj['pixel_bbox'] == [0, 0, 3, 2]
