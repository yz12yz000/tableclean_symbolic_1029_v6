"""One joint inference preserves full-frame masks and separates grasp targets."""
from dataclasses import replace
from types import SimpleNamespace as NS
import sys

import numpy as np
import pytest

from yolo_vision.config import VisionConfig
from yolo_vision.segmentation import Segmenter, SegmentationResult, suppress_scene_duplicates


def config(**changes):
    return VisionConfig(**dict(inference_roi=[2, 1, 8, 5], scene_classes=['bowl', 'tray'],
                               device='cpu', **changes))


def tensor(array):
    return NS(cpu=lambda: NS(numpy=lambda: np.asarray(array)))


def test_near_identical_class_duplicates_removed_but_contained_utensil_kept():
    plate = np.zeros((20, 30), bool)
    plate[2:18, 2:28] = True
    fork = np.zeros_like(plate)
    fork[9:11, 4:26] = True
    scene = {'bowl': [SegmentationResult(plate.copy(), 'bowl', .4)],
             'plate': [SegmentationResult(plate, 'plate', .9)],
             'fork': [SegmentationResult(fork, 'fork', .8)]}
    result = suppress_scene_duplicates(scene)
    assert not result['bowl']
    assert len(result['plate']) == len(result['fork']) == 1


def test_plate_alias_preserves_each_container_mask_and_confidence_in_one_inference():
    segmenter = Segmenter.__new__(Segmenter)
    segmenter.config = VisionConfig(inference_roi=[2, 1, 8, 5], device='cpu',
        scene_classes=['bowl', 'tray', 'plate', 'spoon'], class_aliases={'plate': 'bowl'})
    masks = np.zeros((3, 4, 6), np.float32)
    masks[0, :, :2] = 1  # A real bowl.
    masks[1, :, 3:] = 1  # A separate plate, now another bowl.
    masks[2, 1, 4:6] = 1  # Spoon inside that plate stays independent.
    prediction = NS(names={0:'bowl', 1:'tray', 2:'plate', 3:'spoon'},
        boxes=NS(cls=tensor([0, 2, 3]), conf=tensor([.8, .9, .7])),
        masks=NS(data=[tensor(mask) for mask in masks]))
    calls = []
    segmenter.model = NS(predict=lambda **kw: calls.append(kw) or [prediction])
    image = np.zeros((8, 12, 3), np.uint8)
    scene = segmenter.predict_scene(image)
    assert len(calls) == 1 and set(scene) == {'bowl', 'tray', 'spoon'}
    assert len(scene['bowl']) == 2 and len(scene['spoon']) == 1
    assert [v.confidence for v in scene['bowl']] == [.9, .8]
    assert scene['bowl'][0].mask[1:5, 5:8].all()
    assert all(v.class_name == 'bowl' for v in scene['bowl'])
    assert len(segmenter.predict_all(image)) == 2
    assert len(calls) == 2  # Each invocation still uses exactly one inference.


@pytest.mark.parametrize('aliases', [[], {'plate':'unknown'}, {'plate':'plate'},
    {'plate':'bowl', 'bowl':'plate'}, {'bowl':'tray'}, {'plate':None}])
def test_invalid_aliases_rejected(aliases):
    with pytest.raises(ValueError, match='class_aliases'):
        VisionConfig(inference_roi=[2, 1, 8, 5], device='cpu',
            scene_classes=['bowl', 'tray', 'plate'], class_aliases=aliases)


def test_cup_alias_enters_bowl_callback_cloud_instead_of_obstacle_list(monkeypatch):
    from test_retry import FakeNode, pair
    from test_tray_observation import enable_tray
    from rim_locator.pointcloud import read_xyz_object_ids
    node = enable_tray(FakeNode(monkeypatch, []), monkeypatch)
    node.config = replace(node.config, scene_classes=['bowl', 'tray', 'cup'],
        inference_roi=[0,0,4,3], device='cpu', class_aliases={'cup':'bowl'})
    node.tray_config['yolo'] = {'target_class':'tray'}
    mask = np.zeros((3,4),np.float32)
    mask[:,1:3] = 1
    prediction = NS(names={0:'bowl',1:'tray',2:'cup'},
        boxes=NS(cls=tensor([2]),conf=tensor([.85])),
        masks=NS(data=[tensor(mask)]))
    segmenter = Segmenter.__new__(Segmenter)
    segmenter.config = node.config
    calls=[]
    segmenter.model=NS(predict=lambda **kw:calls.append(kw) or [prediction])
    node.segmenter=segmenter
    node.on_pair(*pair())
    assert node.state=='success' and len(calls)==1
    status=node.reports[-1]
    assert status['detected_count']==1 and status['class_name']=='bowl'
    assert status['perception']['class_counts']=={'bowl':1,'tray':0}
    assert status['obstacles']==[]
    points,ids=read_xyz_object_ids(node.messages[0])
    assert len(points)==6 and ids.tolist()==[0]*6


def test_one_network_call_restores_both_overlapping_classes_in_original_pixels():
    segmenter = Segmenter.__new__(Segmenter)
    segmenter.config = config()
    image = np.zeros((8, 12, 3), np.uint8)
    image[1:5, 2:8] = 42
    masks = np.zeros((2, 4, 6), dtype=np.float32)
    masks[0, 1:3, 1:3] = 1
    masks[1, :, :] = 1
    predictions = NS(names={0: 'bowl', 1: 'tray'},
                     boxes=NS(cls=tensor([0, 1]), conf=tensor([.8, .9])),
                     masks=NS(data=[tensor(mask) for mask in masks]))
    calls = []
    def predict(**kwargs):
        calls.append(kwargs)
        return [predictions]
    segmenter.model = NS(predict=predict)
    scene = segmenter.predict_scene(image)
    assert len(calls) == 1
    assert np.array_equal(calls[0]['source'], image[1:5, 2:8])
    assert calls[0]['rect'] is False and calls[0]['retina_masks'] is True
    assert calls[0]['agnostic_nms'] is False
    bowl, tray = scene['bowl'][0], scene['tray'][0]
    expected = np.zeros((8, 12), bool)
    expected[2:4, 3:5] = True
    assert np.array_equal(bowl.mask, expected)
    assert tray.mask.sum() == 24 and np.all(tray.mask[bowl.mask])
    assert bowl.mask.dtype == tray.mask.dtype == np.bool_


def test_tray_threshold_is_independent_from_bowl_and_uses_one_inference():
    segmenter = Segmenter.__new__(Segmenter)
    segmenter.config = config(class_confidences={'tray': .5})
    masks = np.zeros((3,4,6),np.float32)
    masks[0,:,0] = 1
    masks[1,:,2] = 1
    masks[2,:,4] = 1
    prediction = NS(names={0:'bowl',1:'tray'},
        boxes=NS(cls=tensor([0,1,1]),conf=tensor([.3,.49,.5])),
        masks=NS(data=[tensor(mask) for mask in masks]))
    calls=[]
    segmenter.model=NS(predict=lambda **kw:calls.append(kw) or [prediction])
    scene=segmenter.predict_scene(np.zeros((8,12,3),np.uint8))
    assert len(calls)==1 and calls[0]['conf']==.25
    assert len(scene['bowl'])==1 and scene['bowl'][0].confidence==.3
    assert len(scene['tray'])==1 and scene['tray'][0].confidence==.5


@pytest.mark.parametrize('thresholds',[{'tray':0},{'tray':True},{'tray':float('nan')},
                                      {'unknown':.5},[]])
def test_invalid_class_threshold_rejected(thresholds):
    with pytest.raises(ValueError,match='class_confidences'):
        config(class_confidences=thresholds)


@pytest.mark.parametrize('change,error', [
    ({'scene_classes': ['bowl']}, 'scene_classes'),
    ({'scene_classes': ['bowl', 'bowl']}, 'scene_classes'),
    ({'scene_classes': ['tray', 'plate']}, 'scene_classes'),
    ({'inference_roi': None}, 'table_roi'),
    ({'inference_tile_size': 512}, 'no_tiles'),
])
def test_invalid_joint_settings_fail_before_inference(change, error):
    values = dict(inference_roi=[2, 1, 8, 5], scene_classes=['bowl', 'tray'], device='cpu')
    values.update(change)
    with pytest.raises(ValueError, match=error):
        VisionConfig(**values)


def test_joint_checkpoint_missing_a_class_fails_at_initialization(monkeypatch, tmp_path):
    path = tmp_path / 'wrong.pt'
    path.touch()
    monkeypatch.setitem(sys.modules, 'ultralytics', NS(
        YOLO=lambda *a, **kw: NS(task='segment', names={0: 'tray'})))
    with pytest.raises(ValueError, match='missing_target_class:bowl'):
        Segmenter(path, config())


def test_joint_inference_empty_classes_and_outside_roi(monkeypatch):
    segmenter = Segmenter.__new__(Segmenter)
    segmenter.config = config()
    segmenter.model = NS(predict=lambda **kw: [NS(boxes=None)])
    assert segmenter.predict_scene(np.zeros((8, 12, 3), np.uint8)) == {'bowl': [], 'tray': []}
    with pytest.raises(ValueError, match='roi_outside_image'):
        segmenter.predict_scene(np.zeros((2, 3, 3), np.uint8))


@pytest.mark.parametrize('bowl_count', [0, 1])
def test_real_callback_calls_joint_model_once_and_never_publishes_tray_cloud(monkeypatch, bowl_count):
    from test_retry import FakeNode, pair, result
    from test_tray_observation import enable_tray
    from rim_locator.pointcloud import read_xyz_object_ids
    node = enable_tray(FakeNode(monkeypatch, []), monkeypatch)
    node.config = replace(node.config, scene_classes=['bowl', 'tray'], inference_roi=[0, 0, 4, 3], device='cpu')
    node.tray_config['yolo'] = {'target_class': 'tray'}
    mask = np.zeros((3, 4), bool)
    mask[:, :2] = True
    calls = []
    def predict(bgr):
        calls.append(bgr)
        return {'bowl': result(mask) if bowl_count else [], 'tray': result()}
    node.segmenter = NS(predict_scene=predict)
    node.tray_segmenter.predict_all = lambda *args: pytest.fail('second model called')
    node.on_pair(*pair())
    assert len(calls) == 1
    assert node.state == ('success' if bowl_count else 'no_targets')
    assert node.reports[-1]['perception']['model_inferences'] == 1
    assert node.reports[-1]['perception']['class_counts'] == {'bowl': bowl_count, 'tray': 1}
    assert node.reports[-1]['detected_count'] == bowl_count
    assert node.reports[-1]['source_stamp_ns'] == 99_000_000_042
    assert not node.tray_model_calls
    if bowl_count:
        points, ids = read_xyz_object_ids(node.messages[0])
        assert len(points) == 6 and ids.tolist() == [0]*6
    else:
        assert not node.messages


def test_joint_callback_timeout_does_not_decode_depth_or_publish(monkeypatch):
    from test_retry import FakeNode, pair
    from test_tray_observation import enable_tray
    node = enable_tray(FakeNode(monkeypatch, []), monkeypatch)
    node.config = replace(node.config, scene_classes=['bowl', 'tray'], inference_roi=[0, 0, 4, 3], device='cpu')
    def late(bgr):
        node.clock = 10.
        return {'bowl': [], 'tray': []}
    node.segmenter = NS(predict_scene=late)
    node.on_pair(*pair())
    assert node.state == 'failed' and node.reports[-1]['reason'] == 'input_timeout'
    assert not node.messages and not node.tray_calls and not node.tray_model_calls


def test_cup_is_reported_for_occupancy_and_removed_from_tray_depth_not_grasp_cloud(monkeypatch):
    from test_retry import FakeNode, pair, result
    from test_tray_observation import enable_tray
    from rim_locator.pointcloud import read_xyz_object_ids
    node = enable_tray(FakeNode(monkeypatch, []), monkeypatch)
    node.config = replace(node.config, scene_classes=['bowl', 'tray', 'cup'],
                          inference_roi=[0,0,4,3], device='cpu')
    node.tray_config.update(yolo={'target_class':'tray'}, position_tolerance=.025)
    node.xyz = np.ones((3,4,3))
    bowl_mask = np.zeros((3,4),bool)
    bowl_mask[0,:2] = True
    cup_mask = np.ones((3,4),bool)
    cup_mask[0,:2] = False
    for row in range(3):
        for col in range(4):
            node.xyz[row,col,:2] = [col*.01,row*.01]
    node.segmenter = NS(predict_scene=lambda bgr: {'bowl':result(bowl_mask),
        'tray':result(), 'cup':result(cup_mask)})
    node.on_pair(*pair())
    assert node.state == 'success', node.reports[-1]
    obstacles = node.reports[-1]['obstacles']
    assert len(obstacles)==1 and obstacles[0]['class_name']=='cup'
    assert obstacles[0]['position_valid']
    assert len(node.tray_calls[0][3]['occluder_masks'])==2
    points,ids=read_xyz_object_ids(node.messages[0])
    assert len(points)==2 and ids.tolist()==[0,0]


@pytest.mark.parametrize('mutation,reason,model_count', [
    (None, 'test_sensor_boundary', 1),
    ('model', 'scene_tray_model_must_match_vision_model', 0),
    ('confidence', 'scene_tray_inference_settings_must_match_vision', 0),
    ('class', 'scene_inference_requires_distinct_bowl_and_tray_classes', 0),
])
def test_joint_initialization_loads_one_model_and_rejects_conflicting_settings(
        monkeypatch, tmp_path, mutation, reason, model_count):
    import json
    from yolo_vision import main as vision
    path = tmp_path / 'vision.json'
    path.write_text(json.dumps(dict(tray_config_path='tray.json',
        scene_classes=['bowl', 'tray'], inference_roi=[220, 300, 1000, 700],
        model_path='scene.pt', device='cpu', publish_debug=False)))
    tray = dict(yolo=dict(model_path='scene.pt', target_class='tray', confidence=.25,
                          imgsz=512, device='cpu'))
    if mutation == 'model':
        tray['yolo']['model_path'] = 'other.pt'
    elif mutation == 'confidence':
        tray['yolo']['confidence'] = .5
    elif mutation == 'class':
        tray['yolo']['target_class'] = 'bowl'
    loads, messages = [], []
    monkeypatch.setattr(vision.Node, '__init__', lambda *a: None)
    monkeypatch.setattr(vision.Node, 'create_publisher', lambda *a, **kw: NS(publish=messages.append))
    monkeypatch.setattr(vision.Node, 'declare_parameter', lambda self, name, default: NS(
        value=str(path) if name == 'config_path' else default))
    monkeypatch.setattr(vision.Node, 'get_logger', lambda self: NS(info=lambda payload: None))
    monkeypatch.setattr(vision, 'get_package_share_directory', lambda *a: str(tmp_path))
    monkeypatch.setattr(vision, 'load_tray_config', lambda *a, **kw: tray)
    monkeypatch.setattr(vision, 'Buffer', lambda **kw: NS())
    monkeypatch.setattr(vision, 'TransformListener', lambda *a: NS())
    monkeypatch.setattr(vision, 'Segmenter', lambda *a, **kw: loads.append(a) or NS())
    def sensor_boundary(*args, **kwargs):
        raise RuntimeError('test_sensor_boundary')
    monkeypatch.setattr(vision, 'raw_subscriber', sensor_boundary)
    node = vision.VisionNode()
    assert node.state == 'failed'
    assert json.loads(messages[-1].data)['reason'] == reason
    assert len(loads) == model_count
    assert node.tray_segmenter is None
    if mutation is None:
        assert loads[0][0] == tmp_path/'scene.pt'
