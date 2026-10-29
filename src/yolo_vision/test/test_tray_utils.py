"""Metric tray localization and conservative occupancy, entirely synthetic."""

from copy import deepcopy
import math
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from yolo_vision.tray_utils import (
    choose_slot, classify_object, detect_tray, estimate_object_position,
    load_tray_config, rotate_xy, slot_xy, tray_polygon, validate_objects,
    validate_tray_config, validate_tray_observation,
)


@pytest.fixture
def config():
    c = load_tray_config(Path(__file__).parents[1] / "config/clear_table_tray.json")
    c["calibrated"] = True
    c["source_polygon"] = [[-2., -2.], [2., -2.], [2., 2.], [-2., 2.]]
    return c


def scene(config, center=(0.5, 0.0), yaw=0.):
    return {"status": "ok", "frame_id": "base", "center_xy": list(center), "yaw": yaw,
            "size": config["tray_size"], "identity": "yolo:tray"}


def bowl(key, xy, radius=.075):
    return {"object_id": key, "center_xy": list(xy), "radius": radius,
            "position_valid": True, "frame_id": "base"}


@pytest.mark.parametrize('update', [
    {'known_bowl_radii': [.06, .06]}, {'known_bowl_radii': [.02]},
    {'known_bowl_radii': [float('nan')]}, {'known_bowl_radii': 'small'},
    {'compensate_bowl_xy': True}, {'compensate_bowl_xy': 1},
])
def test_measured_bowl_config_rejects_invalid_calibration(config, update):
    config.update(update)
    with pytest.raises(ValueError, match='invalid_tray:'):
        validate_tray_config(config)


def synthetic_tray(config, angle=0., translation=(0., 0.), camera_tilt=0.):
    """A rectangular YOLO mask and exact registered planar RGB-D; no model."""
    focal, center = 500., np.array([.4, .2])
    yy, xx = np.indices((400, 400))
    rays = np.stack([(xx - 199.5) / focal, (yy - 199.5) / focal, np.ones_like(xx)], axis=-1)
    tilt = math.radians(camera_tilt)
    co, si = math.cos(tilt), math.sin(tilt)
    rotation = np.array([[co, 0, si], [0, 1, 0], [-si, 0, co]]) @ np.diag([1., -1., -1.])
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = [.4 + math.tan(tilt), .2, 1.2]
    world_rays = rays @ rotation.T
    xyz = rays * (-1. / world_rays[:, :, 2, None])
    world = xyz @ rotation.T + transform[:3, 3]
    center = center + np.array([translation[0] / focal, -translation[1] / focal])
    theta = math.radians(angle)
    offset = world[:, :, :2] - center
    local_u = offset[:, :, 0] * math.cos(theta) + offset[:, :, 1] * math.sin(theta)
    local_v = -offset[:, :, 0] * math.sin(theta) + offset[:, :, 1] * math.cos(theta)
    mask = ((np.abs(local_u) <= config['tray_outer_size'][0] / 2)
            & (np.abs(local_v) <= config['tray_outer_size'][1] / 2))
    image = np.full((400, 400, 3), 220, np.uint8)
    image[mask] = [150, 80, 40]
    matrix = np.array([[focal, 0, 199.5], [0, focal, 199.5], [0, 0, 1.]])
    detections = [SimpleNamespace(mask=mask, class_name='tray', confidence=.93)]
    return image, xyz, matrix, transform, detections


def test_bowl_occlusion_uses_visible_tray_plane_and_supported_hidden_corner(config):
    config['source_polygon'] = None
    image, xyz, matrix, transform, detections = synthetic_tray(config)
    rows, cols = np.nonzero(detections[0].mask)
    yy, xx = np.indices(image.shape[:2])
    occluder = (xx - cols.min())**2 + (yy - rows.min())**2 < 50**2
    xyz[occluder] *= .8  # bowl height must not bias the tray plane
    with_bowl = detect_tray(image, xyz, config, matrix, transform, detections=detections,
                            occluder_masks=[occluder])
    assert with_bowl['status'] == 'ok', with_bowl
    assert with_bowl['outline_supported_by_bowl_occlusion']
    assert with_bowl['center_xy'] == pytest.approx([.4, .2], abs=.003)
    # A missing corner without an observed bowl must still fail.
    detections[0].mask &= ~occluder
    missing = detect_tray(image, xyz, config, matrix, transform, detections=detections)
    assert missing['status'] == 'invalid'


@pytest.fixture
def visible_config(config):
    config.update(tray_size=[.18, .12], tray_outer_size=[.20, .14],
                  bowl_radius=.01, edge_margin=.003, position_tolerance=.001, clearance=.003,
                  slots=[{'id': 'A', 'uv': [-.045, -.025]}, {'id': 'B', 'uv': [.045, .025]}],
                  source_polygon=[[.10, -.10], [.70, -.10], [.70, .50], [.10, .50]])
    return validate_tray_config(config, require_calibrated=True)


def test_template_is_readable_but_cannot_plan_live():
    path = Path(__file__).parents[1] / "config/clear_table_tray.json"
    assert not load_tray_config(path)["calibrated"]
    with pytest.raises(ValueError, match="not_calibrated"):
        load_tray_config(path, require_calibrated=True)


def test_configuration_normalizes_without_mutating_input(config):
    old = deepcopy(config)
    validated = validate_tray_config(config, require_calibrated=True)
    validated["slots"][0]["uv"][0] = 999
    assert config == old


@pytest.mark.parametrize("key,value", [
    ("schema_version", True), ("calibrated", "true"), ("frame_id", "base_link"),
    ("detector", "other"), ("bowl_radius", float("nan")), ("clearance", -1),
    ("position_tolerance", float("inf")), ("edge_margin", True),
    ("release_z", float("nan")), ("release_xy_offset", [0, float("inf")]),
    ("empty_confirmations", 1), ("tray_size", [1, -1]),
    ("source_polygon", [[0, 0], [1, 0], [0, 0]]),
    ("source_polygon", [[0, 0], [1, 1], [1, 0], [0, 1]]),
])
def test_bad_config_rejected(config, key, value):
    config[key] = value
    with pytest.raises(ValueError, match="invalid_tray"):
        validate_tray_config(config)


def test_source_region_mandatory_for_live(config):
    config["source_polygon"] = None
    with pytest.raises(ValueError, match="source_polygon"):
        validate_tray_config(config, require_calibrated=True)


@pytest.mark.parametrize("fault", ["id", "position", "edge"])
def test_bad_slots_rejected(config, fault):
    if fault == "id":
        config["slots"][1]["id"] = "A"
    elif fault == "position":
        config["slots"][1]["uv"] = config["slots"][0]["uv"]
    else:
        config["slots"][0]["uv"] = [.3, 0]
    with pytest.raises(ValueError, match="invalid_tray"):
        validate_tray_config(config)


@pytest.mark.parametrize("fault", ["duplicate", "nan", "frame", "missing", "bool_id", "radius"])
def test_malformed_objects_fail_closed(config, fault):
    objects = [bowl(0, [.5, 0])]
    if fault == "duplicate":
        objects.append(deepcopy(objects[0]))
    elif fault == "nan":
        objects[0]["center_xy"][0] = float("nan")
    elif fault == "frame":
        objects[0]["frame_id"] = "base_link"
    elif fault == "missing":
        del objects[0]["position_valid"]
    elif fault == "bool_id":
        objects[0]["object_id"] = True
    else:
        objects[0]["radius"] = -.1
    with pytest.raises(ValueError, match="invalid_tray"):
        validate_objects(objects, config)


@pytest.mark.parametrize("fault", ["nan", "identity", "size", "polygon", "frame"])
def test_malformed_tray_rejected(config, fault):
    tray = scene(config)
    if fault == "nan":
        tray["yaw"] = float("nan")
    elif fault == "identity":
        tray["identity"] = "yolo:other_tray"
    elif fault == "size":
        tray["size"] = [2, 2]
    elif fault == "polygon":
        tray["polygon_xy"] = [[0, 0], [1, 0], [1, 1], [0, 1]]
    else:
        tray["frame_id"] = "camera"
    with pytest.raises(ValueError, match="invalid_tray"):
        validate_tray_observation(tray, config)


def test_region_uses_footprint_and_current_tray_pose(config):
    tray = scene(config)
    assert classify_object(bowl(0, [.5, 0]), tray, config) == "inside"
    assert classify_object(bowl(1, [-.5, 0]), tray, config) == "outside"
    assert classify_object(bowl(2, [.8, 0]), tray, config) == "unknown"
    assert classify_object(bowl(3, [2., 0]), tray, config) == "unknown"
    assert classify_object(bowl(4, [2.2, 0]), tray, config) == "out_of_source"
    moved = scene(config, (-.5, 0), math.pi / 2)
    assert classify_object(bowl(1, [-.5, 0]), moved, config) == "inside"
    assert classify_object(bowl(0, [.5, 0]), moved, config) == "outside"


def test_slot_tracks_rotation_and_translation(config):
    tray = scene(config, (1., 2.), math.pi / 2)
    assert slot_xy(tray, "A", config) == pytest.approx([1.1, 1.84])
    polygon = tray_polygon([1., 2.], math.pi / 2, config["tray_size"])
    assert polygon[0] == pytest.approx([1.22, 1.7])
    assert rotate_xy([.2, 0], math.pi) == pytest.approx([-.2, 0])


def test_occupancy_and_used_positions_never_reused(config):
    tray = scene(config)
    a = slot_xy(tray, "A", config)
    assert choose_slot(tray, [bowl(99, a)], set(), config)["id"] == "B"
    # A disappearing bowl still reserves its relative location, including after movement.
    moved = scene(config, (.8, .4), .3)
    assert choose_slot(moved, [], {"A", "B"}, config)["id"] == "C"
    assert choose_slot(moved, [], {"A", "B", "C", "D"}, config) is None
    assert choose_slot(tray, [bowl(0, [.5, -.1], radius=.2)], set(), config) is None


def test_missing_depth_cannot_make_tray_empty(config):
    obj = {"object_id": 1, "position_valid": False, "frame_id": "base", "center_xy": None, "radius": None}
    assert classify_object(obj, scene(config), config) == "unknown"
    assert choose_slot(scene(config), [obj], set(), config) is None


def test_used_slot_reserves_footprint_even_if_neighboring_slot_has_another_id(config):
    config["slots"][1]["uv"] = [-.12, -.10]
    config = validate_tray_config(config)
    # B is different from A but only 4 cm away: missing the A bowl must not free B.
    assert choose_slot(scene(config), [], {"A"}, config)["id"] == "C"


def test_rgbd_bowl_center_is_not_a_grasp_contact(config):
    theta = np.linspace(0, 2 * np.pi, 80, endpoint=False)
    points = np.stack([.03 + .06 * np.cos(theta), .02 + .06 * np.sin(theta), np.ones(80)], axis=1)
    transform = np.eye(4)
    transform[:3, 3] = [.1, .2, 0]
    result = estimate_object_position(points, transform, config)
    assert result["position_valid"]
    assert result["center_xy"] == pytest.approx([.13, .22])
    assert result["radius"] == pytest.approx(.075)


def test_object_disk_encloses_every_point_without_box_midpoint_inflation(config):
    # The lower tip of an oblique mask moves a box midpoint away from the
    # actual enclosing-circle centre. No sample may be trimmed as an outlier.
    triangle = np.array([[-.06, 0., 1.], [.06, 0., 1.], [.025, .045, 1.]])
    points = np.repeat(triangle, 20, axis=0)
    result = estimate_object_position(points, np.eye(4), config)
    assert result['position_valid']
    assert result['center_xy'] == pytest.approx([0., 0.], abs=1e-6)
    assert result['radius'] == pytest.approx(.075, abs=1e-6)
    distances = np.linalg.norm(points[:, :2] - result['center_xy'], axis=1)
    assert np.max(distances) <= result['radius'] - config['position_tolerance'] + 1e-8


@pytest.mark.parametrize('value', [0, -.001, True, float('nan'), .0101, .016])
def test_depth_noise_allowance_has_a_hard_limit(config, value):
    config['depth_xy_tolerance'] = value
    with pytest.raises(ValueError, match='depth_xy_tolerance'):
        validate_tray_config(config)


def test_extra_depth_noise_must_fit_position_allowance(config):
    config.update(position_tolerance=.007, depth_xy_tolerance=.008)
    with pytest.raises(ValueError, match='exceeds_position_allowance'):
        validate_tray_config(config)
    config['position_tolerance'] = .015
    assert validate_tray_config(config)['depth_xy_tolerance'] == .008


def test_observed_boundary_limits_both_occupancy_and_destinations(config):
    tray = scene(config)
    tray['observed_polygon_xy'] = [[.28, -.22], [.8, -.22], [.8, .22], [.28, .22]]
    # A lies in the nominal rectangle but its disk crosses the observed edge.
    assert classify_object(bowl(0, [.34, -.1]), tray, config) == 'unknown'
    assert classify_object(bowl(1, [.66, -.1]), tray, config) == 'inside'
    assert choose_slot(tray, [], set(), config)['id'] == 'B'


def test_rounded_tray_uses_actual_disk_support_instead_of_empty_corners(visible_config):
    visible_config['tray_size'] = [.195, .135]
    image, xyz, matrix, transform, detections = synthetic_tray(visible_config)
    world = xyz @ transform[:3, :3].T + transform[:3, 3]
    distance = np.maximum(np.abs(world[:, :, :2] - [.4, .2]) - [.08, .05], 0)
    detections[0].mask &= (distance ** 2).sum(axis=2) <= .02 ** 2
    result = detect_tray(image, xyz, visible_config, camera_matrix=matrix,
                         camera_to_base=transform, detections=detections)
    assert result['status'] == 'ok', result
    assert 'observed_polygon_xy' in result
    assert choose_slot(result, [], set(), visible_config) is not None


def test_optional_boundary_refinement_recovers_small_background_leak(visible_config):
    image, xyz, matrix, transform, detections = synthetic_tray(visible_config)
    original_image = image.copy()
    detections[0].mask = cv2.dilate(detections[0].mask.astype(np.uint8),
                                   np.ones((7, 7), np.uint8)).astype(bool)
    original_mask = detections[0].mask.copy()
    visible_config['refine_boundary'] = True
    result = detect_tray(image, xyz, visible_config, camera_matrix=matrix,
                         camera_to_base=transform, detections=detections)
    assert result['status'] == 'ok', result
    assert result['boundary_refined'] is True
    assert result['center_xy'] == pytest.approx([.4, .2], abs=.002)
    assert np.array_equal(image, original_image)
    assert np.array_equal(detections[0].mask, original_mask)
    assert detect_tray(image, xyz, visible_config, camera_to_base=transform,
                       detections=[])['reason'] == 'tray_yolo_not_found'


def test_boundary_refinement_cannot_expand_into_a_different_object(visible_config, monkeypatch):
    image, xyz, _, transform, detections = synthetic_tray(visible_config)
    visible_config['refine_boundary'] = True
    def oversized(image, labels, *args):
        labels[:] = cv2.GC_FGD
    monkeypatch.setattr(cv2, 'grabCut', oversized)
    result = detect_tray(image, xyz, visible_config, camera_to_base=transform, detections=detections)
    assert result['reason'] == 'tray_boundary_refinement_excessive_change'


def test_boundary_refinement_flag_must_be_boolean(config):
    config['refine_boundary'] = 'true'
    with pytest.raises(ValueError, match='refine_boundary'):
        validate_tray_config(config)


def test_measured_noise_allowance_keeps_default_and_rejects_larger_errors(visible_config):
    image, xyz, matrix, transform, detections = synthetic_tray(visible_config)
    # Symmetric alternating sub-centimetre XY noise: the fitted pose is stable
    # but the original strict default rejects the individual depth residuals.
    xyz[::2, :, 0] += .007
    xyz[1::2, :, 0] -= .007
    kwargs = dict(camera_matrix=matrix, camera_to_base=transform, detections=detections)
    assert detect_tray(image, xyz, visible_config, **kwargs)['reason'] == 'tray_inconsistent_depth'
    visible_config.update(position_tolerance=.015, depth_xy_tolerance=.008)
    visible_config = validate_tray_config(visible_config)
    result = detect_tray(image, xyz, visible_config, **kwargs)
    assert result['status'] == 'ok', result
    assert .006 < result['depth_xy_residual_p95'] < .008
    assert result['center_xy'] == pytest.approx([.4, .2], abs=.001)
    xyz[::2, :, 0] += .005
    xyz[1::2, :, 0] -= .005
    assert detect_tray(image, xyz, visible_config, **kwargs)['reason'] == 'tray_inconsistent_depth'


@pytest.mark.parametrize("fault", ["empty", "nan", "sparse", "tiny", "outlier", "transform"])
def test_unreliable_object_depth_remains_unresolved(config, fault):
    points = np.tile([.1, .2, 1.], (100, 1))
    points[:, 0] += np.linspace(-.06, .06, 100)
    matrix = np.eye(4)
    if fault == "empty":
        points = np.empty((0, 3))
    elif fault == "nan":
        points[:] = np.nan
    elif fault == "sparse":
        points[:80] = np.nan
    elif fault == "tiny":
        points[:] = [.1, .2, 1.]
    elif fault == "outlier":
        points[0] = [3., 3., 1.]
    else:
        matrix[0, 0] = 2
    result = estimate_object_position(points, matrix, config)
    assert not result["position_valid"]
    assert result["center_xy"] is None


@pytest.mark.parametrize("angle,translation,camera_tilt", [
    (0, (0, 0), 0), (32, (37, -20), 0), (-73, (-40, 30), 0),
    (32, (20, -10), 20), (179, (0, 0), 0),
])
def test_yolo_mask_follows_metric_pose(visible_config, angle, translation, camera_tilt):
    image, xyz, matrix, transform, detections = synthetic_tray(visible_config, angle, translation, camera_tilt)
    result = detect_tray(image, xyz, visible_config, camera_matrix=matrix, camera_to_base=transform,
                         detections=detections)
    assert result["status"] == "ok", result
    expected_center = [.4 + translation[0] / matrix[0, 0], .2 - translation[1] / matrix[1, 1]]
    assert result["center_xy"] == pytest.approx(expected_center, abs=.003)
    expected_yaw = (math.radians(angle) + math.pi / 2) % math.pi - math.pi / 2
    assert result["yaw"] == pytest.approx(expected_yaw, abs=.015)
    assert result["identity"] == "yolo:tray"
    assert result["size"] == visible_config["tray_size"]
    assert len(result["polygon_xy"]) == 4
    assert result["yaw_modulo_pi"]


@pytest.mark.parametrize('roi,reason', [
    ([170, 0, 400, 400], 'tray_outside_detection_view'),
    ([60, 60, 375, 375], 'source_outside_detection_view'),
])
def test_visible_tray_but_cropped_bowl_detection_region_is_rejected(visible_config, roi, reason):
    image, xyz, _, transform, detections = synthetic_tray(visible_config)
    result = detect_tray(image, xyz, visible_config, camera_to_base=transform,
                         inference_roi=roi, detections=detections)
    assert result['status'] == 'invalid'
    assert result['reason'] == reason


def test_source_region_outside_camera_cannot_be_declared_empty(visible_config):
    image, xyz, _, transform, detections = synthetic_tray(visible_config)
    visible_config['source_polygon'][0][0] = -.5
    result = detect_tray(image, xyz, visible_config, camera_to_base=transform, detections=detections)
    assert result['status'] == 'invalid'
    assert result['reason'] == 'source_outside_detection_view'


def test_tray_and_source_inside_bowl_roi_keep_pixel_diagnostics(visible_config):
    image, xyz, _, transform, detections = synthetic_tray(visible_config)
    result = detect_tray(image, xyz, visible_config, camera_to_base=transform,
                         inference_roi=[25, 25, 375, 375], detections=detections)
    assert result['status'] == 'ok', result
    assert len(result['tray_polygon_px']) == len(result['source_polygon_px']) == 4
    assert len(result['tray_outer_polygon_px']) == 4


def test_bowl_occlusion_inside_tray_does_not_change_outer_pose(visible_config):
    image, xyz, _, transform, detections = synthetic_tray(visible_config, angle=20)
    detections[0].mask[190:210, 185:215] = False
    result = detect_tray(image, xyz, visible_config, camera_to_base=transform, detections=detections)
    assert result['status'] == 'ok', result
    assert result['center_xy'] == pytest.approx([.4, .2], abs=.003)


def test_actual_incoming_radius_controls_obstacle_clearance(config):
    tray = scene(config)
    # A is occupied, B has been used. C is 18 cm from an existing bowl; it
    # accommodates the default radius but cannot accept the larger source.
    objects = [bowl(0, [.34, -.08])]
    assert choose_slot(tray, objects, {'B'}, config)['id'] == 'C'
    assert choose_slot(tray, objects, {'B'}, config, incoming_radius=.09)['id'] == 'D'


@pytest.mark.parametrize("fault,reason", [
    ("missing", "tray_yolo_not_found"), ("low_confidence", "tray_yolo_not_found"),
    ("wrong_class", "tray_yolo_not_found"), ("mask", "tray_mask_missing_or_invalid"),
    ("no_detections", "tray_yolo_detections_missing"), ("depth", "tray_insufficient_depth"),
    ("sparse", "tray_insufficient_depth"), ("inconsistent", "tray_inconsistent_depth"),
    ("scale", "tray_outer_size_mismatch"), ("tilt", "tray_not_horizontal"),
    ("duplicate", "tray_yolo_ambiguous"), ("transform", "camera_to_base"),
    ("truncated", "tray_mask_truncated"), ("missing_corner", "tray_outline_incomplete"),
    ("confidence_nan", "confidence"), ("confidence_high", "tray_invalid_confidence"),
    ("square", "tray_yaw_ambiguous"),
])
def test_unreliable_yolo_tray_fails_closed(visible_config, fault, reason):
    image, xyz, _, transform, detections = synthetic_tray(visible_config)
    if fault == "missing":
        detections = []
    elif fault == "low_confidence":
        detections[0].confidence = .1
    elif fault == "wrong_class":
        detections[0].class_name = 'bowl'
    elif fault == "no_detections":
        detections = None
    elif fault == "mask":
        detections[0].mask = None
    elif fault == "depth":
        xyz[:] = np.nan
    elif fault == "sparse":
        xyz[::2] = np.nan
    elif fault == "inconsistent":
        xyz[::2, :, 2] += .2
    elif fault == "scale":
        xyz[:, :, :2] *= 2
    elif fault == "tilt":
        xyz[:, :, 2] += xyz[:, :, 0] * .5
    elif fault == "duplicate":
        detections.append(deepcopy(detections[0]))
    elif fault == "transform":
        transform[0, 0] = 2
    elif fault == "truncated":
        detections[0].mask[0:200, 199:201] = True
    elif fault == "missing_corner":
        # Large diagonal cut leaves the bounding rectangle unchanged.
        yy, xx = np.indices(image.shape[:2])
        detections[0].mask &= (xx + yy) > 345
    elif fault == "confidence_nan":
        detections[0].confidence = float('nan')
    elif fault == "confidence_high":
        detections[0].confidence = 1.1
    else:
        detections[0].mask[:] = False
        detections[0].mask[160:240, 160:240] = True
    result = detect_tray(image, xyz, visible_config, camera_to_base=transform, detections=detections)
    assert result["status"] != "ok", result
    assert reason in result["reason"], result


@pytest.mark.parametrize('key,value', [
    ('model_path', ''), ('target_class', ' tray'), ('device', None),
    ('confidence', 0), ('confidence', 1.1), ('confidence', True),
    ('imgsz', 0), ('imgsz', 641), ('imgsz', True),
])
def test_bad_yolo_configuration_rejected(config, key, value):
    config['yolo'][key] = value
    with pytest.raises(ValueError, match='invalid_tray:yolo'):
        validate_tray_config(config)


@pytest.mark.parametrize('outer,inner', [([.60, .59], [.5, .4]), ([.44, .60], [.4, .3]),
                                         ([.64, .48], [.7, .44]), ([0., .48], [.6, .44])])
def test_ambiguous_or_inconsistent_measured_rectangle_rejected(config, outer, inner):
    config['tray_outer_size'], config['tray_size'] = outer, inner
    with pytest.raises(ValueError, match='invalid_tray:tray'):
        validate_tray_config(config)


def test_large_tray_clipped_by_camera_cannot_be_used(visible_config):
    visible_config['tray_outer_size'] = [1., .5]
    image, xyz, _, transform, detections = synthetic_tray(visible_config)
    result = detect_tray(image, xyz, visible_config, camera_to_base=transform, detections=detections)
    assert result['reason'] == 'tray_mask_truncated'


def test_depth_in_only_the_mask_center_cannot_define_extent(visible_config):
    image, xyz, _, transform, detections = synthetic_tray(visible_config)
    # Most missing depth already fails the coverage gate, never extrapolates a pose.
    xyz[:185] = np.nan
    xyz[215:] = np.nan
    result = detect_tray(image, xyz, visible_config, camera_to_base=transform, detections=detections)
    assert result['status'] == 'invalid'
    assert 'depth' in result['reason']
