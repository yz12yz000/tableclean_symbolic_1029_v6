from dataclasses import replace
import math
import numpy as np
import pytest

from rim_locator.config import RimConfig
from rim_locator.circle_fit import fit_rim_circle
from rim_locator.rim_detection import locate_grasp_candidates, locate_rim, _rim_contacts
from grasp_executor.cartesian import rotate
from test_multi_pose import wall_cloud, bowl_points


def config(**kwargs):
    return replace(RimConfig(orientation_mode='circle_radial', max_tool_tilt_deg=0.), **kwargs)


def circle(angles=None, radius=.085, center=(.4, .15)):
    angles = np.linspace(0, 2*np.pi, 360, endpoint=False) if angles is None else angles
    return np.c_[center[0]+radius*np.cos(angles), center[1]+radius*np.sin(angles), np.full(len(angles), .4)]


def test_fit_known_circle_noisy_outliers_and_shuffle():
    rng = np.random.default_rng(12)
    p = circle()
    p[:, :2] += rng.normal(0., .0008, p[:, :2].shape)
    outliers = np.c_[rng.uniform(.27, .53, 45), rng.uniform(.02, .28, 45), np.full(45, .4)]
    p = np.r_[p, outliers]
    c, d = fit_rim_circle(p, config())
    np.testing.assert_allclose(c, [.4, .15], atol=.001)
    assert d['circle_radius'] == pytest.approx(.085, abs=.001)
    shuffled, _ = fit_rim_circle(p[rng.permutation(len(p))], config())
    np.testing.assert_array_equal(shuffled, c)


@pytest.mark.parametrize('points', [circle(np.linspace(0, 1.8, 300)),
    np.c_[np.linspace(.2,.5,100),np.zeros(100),np.full(100,.4)],
    np.repeat([[.4,.15,.4]], 50, axis=0), circle(radius=.02),
    circle()*[1., .55, 1.]])
def test_poor_geometry_is_rejected_without_old_orientation_fallback(points):
    with pytest.raises(ValueError, match='pose_not_found:circle'):
        fit_rim_circle(points, config())


@pytest.mark.parametrize('outside', ['positive_y', 'negative_y'])
@pytest.mark.parametrize('center', [(.4,.15),(-.4,.15),(-.4,-.15),(.4,-.15)])
def test_fixed_physical_finger_and_constant_center_offset(outside, center):
    points = wall_cloud(center=center)
    cfg = config(outside_finger_axis=outside)
    poses = locate_grasp_candidates(points, cfg)
    assert len(poses) > 1
    observed = [point for point, _ in _rim_contacts(locate_rim(points,cfg),cfg)]
    offsets = []
    sign = 1 if outside == 'positive_y' else -1
    for p in poses:
        # Candidate XYZ is still an observed contact, not projected onto a circle.
        assert any(np.array_equal(p.point, point) for point in observed)
        y = np.array(rotate(p.orientation,(0.,1.,0.)))
        outward = np.r_[p.point[:2]-center,0.];outward /= np.linalg.norm(outward)
        assert np.dot(sign*y, outward) > .99999
        np.testing.assert_allclose(rotate(p.orientation,(0.,0.,1.)),[0.,0.,-1.],atol=1e-12)
        to_center = np.r_[np.array(center)-p.point[:2],0.]
        local = rotate(tuple(-v for v in p.orientation[:3])+(p.orientation[3],),to_center)
        assert abs(local[0]) < 1e-6 and sign*local[1] < 0
        offsets.append(local)
    # The same final tool pose yields the same bowl centre across these grasps.
    assert np.ptp(np.array(offsets)[:,1]) < .001


def test_sign_switch_rotates_tool_180_without_changing_contact():
    p = wall_cloud()
    plus = locate_grasp_candidates(p,config())
    minus = locate_grasp_candidates(p,config(outside_finger_axis='negative_y'))
    for a,b in zip(plus,minus):
        np.testing.assert_array_equal(a.point,b.point)
        np.testing.assert_allclose(rotate(a.orientation,(0.,1.,0.)), -np.array(rotate(b.orientation,(0.,1.,0.))),atol=1e-12)


def test_rim_alone_cannot_bypass_wall_support():
    with pytest.raises(ValueError,match='insufficient_wall_points'):
        locate_grasp_candidates(bowl_points(),config())


@pytest.mark.parametrize('field', ['circle_min_radius','circle_max_radius','circle_inlier_distance',
    'circle_min_inlier_ratio','circle_max_residual','circle_min_coverage_deg',
    'circle_max_contact_error','circle_max_normal_deviation_deg'])
@pytest.mark.parametrize('value', [0,-1,float('nan'),True])
def test_invalid_circle_parameters(field,value):
    with pytest.raises(ValueError):config(**{field:value})


def test_invalid_side_tilt_and_limits():
    for kwargs in [dict(outside_finger_axis='left'),dict(max_tool_tilt_deg=5),
                   dict(circle_min_coverage_deg=119),dict(circle_min_radius=.2),
                   dict(circle_min_inlier_ratio=1.1)]:
        with pytest.raises(ValueError):config(**kwargs)


def test_no_supported_radial_contact_is_explicit_failure(monkeypatch):
    from rim_locator import rim_detection as g
    points = wall_cloud()
    original = g._supported_walls
    def wrong_direction(*args, **kwargs):
        for point, normal, local, wall, diagnostics in original(*args, **kwargs):
            yield point, np.array([-normal[1], normal[0], normal[2]]), local, wall, diagnostics
    monkeypatch.setattr(g,'_supported_walls',wrong_direction)
    with pytest.raises(ValueError,match='circle_no_supported_radial_contact'):
        locate_grasp_candidates(points,config())


def test_radial_offset_is_bounded_by_configured_contact_tolerance():
    poses = locate_grasp_candidates(wall_cloud(tilt_deg=20.), config())
    for p in poses:
        d = p.diagnostics
        assert abs(np.linalg.norm(p.point[:2]-d['circle_center_xy'])-d['circle_radius']) <= .001


@pytest.mark.parametrize('center', [(.4,.15),(-.4,.15),(-.4,-.15),(.4,-.15)])
def test_deployed_config_puts_confirmed_left_positive_y_finger_outside(center):
    """2026-09-28 RViz confirmation: marked physical left finger is tool +Y."""
    from pathlib import Path
    from rim_locator.config import load_config
    deployed = load_config(Path(__file__).parents[1] / 'config/place_config.json')
    assert deployed.tool_frame == 'grasp_center'
    assert deployed.orientation_mode == 'circle_radial'
    assert deployed.outside_finger_axis == 'positive_y'
    poses = locate_grasp_candidates(wall_cloud(center=center, tilt_deg=20.), deployed)
    assert poses
    for pose in poses:
        c = np.array(pose.diagnostics['circle_center_xy'])
        y_axis = np.array(rotate(pose.orientation, (0., 1., 0.)))[:2]
        # Probe the two sides of the actual contact in world coordinates:
        # the marked left (+Y) is outside the fitted rim, right (-Y) is inside.
        left_side = pose.point[:2] + .005 * y_axis
        right_side = pose.point[:2] - .005 * y_axis
        assert np.linalg.norm(left_side-c) > pose.diagnostics['circle_radius']
        assert np.linalg.norm(right_side-c) < pose.diagnostics['circle_radius']


@pytest.mark.parametrize('outside', ['positive_y', 'negative_y'])
def test_circle_radial_node_output_passes_actual_candidate_protocol(outside):
    from builtin_interfaces.msg import Time
    from rim_locator.main import RimNode
    from rim_locator.candidate_protocol import decode_candidate_batch
    from test_grasp_candidates import node_for_finish
    cfg = config(outside_finger_axis=outside, publish_debug=False)
    node, outputs = node_for_finish(cfg)
    points = wall_cloud(tilt_deg=20.)
    RimNode.finish(node, points, np.zeros(len(points), dtype=np.uint32), Time(sec=123))
    batch = decode_candidate_batch(outputs.groups[0].data,
                                   expected_task_id='task', expected_observation_id='task/round_0001')
    assert outputs.reports[-1][0] == 'success'
    assert batch['objects'] and outputs.poses and outputs.points
    for candidate in batch['objects'][0]['candidates']:
        assert type(candidate['geometry']['radial_contact_error']) is float
        assert candidate['geometry']['outside_finger_axis'] == outside


@pytest.mark.parametrize('phase', [0., math.pi/2, math.pi, 3*math.pi/2])
def test_noisy_partial_rim_is_usable_without_requiring_half_circle(phase):
    rng = np.random.default_rng(35)
    points = circle(np.linspace(phase, phase + math.radians(140), 300))
    points[:, :2] += rng.normal(0., .0008, points[:, :2].shape)
    center, diagnostics = fit_rim_circle(points, config())
    np.testing.assert_allclose(center, [.4, .15], atol=.003)
    assert diagnostics['circle_radius'] == pytest.approx(.085, abs=.003)
    assert 120 <= diagnostics['circle_coverage_deg'] < 180
    assert diagnostics['circle_sector_count'] >= 4
    with pytest.raises(ValueError, match='circle_insufficient_coverage'):
        fit_rim_circle(points, config(circle_min_coverage_deg=180.))


def test_isolated_opposite_patch_does_not_rescue_short_main_arc():
    # Total angular extent exceeds 120 degrees, but only three sectors have
    # observations: a short main arc plus one tiny patch is not enough.
    angles = np.r_[np.linspace(0, math.radians(50), 200),
                   np.linspace(math.radians(175), math.pi, 30)]
    with pytest.raises(ValueError, match='circle_insufficient_sectors'):
        fit_rim_circle(circle(angles), config())


@pytest.mark.parametrize('outside', ['positive_y', 'negative_y'])
def test_partial_wall_preserves_finger_direction_and_encodes_real_node_output(outside):
    from builtin_interfaces.msg import Time
    from rim_locator.main import RimNode
    from rim_locator.candidate_protocol import decode_candidate_batch
    from test_grasp_candidates import node_for_finish
    points = wall_cloud(tilt_deg=10.)
    angles = np.arctan2(points[:, 1]-.15, points[:, 0]-.4)
    points = points[np.abs(angles) < math.radians(70)]
    cfg = config(outside_finger_axis=outside, publish_debug=False)
    node, outputs = node_for_finish(cfg)
    RimNode.finish(node, points, np.zeros(len(points), dtype=np.uint32), Time(sec=123))
    assert outputs.reports[-1][0] == 'success'
    batch = decode_candidate_batch(outputs.groups[0].data,
                                   expected_task_id='task', expected_observation_id='task/round_0001')
    assert batch['objects'][0]['candidates']
    sign = 1 if outside == 'positive_y' else -1
    for pose in locate_grasp_candidates(points, cfg):
        d = pose.diagnostics
        assert 120 <= d['circle_coverage_deg'] < 180
        y_axis = np.array(rotate(pose.orientation, (0., 1., 0.)))[:2]
        outward = pose.point[:2] - np.array(d['circle_center_xy'])
        assert np.dot(sign*y_axis, outward) / np.linalg.norm(outward) > .99999
        assert d['radial_contact_error'] <= cfg.circle_max_contact_error


@pytest.mark.parametrize('outside', ['positive_y', 'negative_y'])
def test_short_arc_uses_supported_wall_and_serializes_with_fixed_finger(outside):
    from builtin_interfaces.msg import Time
    from rim_locator.main import RimNode
    from rim_locator.candidate_protocol import decode_candidate_batch
    from test_grasp_candidates import node_for_finish
    points = wall_cloud(tilt_deg=10.)
    angles = np.arctan2(points[:, 1]-.15, points[:, 0]-.4)
    points = points[np.abs(angles) < math.radians(50)]
    cfg = config(circle_allow_wall_fallback=True, outside_finger_axis=outside, publish_debug=False)
    with pytest.raises(ValueError, match='circle_insufficient_coverage') as rejected:
        locate_grasp_candidates(points, replace(cfg, circle_allow_wall_fallback=False))
    assert rejected.value.diagnostics['circle_coverage_deg'] < 120
    poses = locate_grasp_candidates(points, cfg)
    assert poses
    contacts = [point for point, _ in _rim_contacts(locate_rim(points, cfg), cfg)]
    sign = 1 if outside == 'positive_y' else -1
    for pose in poses:
        assert any(np.array_equal(pose.point, point) for point in contacts)
        assert pose.diagnostics['method'] == 'supported_wall_fallback'
        assert 'circle_center_xy' not in pose.diagnostics
        assert pose.diagnostics['circle_fit_rejected']['circle_coverage_deg'] < 120
        outward = pose.point[:2] - [.4, .15]
        outward /= np.linalg.norm(outward)
        y = np.array(rotate(pose.orientation, (0., 1., 0.)))[:2]
        assert np.dot(sign*y, outward) > .99
        np.testing.assert_allclose(rotate(pose.orientation, (0., 0., 1.)), [0, 0, -1], atol=1e-12)
    node, outputs = node_for_finish(cfg)
    RimNode.finish(node, points, np.zeros(len(points), dtype=np.uint32), Time(sec=123))
    batch = decode_candidate_batch(outputs.groups[0].data)
    assert outputs.reports[-1][0] == 'success'
    assert batch['objects'][0]['candidates'][0]['geometry']['method'] == 'supported_wall_fallback'


def test_fallback_still_requires_real_wall_support():
    points = circle(np.linspace(0, math.radians(95), 300))
    with pytest.raises(ValueError, match='insufficient_wall_points') as rejected:
        locate_grasp_candidates(points, config(circle_allow_wall_fallback=True))
    assert rejected.value.diagnostics['circle_fallback_reason'].endswith('circle_insufficient_coverage')


@pytest.mark.parametrize('reason', ['circle_residual', 'circle_radius_out_of_range', 'circle_low_inlier_ratio'])
def test_bad_circle_quality_cannot_enable_wall_fallback(monkeypatch, reason):
    from rim_locator import circle_fit
    def reject(*args):
        raise circle_fit.CircleFitError('pose_not_found:' + reason, {})
    monkeypatch.setattr(circle_fit, 'fit_rim_circle', reject)
    with pytest.raises(ValueError, match=reason):
        locate_grasp_candidates(wall_cloud(), config(circle_allow_wall_fallback=True))


@pytest.mark.parametrize('value', [1, 'true', None])
def test_wall_fallback_requires_boolean_configuration(value):
    with pytest.raises(ValueError, match='circle_allow_wall_fallback'):
        config(circle_allow_wall_fallback=value)
