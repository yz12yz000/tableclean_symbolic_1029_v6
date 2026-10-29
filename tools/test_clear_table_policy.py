"""Fresh-observation and occupancy regressions using software-only fixtures."""

from copy import deepcopy
from dataclasses import asdict
import math

import pytest

from clear_table_offline import SyntheticTable, WORKSPACE, synthetic_config
from grasp_executor.config import load_config
from grasp_executor.grasp import build_place_drop_pose, build_place_ready_pose, build_return_home_pose
from place_rounds import ClearTableSession, load_place_zone, load_release_coordinates, tray_release_config
from yolo_vision.tray_utils import load_tray_config, validate_tray_config


def setup_scene():
    config = synthetic_config()
    return ClearTableSession(config), SyntheticTable(config, 'normal')


def observe(session, table, index):
    vision, batch = table.observation(index, 'policy-test')
    return session.observe(vision, batch)


def reject_geometry(batch, object_id, reason='pose_not_found:circle_insufficient_points'):
    batch['objects'] = [group for group in batch['objects'] if group['object_id'] != object_id]
    batch['rejected'].append(dict(object_id=object_id, reason=reason))


def test_fresh_ids_and_moving_tray_verify_then_select_different_slot():
    session, table = setup_scene()
    first = observe(session, table, 1)
    session.record_motion(first)
    table.simulate_placement(first)
    second = observe(session, table, 2)
    assert second['kind'] == 'ready'
    assert second['tray']['center_xy'] != first['tray']['center_xy']
    assert second['tray']['yaw'] != first['tray']['yaw']
    assert second['source']['object_id'] != first['source']['object_id']
    assert second['slot']['id'] != first['slot']['id']
    assert all(second['verification'].values())
    assert session.motion_completed_count == session.verified_placed_count == 1
    assert len(session.remaining) == 2


def test_measured_sizes_place_largest_first_without_changing_its_contacts():
    config = synthetic_config()
    config['known_bowl_radii'] = [.06, .09]
    session = ClearTableSession(config)
    table = SyntheticTable(config, 'normal')
    vision, batch = table.observation(1, 'policy-test')
    target = vision['objects'][1]
    target['radius'] = .09
    before = deepcopy(next(g for g in batch['objects'] if g['object_id'] == target['object_id']))
    decision = session.observe(vision, batch)
    assert decision['kind'] == 'ready'
    assert decision['source']['object_id'] == target['object_id']
    assert decision['selected']['objects'] == [before]


def test_large_source_bowl_cannot_use_a_slot_sized_for_a_smaller_bowl():
    session, table = setup_scene()
    vision, batch = table.observation(1, 'policy-test')
    # This visible bowl fits the source region, but all destinations have only
    # 12 cm centre-to-edge clearance. Its 11 cm radius plus 2 cm margin cannot fit.
    vision['objects'][0]['radius'] = .11
    decision = session.observe(vision, batch)
    assert decision['kind'] == 'stop'
    assert decision['reason'] == 'tray_full'
    assert not session.used_positions


def test_unprocessed_sources_disappearing_never_proves_completion():
    session, table = setup_scene()
    assert observe(session, table, 1)['kind'] == 'ready'
    table.sources = []
    for index in range(2, 6):
        decision = observe(session, table, index)
        assert decision['kind'] == 'observe_again'
        assert decision['reason'] == 'observation_objects_missing'
    assert len(session.remaining) == 3
    assert session.empty_count == session.verified_placed_count == 0


@pytest.mark.parametrize('depth_rejected', [False, True])
def test_initial_unlocalized_object_remains_unresolved_after_disappearing(depth_rejected):
    session, table = setup_scene()
    table.sources = table.sources[:1]
    vision, batch = table.observation(1, 'policy-test')
    unknown = vision['objects'][0]
    unknown.update(position_valid=False, center_xy=None, radius=None, reason='no_valid_depth')
    if depth_rejected:
        vision['rejected'].append(vision['objects'].pop(0))
        vision['object_count'] -= 1
        batch['objects'] = [group for group in batch['objects'] if group['object_id'] != unknown['object_id']]
    decision = session.observe(vision, batch)
    assert decision['reason'] == 'object_position_uncertain'
    assert session.minimum_object_count == 2
    table.sources = []
    for index in (2, 3, 4):
        decision = observe(session, table, index)
        assert decision['kind'] == 'observe_again'
        assert decision['reason'] == 'observation_objects_missing'
    assert not session.used_positions


def test_preoccupied_bowl_missing_blocks_next_selection():
    session, table = setup_scene()
    assert observe(session, table, 1)['slot']['id'] != 'A'
    table.inside = []
    second = observe(session, table, 2)
    assert second['kind'] == 'observe_again'
    assert second['reason'] == 'observation_objects_missing'
    assert len(session.known_inside) == 1


def test_new_source_cannot_mask_loss_of_preoccupied_slot_after_tray_moves():
    session, table = setup_scene()
    assert observe(session, table, 1)['slot']['id'] != 'A'
    table.inside = []
    table.sources.append(dict(key=99, xy=[.15, .30]))
    table.motion_rounds = 3
    decision = observe(session, table, 2)
    assert decision['kind'] == 'ready'
    assert decision['slot']['id'] != 'A'
    assert len(session.known_inside) == 1
    assert session.known_inside[0]['uv'] == pytest.approx(table.config['slots'][0]['uv'])


def test_small_continuous_occupant_drift_updates_record_without_ghost_occupancy():
    session, table = setup_scene()
    fixed_slot = table.config['slots'][3]
    table.inside.append(dict(id=fixed_slot['id'], uv=list(fixed_slot['uv'])))
    initial_uv = table.config['slots'][0]['uv']
    step = .01
    assert step < session.config['position_tolerance'] < 4 * step

    def shifted_observation(index, cumulative_shift):
        vision, batch = table.observation(index, 'policy-test')
        # Shift only the original occupant in tray coordinates. SyntheticTable
        # also changes instance IDs each time and moves the tray after a pick.
        obj = vision['objects'][len(table.sources)]
        yaw = vision['tray']['yaw']
        obj['center_xy'][0] += math.cos(yaw) * cumulative_shift
        obj['center_xy'][1] += math.sin(yaw) * cumulative_shift
        group = next(group for group in batch['objects'] if group['object_id'] == obj['object_id'])
        x, y = (-value for value in obj['center_xy'])
        group['candidates'][0]['position'][:2] = [x, y]
        group['distance'] = math.hypot(x, y)
        return vision, batch

    for index in range(1, 6):
        shift = (index - 1) * step
        decision = session.observe(*shifted_observation(index, shift))
        assert decision['kind'] == 'ready'
        assert len(session.known_inside) == 2
        assert session.known_inside[0]['uv'] == pytest.approx([initial_uv[0] + shift, initial_uv[1]])
        assert session.known_inside[1]['uv'] == pytest.approx(fixed_slot['uv'])
        assert len(decision['inside_uv']) == 2

    session.record_motion(decision)
    table.simulate_placement(decision)
    after = session.observe(*shifted_observation(6, 4 * step + step / 2))
    assert after['kind'] == 'ready'
    assert all(after['verification'].values())
    assert session.verified_placed_count == 1
    assert len(session.known_inside) == 3  # Two prior occupants and one new bowl.


def test_last_placed_bowl_missing_stays_pending_and_reserved_until_seen():
    session, table = setup_scene()
    first = observe(session, table, 1)
    session.record_motion(first)
    table.simulate_placement(first)
    placed = table.inside.pop()
    for index in (2, 3):
        decision = observe(session, table, index)
        assert decision['kind'] == 'observe_again'
        assert session.last_placement is not None
        assert session.verified_placed_count == 0
        assert session.used_positions == {first['slot']['id']}
    table.inside.append(placed)
    decision = observe(session, table, 4)
    assert decision['kind'] == 'ready'
    assert session.verified_placed_count == 1
    assert decision['slot']['id'] != first['slot']['id']


def test_motion_ack_without_source_removal_or_new_occupancy_never_verifies():
    session, table = setup_scene()
    first = observe(session, table, 1)
    session.record_motion(first)
    for index in (2, 3, 4):
        decision = observe(session, table, index)
        assert decision['kind'] == 'observe_again'
        assert decision['reason'] == 'placement_unverified'
        assert decision['verification']['source_absent'] is False
        assert decision['verification']['new_at_slot'] is False
    assert session.motion_completed_count == 1 and session.verified_placed_count == 0


@pytest.mark.parametrize('reason', ['pose_not_found:circle_insufficient_points', 'output_z_outside_workspace'])
def test_all_bowls_inside_with_geometric_rejections_complete_after_fresh_confirmation(reason):
    session, table = setup_scene()
    table.sources = []
    for index in (1, 2):
        vision, batch = table.observation(index, 'policy-test')
        reject_geometry(batch, vision['objects'][0]['object_id'], reason)
        decision = session.observe(vision, batch)
        assert decision['kind'] == ('observe_again' if index == 1 else 'complete')
    assert session.motion_completed_count == session.verified_placed_count == 0


def test_unknown_rejection_inside_tray_stops_instead_of_reporting_complete():
    session, table = setup_scene()
    table.sources = []
    vision, batch = table.observation(1, 'policy-test')
    reject_geometry(batch, vision['objects'][0]['object_id'], 'unexpected_geometry_exception')
    decision = session.observe(vision, batch)
    assert decision['kind'] == 'stop'
    assert decision['reason'] == 'localization_rejected:unexpected_geometry_exception'


def test_preview_selection_never_commits_motion_or_reserves_destination():
    session, table = setup_scene()
    first = observe(session, table, 1)
    second = observe(session, table, 2)
    assert first['slot'] == second['slot']
    assert first['source']['center_xy'] == second['source']['center_xy']
    assert session.motion_completed_count == session.verified_placed_count == 0
    assert session.last_placement is None and not session.used_positions
    with pytest.raises(ValueError, match='invalid_clear_table_motion_commit'):
        session.record_motion(first)


def test_motion_commit_cannot_be_repeated_or_altered():
    session, table = setup_scene()
    decision = observe(session, table, 1)
    changed = deepcopy(decision)
    changed['slot']['xy'][0] += .01
    with pytest.raises(ValueError, match='invalid_clear_table_motion_commit'):
        session.record_motion(changed)
    session.record_motion(decision)
    with pytest.raises(ValueError, match='invalid_clear_table_motion_commit'):
        session.record_motion(decision)
    assert session.motion_completed_count == 1


@pytest.mark.parametrize('stamp', [1_000_000_000, 999_999_999, 0, True])
def test_repeated_or_stale_observation_cannot_advance_session(stamp):
    session, table = setup_scene()
    observe(session, table, 1)
    vision, batch = table.observation(2, 'policy-test')
    vision['source_stamp_ns'] = batch['source_stamp_ns'] = stamp
    with pytest.raises(ValueError, match='clear_table_observation_not_fresh'):
        session.observe(vision, batch)
    assert session.motion_completed_count == 0


def test_other_successes_do_not_erase_a_source_with_rejected_geometry():
    session, table = setup_scene()
    for index in (1, 2):
        vision, batch = table.observation(index, 'policy-test')
        reject_geometry(batch, vision['objects'][0]['object_id'])
        decision = session.observe(vision, batch)
        assert decision['kind'] == 'ready'
        assert decision['source']['center_xy'] != table.sources[0]['xy']
        session.record_motion(decision)
        table.simulate_placement(decision)
    vision, batch = table.observation(3, 'policy-test')
    reject_geometry(batch, vision['objects'][0]['object_id'])
    decision = session.observe(vision, batch)
    assert decision['reason'] == 'source_candidates_exhausted'
    assert session.verified_placed_count == 2
    assert session.remaining == [table.sources[0]['xy']]
    table.sources = []
    decision = observe(session, table, 4)
    assert decision['kind'] == 'observe_again'
    assert decision['reason'] == 'observation_objects_missing'
    assert len(session.remaining) == 1


@pytest.mark.parametrize('position', [[-.15, -.40], [.95, .30]])
def test_tray_or_source_region_boundary_bowl_is_unresolved(position):
    session, table = setup_scene()
    table.sources[0]['xy'] = position
    decision = observe(session, table, 1)
    assert decision['kind'] == 'observe_again'
    assert decision['reason'] == 'object_position_uncertain'
    assert not session.used_positions


def test_missing_tray_retains_pending_state_until_tray_returns():
    session, table = setup_scene()
    first = observe(session, table, 1)
    session.record_motion(first)
    table.simulate_placement(first)
    table.scenario = 'missing'
    decision = observe(session, table, 2)
    assert decision['kind'] == 'observe_again'
    assert session.last_placement is not None
    assert session.verified_placed_count == 0
    table.scenario = 'normal'
    decision = observe(session, table, 3)
    assert decision['kind'] == 'ready' and session.verified_placed_count == 1


def test_different_tray_class_is_rejected_before_any_action():
    session, table = setup_scene()
    observe(session, table, 1)
    vision, batch = table.observation(2, 'policy-test')
    vision['tray']['identity'] = 'yolo:other_tray'
    with pytest.raises(ValueError, match='invalid_tray:observation.identity'):
        session.observe(vision, batch)
    assert session.motion_completed_count == 0


def _rotated_scene(table, index, physical_degrees, reported_degrees):
    """Move tray contents physically, independently of its undirected mask yaw."""
    vision, batch = table.observation(index, 'policy-test')
    tray = vision['tray']
    delta = math.radians(physical_degrees) - tray['yaw']
    c, s = math.cos(delta), math.sin(delta)
    for obj in vision['objects'][len(table.sources):]:
        dx, dy = (obj['center_xy'][i] - tray['center_xy'][i] for i in range(2))
        obj['center_xy'] = [tray['center_xy'][0] + c*dx - s*dy,
                            tray['center_xy'][1] + s*dx + c*dy]
        group = next(g for g in batch['objects'] if g['object_id'] == obj['object_id'])
        group['candidates'][0]['position'][:2] = [-v for v in obj['center_xy']]
        group['distance'] = math.hypot(*obj['center_xy'])
    tray['yaw'] = math.radians(reported_degrees)
    return vision, batch


@pytest.mark.parametrize('first_angle,next_angle,reported_angle', [
    (89., 91., -89.), (-89., -91., 89.), (0., 2., -178.)])
def test_mask_axis_flip_preserves_placement_verification_and_reserved_slot(
        first_angle, next_angle, reported_angle):
    session, table = setup_scene()
    first = session.observe(*_rotated_scene(table, 1, first_angle, first_angle))
    assert first['kind'] == 'ready'
    session.record_motion(first)
    table.simulate_placement(first)
    second = session.observe(*_rotated_scene(table, 2, next_angle, reported_angle))
    assert second['kind'] == 'ready'
    assert second['tray']['yaw'] == pytest.approx(math.radians(next_angle))
    assert all(second['verification'].values())
    assert session.verified_placed_count == 1
    assert first['slot']['id'] in session.used_positions
    assert second['slot']['id'] not in session.used_positions
    assert len(session.known_inside) == 2


def test_large_inter_observation_rotation_preserves_pending_placement():
    session, table = setup_scene()
    first = observe(session, table, 1)
    session.record_motion(first)
    table.simulate_placement(first)
    previous = deepcopy(session.last_placement)
    decision = session.observe(*_rotated_scene(table, 2, 60., 60.))
    assert decision == {'kind': 'observe_again', 'reason': 'tray_rotation_too_large'}
    assert session.last_placement == previous
    assert session.verified_placed_count == 0
    assert session.tray_yaw == pytest.approx(0.)
    assert session.used_positions == {first['slot']['id']}


def test_live_config_requires_calibration_and_source_region(tmp_path):
    config = synthetic_config()
    config['calibrated'] = False
    with pytest.raises(ValueError, match='tray_config_not_calibrated'):
        validate_tray_config(config, require_calibrated=True)
    config['calibrated'], config['source_polygon'] = True, None
    with pytest.raises(ValueError, match='source_polygon:required_for_live'):
        validate_tray_config(config, require_calibrated=True)
    with pytest.raises(FileNotFoundError):
        load_tray_config(tmp_path / 'missing.json', require_calibrated=True)


@pytest.mark.parametrize('release_z, offset', [(None, [0., 0.]), (.18, [.01, -.02])])
def test_same_release_xyz_preserves_old_drop_pose_orientation_home_and_grasp_config(release_z, offset):
    config = synthetic_config()
    config.update(release_z=release_z, release_xy_offset=offset)
    zones = WORKSPACE / 'src/grasp_executor/config/place_zones.json'
    grasp = load_config(WORKSPACE / 'src/grasp_executor/config/place_config.json')
    slot = dict(id='B', uv=[.16, -.10], xy=[-.29, -.50])
    blue = load_place_zone(zones, 'blue', grasp)
    height = release_z if release_z is not None else blue.place_reference_position['z'] - blue.place_drop_distance
    xyz = [slot['xy'][0] + offset[0], slot['xy'][1] + offset[1], height]
    old = load_release_coordinates(zones, xyz, grasp)
    new = tray_release_config(slot, config, blue)
    assert asdict(new) == asdict(old)
    assert build_place_ready_pose(new) == build_place_ready_pose(old)
    assert build_place_drop_pose(build_place_ready_pose(new), new) == build_place_drop_pose(build_place_ready_pose(old), old)
    assert build_return_home_pose(new) == build_return_home_pose(old)
    assert new.return_home_joint_positions == old.return_home_joint_positions


@pytest.mark.parametrize('radius,yaw', [(.065, -.9), (.0925, 2.7)])
def test_measured_bowl_center_lands_at_slot_preserving_original_release(radius, yaw):
    import numpy as np
    from rim_locator.rim_detection import transform_points
    config = synthetic_config()
    config.update(known_bowl_radii=[.065, .0925], compensate_bowl_xy=True, release_xy_offset=[0., 0.])
    blue = load_place_zone(WORKSPACE / 'src/grasp_executor/config/place_zones.json', 'blue',
                           load_config(WORKSPACE / 'src/grasp_executor/config/place_config.json'))
    source = dict(frame_id='base', position_valid=True, position_reason='measured_rim_circle',
                  measured_radius=radius, center_xy=[-.65, -.08], center_z=-.02)
    q = [math.cos(yaw / 2), math.sin(yaw / 2), 0., 0.]
    rotation = transform_points(np.eye(3), [0., 0., 0.], q).T
    # Include an off-axis observed contact, not just an ideal radial offset.
    held_center = np.array([.003, -radius, -.004])
    center = np.array([.65, .08, -.02])
    contact = center - rotation @ held_center
    candidate = dict(position=contact.tolist(), orientation=q)
    slot = dict(xy=[-.45, -.35])
    result = tray_release_config(slot, config, blue, source=source, candidate=candidate)
    release = build_place_drop_pose(build_place_ready_pose(result), result)
    release_rotation = transform_points(np.eye(3), [0., 0., 0.],
                                        [result.place_ready_orientation[k] for k in 'xyzw']).T
    actual_center = np.array([result.place_ready_position[k] for k in 'xyz']) + release_rotation @ held_center
    assert -actual_center[:2] == pytest.approx(slot['xy'])
    assert result.place_ready_position['z'] == blue.place_ready_position['z']
    assert result.place_ready_orientation == blue.place_ready_orientation
    assert result.place_reference_orientation == blue.place_reference_orientation
    assert result.place_drop_distance == blue.place_drop_distance
    assert result.grasp_offset == blue.grasp_offset
    assert result.return_home_joint_positions == blue.return_home_joint_positions
    with pytest.raises(ValueError, match='requires_measured'):
        tray_release_config(slot, config, blue)


@pytest.mark.parametrize('corruption, reason', [
    ('vision_count', 'clear_table_observation_count_mismatch'),
    ('candidate_context', 'clear_table_batch_context_mismatch'),
    ('candidate_stamp', 'clear_table_batch_context_mismatch'),
    ('missing_candidate', 'perception_object_mismatch'),
])
def test_inconsistent_observation_cannot_be_selected(corruption, reason):
    session, table = setup_scene()
    vision, batch = table.observation(1, 'policy-test')
    if corruption == 'vision_count':
        vision['detected_count'] += 1
    elif corruption == 'candidate_context':
        batch['observation_id'] = 'old-observation'
    elif corruption == 'candidate_stamp':
        batch['source_stamp_ns'] -= 1
    else:
        batch['objects'].pop()
    with pytest.raises(ValueError, match=reason):
        session.observe(vision, batch)
    assert session.motion_completed_count == session.verified_placed_count == 0
