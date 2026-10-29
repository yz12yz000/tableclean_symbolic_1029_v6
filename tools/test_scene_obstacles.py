"""New scene classes reserve tray space but never become grasp candidates."""
from copy import deepcopy

import numpy as np

from clear_table_offline import SyntheticTable, synthetic_config
from place_rounds import ClearTableSession
from yolo_vision.tray_utils import estimate_obstacle_position, slot_xy


def test_cup_reserves_destination_and_missed_next_frame_keeps_occupancy():
    config = synthetic_config()
    table = SyntheticTable(config, 'normal')
    table.inside = []
    table.sources = table.sources[:1]
    vision, batch = table.observation(1, 'test')
    baseline = ClearTableSession(config).observe(vision, batch)
    assert baseline['kind'] == 'ready'
    occupied = baseline['slot']
    cup = dict(object_id=0, class_name='cup', confidence=.9, position_valid=True,
               frame_id='base', center_xy=occupied['xy'], radius=config['bowl_radius'])
    vision['obstacles'] = [cup]
    original = deepcopy(batch)
    session = ClearTableSession(config)
    selected = session.observe(vision, batch)
    assert selected['kind'] == 'ready'
    assert selected['slot']['id'] != occupied['id']
    assert batch == original
    assert len(selected['selected']['objects']) == 1
    assert any(np.linalg.norm(np.array(record['uv'])-occupied['uv']) < 1e-8
               for record in session.known_inside)
    vision, batch = table.observation(2, 'test')
    selected = session.observe(vision, batch)
    assert selected['kind'] == 'ready' and selected['slot']['id'] != occupied['id']


def test_cup_is_preserved_for_placement_verification_but_cannot_verify_a_bowl():
    config = synthetic_config()
    table = SyntheticTable(config, 'normal')
    table.inside = []
    table.sources = table.sources[:1]
    vision, batch = table.observation(1, 'test')
    occupied = ClearTableSession(config).observe(vision, batch)['slot']
    cup = dict(object_id=0, class_name='cup', position_valid=True, frame_id='base',
               center_xy=occupied['xy'], radius=config['bowl_radius'])
    vision['obstacles'] = [cup]
    session = ClearTableSession(config)
    decision = session.observe(vision, batch)
    session.record_motion(decision)
    table.simulate_placement(decision)
    vision, batch = table.observation(2, 'test')
    # Simulator moves/rotates the tray, so the cup follows its stored local UV.
    cup['center_xy'] = slot_xy(vision['tray'], occupied['id'], config)
    vision['obstacles'] = [cup]
    result = session.observe(vision, batch)
    assert session.verified_placed_count == 1, result
    assert result['verification']['previous_occupancy_preserved']


def test_obstacle_alone_at_release_slot_does_not_count_as_placed_bowl():
    config = synthetic_config()
    table = SyntheticTable(config, 'normal')
    table.inside = []
    table.sources = table.sources[:1]
    session = ClearTableSession(config)
    decision = session.observe(*table.observation(1, 'test'))
    session.record_motion(decision)
    table.sources = []
    vision, batch = table.observation(2, 'test')
    vision['obstacles'] = [dict(object_id=0, class_name='cup', position_valid=True,
        frame_id='base', center_xy=slot_xy(vision['tray'], decision['slot']['id'], config),
        radius=decision['source']['radius'])]
    result = session.observe(vision, batch)
    assert result['kind'] == 'observe_again' and session.verified_placed_count == 0


def test_unknown_cup_depth_stops_before_selecting_a_grasp():
    config = synthetic_config()
    vision, batch = SyntheticTable(config, 'normal').observation(1, 'test')
    vision['obstacles'] = [dict(object_id=0, class_name='cup', position_valid=False,
        frame_id='base', center_xy=None, radius=None, position_reason='insufficient_obstacle_depth')]
    decision = ClearTableSession(config).observe(vision, batch)
    assert decision['kind'] == 'observe_again' and decision['reason'] == 'obstacle_position_uncertain'
    assert 'selected' not in decision


def test_obstacle_uses_all_measured_points_and_rejects_sparse_depth():
    config = synthetic_config()
    theta = np.linspace(0, 2*np.pi, 80, endpoint=False)
    points = np.c_[.1+.03*np.cos(theta), .2+.03*np.sin(theta), np.ones(80)]
    result = estimate_obstacle_position(points, np.eye(4), config)
    assert result['position_valid']
    assert result['radius'] >= .03+config['position_tolerance']-1e-6
    assert np.allclose(result['center_xy'], [.1,.2], atol=1e-6)
    points[:60] = 0
    result = estimate_obstacle_position(points, np.eye(4), config)
    assert not result['position_valid'] and result['position_reason'] == 'insufficient_obstacle_depth'
