"""Bounded recovery policy tests without ROS or a device."""
import json
import pytest
from place_rounds import (RoundBudget, RoundLimits, load_place_zone, load_release_coordinates,
                          recoverable_failure, select_candidate_groups)
import test_live_place_pipeline as runner


def test_taught_zone_selects_ready_and_reference_without_changing_grasp_config(tmp_path):
    from grasp_executor.config import load_config
    root = runner.Path(__file__).resolve().parents[1]
    grasp = load_config(root / 'src/grasp_executor/config/place_config.json')
    zones = json.loads((root / 'src/grasp_executor/config/place_zones.json').read_text())
    for index, name in enumerate(('左上', '左下', '中间', '右上', '右下', 'blue')):
        selected = zones['zones'][name]
        selected['enabled'] = True
        selected['place_reference_position'] = dict(grasp.place_reference_position,
                                                    x=grasp.place_reference_position['x'] + index * .01)
        selected['place_reference_orientation'] = grasp.place_reference_orientation
        selected['place_ready_position'] = dict(grasp.place_ready_position,
                                                x=grasp.place_ready_position['x'] + index * .01)
        selected['place_ready_orientation'] = grasp.place_ready_orientation
    path = tmp_path / 'zones.json'
    path.write_text(json.dumps(zones))
    for index, name in enumerate(('左上', '左下', '中间', '右上', '右下', 'blue')):
        current = load_place_zone(path, name, grasp)
        assert current.place_ready_position['x'] == pytest.approx(grasp.place_ready_position['x'] + index * .01)
        assert current.place_reference_position['x'] == pytest.approx(grasp.place_reference_position['x'] + index * .01)
        assert current.place_drop_distance == zones['zones'][name]['place_drop_distance']
        assert current.return_home_joint_positions == zones['return_home']['joint_positions']
    assert grasp.place_ready_position['x'] == pytest.approx(.388824983)


def test_untaught_or_invalid_zone_fails_before_ros(tmp_path):
    from grasp_executor.config import load_config
    root = runner.Path(__file__).resolve().parents[1]
    grasp = load_config(root / 'src/grasp_executor/config/place_config.json')
    zones = json.loads((root / 'src/grasp_executor/config/place_zones.json').read_text())
    for entry in zones['zones'].values():
        entry['enabled'] = False
        entry['place_ready_position'] = None
    path = tmp_path / 'zones.json'
    path.write_text(json.dumps(zones))
    with pytest.raises(ValueError, match='place_zone_not_taught'):
        load_place_zone(path, '左上', grasp)
    with pytest.raises(ValueError, match='place_zone_not_taught'):
        load_place_zone(path, '中间', grasp)
    with pytest.raises(ValueError, match='invalid_target_zone'):
        load_place_zone(path, '其他', grasp)
    zones['zones']['左上']['enabled'] = True
    path.write_text(json.dumps(zones))
    with pytest.raises(ValueError, match='invalid_config:place_ready_position'):
        load_place_zone(path, '左上', grasp)


def test_no_progress_limit_not_keyed_by_object_ids():
    budget = RoundBudget(100., RoundLimits())
    for i in range(3):
        budget.begin_round(i)
        assert budget.retry_observation(i) is (i < 2)
    assert budget.observation_count == 3 and budget.reobservations_without_progress == 2
    assert budget.empty_result() == (False, 'failed', 'remaining_unknown')


@pytest.mark.parametrize('distance', [None, True, 0, -.1, float('nan'), .31])
def test_invalid_zone_descent_rejected(tmp_path, distance):
    from grasp_executor.config import load_config
    root = runner.Path(__file__).resolve().parents[1]
    config = json.loads((root / 'src/grasp_executor/config/place_zones.json').read_text())
    config['zones']['左上']['place_drop_distance'] = distance
    path = tmp_path / 'zone.json'
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError, match='invalid_place_zone_drop_distance'):
        load_place_zone(path, '左上', load_config(root / 'src/grasp_executor/config/place_config.json'))


def test_all_zones_lower_release_22mm_from_unchanged_ready_pose():
    from grasp_executor.config import load_config
    from grasp_executor.grasp import build_place_ready_pose, build_place_drop_pose
    root = runner.Path(__file__).resolve().parents[1]
    config = load_config(root / 'src/grasp_executor/config/place_config.json')
    original_release_z = {
        '左上': (.21900710047338923, .052514264758302495),
        '左下': (0.19203436539272722, 0.02593378104163971),
        '中间': (.2025063268803816, .03601186112739674),
        '右上': (.2048408181743376, .03835046299523383),
        '右下': (.19197654583545765, .02587331395528389),
        'blue': (.2016637394122519, .03517464680738627),
    }
    for zone, (reference_release_z, tool_release_z) in original_release_z.items():
        current = load_place_zone(root / 'src/grasp_executor/config/place_zones.json', zone, config)
        ready = build_place_ready_pose(current)
        drop = build_place_drop_pose(ready, current)
        assert current.place_drop_distance == pytest.approx(.122)
        assert current.place_reference_position['z'] - .122 == pytest.approx(reference_release_z - .022)
        assert drop.position[2] == pytest.approx(tool_release_z - .022)
        assert ready.position[2] - drop.position[2] == pytest.approx(.122)
        assert ready.position[:2] == drop.position[:2]
        assert ready.orientation == drop.orientation
        assert len(current.return_home_joint_positions) == 6


@pytest.mark.parametrize('zone,filename', [
    ('blue', 'blue_release_position.json'),
    ('左下', 'teach_capture_left_lower_release_20260929_143829.json'),
    ('右下', 'teach_capture_right_lower_release_20260929_143541.json'),
])
def test_release_matches_stationary_capture(zone, filename):
    from grasp_executor.config import load_config
    from grasp_executor.grasp import build_place_ready_pose, build_place_drop_pose
    root = runner.Path(__file__).resolve().parents[1]
    captured = json.loads((root / 'docs' / 'records' / filename).read_text())
    grasp = load_config(root / 'src/grasp_executor/config/place_config.json')
    selected = load_place_zone(root / 'src/grasp_executor/config/place_zones.json', zone, grasp)
    ready = build_place_ready_pose(selected)
    release = build_place_drop_pose(ready, selected)
    assert ready.position == pytest.approx(tuple(captured['descent_start_base_link_grasp_center']['position'].values()))
    original_release = captured['release_base_link_grasp_center']['position']
    assert release.position == pytest.approx((original_release['x'], original_release['y'],
                                              original_release['z'] - .022))
    assert ready.orientation == pytest.approx(tuple(captured['release_base_link_grasp_center']['orientation'].values()))
    assert selected.place_reference_position == pytest.approx(captured['descent_start_base_tool0']['position'])
    assert selected.place_reference_orientation == pytest.approx(captured['release_base_tool0']['orientation'])


def test_xyz_release_uses_blue_orientation_and_preserves_release_coordinates():
    from grasp_executor.config import load_config
    from grasp_executor.grasp import build_place_ready_pose, build_place_drop_pose
    root = runner.Path(__file__).resolve().parents[1]
    path = root / 'src/grasp_executor/config/place_zones.json'
    grasp = load_config(root / 'src/grasp_executor/config/place_config.json')
    blue = load_place_zone(path, 'blue', grasp)
    coords = (-.30, -.50, .22)
    selected = load_release_coordinates(path, coords, grasp)
    ready = build_place_ready_pose(selected)
    release = build_place_drop_pose(ready, selected)
    assert selected.place_reference_position == pytest.approx(dict(zip('xyz', (*coords[:2], coords[2]+.122))))
    assert selected.place_reference_orientation == blue.place_reference_orientation
    assert selected.place_ready_orientation == blue.place_ready_orientation
    assert selected.place_drop_distance == pytest.approx(.122)
    assert release.position == pytest.approx((
        blue.place_ready_position['x'] - (coords[0]-blue.place_reference_position['x']),
        blue.place_ready_position['y'] - (coords[1]-blue.place_reference_position['y']),
        blue.place_ready_position['z'] - blue.place_reference_position['z'] + coords[2]))
    assert ready.position[2] - release.position[2] == pytest.approx(.122)


@pytest.mark.parametrize('coords', [(0, 0), (0, 0, float('nan')),
                                    (0, 0, float('inf')), (True, 0, 0), (5, 0, 0)])
def test_invalid_xyz_release_rejected(coords):
    from grasp_executor.config import load_config
    root = runner.Path(__file__).resolve().parents[1]
    with pytest.raises(ValueError):
        load_release_coordinates(root / 'src/grasp_executor/config/place_zones.json', coords,
                                 load_config(root / 'src/grasp_executor/config/place_config.json'))


def test_empty_retries_do_not_mark_success_as_unresolved():
    budget = RoundBudget(10., RoundLimits())
    assert budget.retry_observation(1., record_failure=False)
    assert budget.retry_observation(2., record_failure=False)
    assert not budget.retry_observation(3., record_failure=False)
    assert not budget.pending_failure
    budget.complete_object()
    assert not budget.unresolved_rejections
    assert budget.reobservations_without_progress == 0


def test_empty_retry_preserves_prior_failure_and_deadline():
    budget = RoundBudget(10., RoundLimits())
    assert budget.retry_observation(1.)
    assert budget.retry_observation(2., record_failure=False)
    assert budget.pending_failure
    budget.complete_object()
    assert budget.unresolved_rejections
    assert not budget.retry_observation(10., record_failure=False)
    assert budget.reobservations_without_progress == 0


def test_success_resets_retry_count_but_not_global_budgets():
    budget = RoundBudget(10., RoundLimits(max_observation_rounds=3))
    budget.begin_round(1)
    budget.retry_observation(2)
    budget.complete_object()
    assert budget.reobservations_without_progress == 0 and budget.observation_count == 1
    with pytest.raises(TimeoutError): budget.begin_round(10)


@pytest.mark.parametrize('value', [True, -1, 1.5, '2'])
def test_invalid_limits(value):
    with pytest.raises(ValueError): RoundLimits(max_reobservations=value)


def test_unknown_failure_cannot_be_recovered_alongside_geometry_failure():
    statuses = {'grasp_executor': {'status': 'candidates_exhausted', 'command_attempted': False},
                'yolo_vision': {'status': 'failed', 'reason': 'input_timeout'}}
    assert not recoverable_failure(statuses)
    assert not recoverable_failure({'grasp_executor': {'status': 'candidates_exhausted'}})
    assert not recoverable_failure({'grasp_executor': {'status': 'failed', 'reason': 'ik_failure'}})


def test_selection_preserves_candidate_order_and_applies_offset_to_object_order_only():
    def group(oid, x):
        return {'object_id': oid, 'candidates': [
            {'candidate_id': 0, 'position': [x, 0., .2], 'geometry': {'score': 0}},
            {'candidate_id': 1, 'position': [0., 0., .2], 'geometry': {'score': 100}}]}
    batch = {'objects': [group(1, .2), group(2, .5)], 'rejected': []}
    original = json.dumps(batch)
    selected = select_candidate_groups(batch, {'x': -.5, 'y': 0., 'z': 0.}, True)
    assert selected['objects'][0]['object_id'] == 2
    assert [c['candidate_id'] for c in selected['objects'][0]['candidates']] == [0, 1]
    assert selected['objects'][0]['candidates'][0]['position'][0] == .5
    assert json.dumps(batch) == original


def test_task_mutex_is_independent_of_namespace(monkeypatch, tmp_path):
    monkeypatch.setattr(runner, 'task_lock_path', lambda: tmp_path/'lock')
    with runner.TaskLock():
        with pytest.raises(RuntimeError, match='another_place_task'):
            with runner.TaskLock(): pass
    with runner.TaskLock(): pass


def test_cleanup_stops_descendant_after_launch_leader_exits():
    import os
    import signal
    import subprocess
    import sys
    child_script = 'import time; time.sleep(60)'
    parent_script = (
        'import subprocess, sys; '
        'child = subprocess.Popen([sys.executable, "-c", sys.argv[1]], '
        'stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL); '
        'print(child.pid, flush=True)')
    leader = subprocess.Popen([sys.executable, '-c', parent_script, child_script],
        start_new_session=True, stdout=subprocess.PIPE, text=True)
    try:
        child_pid = int(leader.communicate(timeout=5)[0].strip())
        assert child_pid != leader.pid and leader.returncode == 0
        assert runner.process_group_alive(leader.pid)
        assert runner.stop_owned_processes({'already_exited_launch': leader}) == []
        assert not runner.process_group_alive(leader.pid)
    finally:
        try:
            os.killpg(leader.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        leader.wait(timeout=5)


def test_isolated_ros_fixtures_cannot_probe_concurrently(tmp_path):
    from isolated_ros import IsolatedRosTestLock
    path = tmp_path / 'ros-fixture.lock'
    with IsolatedRosTestLock(path):
        with pytest.raises(RuntimeError, match='another_isolated_ros_test'):
            IsolatedRosTestLock(path)
    with IsolatedRosTestLock(path):
        pass


def test_circle_rejection_reobserves_only_before_motion():
    from place_rounds import recoverable_failure
    statuses = {'rim_locator': {'status': 'failed', 'reason': 'pose_not_found:no_valid_object',
        'rejected': [{'object_id': 0, 'reason': 'pose_not_found:circle_insufficient_coverage'}]}}
    assert recoverable_failure(statuses)
    statuses['grasp_executor'] = {'status': 'failed', 'command_attempted': True}
    assert not recoverable_failure(statuses)
