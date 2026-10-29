"""Actual clear-table runner with mocked ROS transport and subprocesses, no devices."""
import json
from pathlib import Path

import pytest
import test_live_place_pipeline as runner

from clear_table_offline import synthetic_config, SyntheticTable
from place_rounds import ClearTableSession
from test_place_runner import run_fake


def snapshot(table, number):
    vision, batch = table.observation(number, 'test')
    return dict(count=len(vision['objects']), ids=[o['object_id'] for o in vision['objects']],
                positions=[g['candidates'][0]['position'] for g in batch['objects']],
                physical=vision['objects'], tray=vision['tray'])


def test_compensated_round_keeps_contacts_for_candidate_specific_recompute(monkeypatch, tmp_path):
    config = synthetic_config()
    config.update(known_bowl_radii=[.075], compensate_bowl_xy=True)
    table = SyntheticTable(config, 'normal')
    scene = snapshot(table, 1)
    for obj, pos in zip(scene['physical'], scene['positions']):
        obj.update(position_reason='measured_rim_circle', measured_radius=.075, center_z=pos[2])
        # Original candidate positions stay on the measured rim in base_link.
        pos[0], pos[1] = -obj['center_xy'][0] - .075, -obj['center_xy'][1]
    code, result, _, selected = run_fake(monkeypatch, tmp_path, [scene], False, clear_table=config)
    assert code == 0, result
    assert len(selected) == 1
    candidates = selected[0]['objects'][0]['candidates']
    assert len(candidates) == 2
    assert [candidate['candidate_id'] for candidate in candidates] == [0, 1]
    assert candidates[0]['orientation'] == [0., 1., 0., 0.]
    assert candidates[0]['position'] in scene['positions']
    assert not result['rounds'][0]['statuses']['grasp_executor']['command_attempted']


def successful_scenes(config):
    table = SyntheticTable(config, 'normal')
    policy = ClearTableSession(config)
    scenes = []
    for number in range(1, 7):
        scenes.append(snapshot(table, number))
        vision, batch = table.observation(number, 'test')
        decision = policy.observe(vision, batch)
        if decision['kind'] == 'ready':
            policy.record_motion(decision)
            table.simulate_placement(decision)
        if decision['kind'] == 'complete':
            break
    return scenes


def test_actual_runner_three_bowls_new_observations_and_config_per_round(monkeypatch, tmp_path):
    config = synthetic_config()
    scenes = successful_scenes(config)
    code, result, commands, selected = run_fake(monkeypatch, tmp_path, scenes, True, clear_table=config,
                                                extra=['--max-completed-objects', '3'])
    assert code == 0, result
    assert result['outcome'] == 'completed'
    assert result['motion_completed_count'] == result['verified_placed_count'] == result['completed_count'] == 3
    assert result['observation_count'] == 5 and len(selected) == 3
    assert result['returned_home'] and result['placement_verified']
    assert all(len(batch['objects']) == 1 for batch in selected)
    assert len(result['used_positions']) == 3 and 'A' not in result['used_positions']
    assert sum('position_control' in command[0] for command in commands) == 1
    configs = [json.loads(Path(row['config_snapshots']['grasp']).read_text())
               for row in result['rounds'] if row['config_snapshots']['grasp']]
    assert len(configs) == 3
    assert len({c['place_reference_position']['x'] for c in configs}) >= 2
    assert all(c['place_reference_orientation'] == configs[0]['place_reference_orientation'] for c in configs)
    assert all(c['place_drop_distance'] == configs[0]['place_drop_distance'] for c in configs)
    assert all(p['segments'] == 7 for p in result['preview_batches'])
    # All observations share the validated task snapshot, even if someone
    # changes the source config while this multi-round task is running.
    vision_configs = [json.loads(Path(row['config_snapshots']['vision']).read_text())
                      for row in result['rounds']]
    tray_paths = {Path(c['tray_config_path']) for c in vision_configs}
    assert len(tray_paths) == 1
    tray_snapshot = tray_paths.pop()
    assert tray_snapshot.name == 'effective_tray_config.json'
    assert tray_snapshot != Path(result['tray_config'])
    assert json.loads(tray_snapshot.read_text()) == config


def test_clear_table_preview_selects_only_one_without_claiming_or_occupying(monkeypatch, tmp_path):
    config = synthetic_config()
    scene = snapshot(SyntheticTable(config, 'normal'), 1)
    code, result, commands, selected = run_fake(monkeypatch, tmp_path, [scene], clear_table=config)
    assert code == 0 and result['outcome'] == 'planned'
    assert len(selected) == 1 and len(selected[0]['objects']) == 1
    assert result['motion_completed_count'] == result['verified_placed_count'] == 0
    assert result['used_positions'] == [] and result['last_placement'] is None
    assert not any('position_control' in command[0] for command in commands)
    verification = json.loads((tmp_path / 'output' / 'round_0001' / 'verification.json').read_text())
    assert verification['passed'] and verification['command_attempted'] is False


def test_successful_clear_table_needs_no_failure_retries(monkeypatch, tmp_path):
    config = synthetic_config()
    code, result, _, selected = run_fake(
        monkeypatch, tmp_path, successful_scenes(config), True, clear_table=config,
        extra=['--max-reobservations', '0'])
    assert code == 0, result.get('error')
    assert result['verified_placed_count'] == 3 and len(selected) == 3
    assert result['observation_count'] == 5
    assert result['reobservations_without_progress'] == 0


def test_running_task_uses_loaded_release_parameters_even_if_source_file_changes(monkeypatch, tmp_path):
    config = synthetic_config()
    root = Path(__file__).resolve().parents[1]
    zones = tmp_path / 'original_zones.json'
    zones.write_bytes((root / 'src/grasp_executor/config/place_zones.json').read_bytes())
    load = runner.load_place_zone

    def load_then_remove(path, zone, grasp):
        result = load(path, zone, grasp)
        Path(path).unlink()  # Simulate a field config being replaced mid-task.
        return result

    monkeypatch.setattr(runner, 'load_place_zone', load_then_remove)
    code, result, _, selected = run_fake(
        monkeypatch, tmp_path, successful_scenes(config), True, clear_table=config,
        extra=['--place-zones-config', str(zones)])
    assert code == 0, result.get('error')
    assert result['verified_placed_count'] == len(selected) == 3


@pytest.mark.parametrize('scenario,reason', [('tray_full', 'tray_full'), ('missing', 'synthetic_tray_not_visible')])
def test_clear_table_no_destination_has_no_controller_or_executor(monkeypatch, tmp_path, scenario, reason):
    config = synthetic_config()
    scene = snapshot(SyntheticTable(config, scenario), 1)
    code, result, commands, selected = run_fake(monkeypatch, tmp_path, [scene], True, clear_table=config)
    assert code == 1 and result['error'] == reason
    assert not selected and result['motion_completed_count'] == 0
    assert not any('position_control' in cmd[0] or 'run_grasp_executor' in cmd[0] for cmd in commands)


def test_post_action_uncertainty_stops_before_second_pick(monkeypatch, tmp_path):
    config = synthetic_config()
    scene = snapshot(SyntheticTable(config, 'unverified'), 1)
    code, result, _, selected = run_fake(monkeypatch, tmp_path, [scene], True, clear_table=config)
    assert code == 1 and result['outcome'] == 'partial' and result['error'] == 'placement_unverified'
    assert len(selected) == result['motion_completed_count'] == 1
    assert result['verified_placed_count'] == 0 and result['last_placement'] is not None


def test_all_bowls_inside_can_finish_when_rim_geometry_rejects_them(monkeypatch, tmp_path):
    config = synthetic_config()
    table = SyntheticTable(config, 'normal')
    table.sources = []
    scene = dict(snapshot(table, 1), rim_error='insufficient_points')
    code, result, commands, selected = run_fake(monkeypatch, tmp_path, [scene], True, clear_table=config)
    assert code == 0 and result['outcome'] == 'no_targets'
    assert result['observation_count'] == 2 and not selected
    assert not any('position_control' in cmd[0] or 'run_grasp_executor' in cmd[0] for cmd in commands)


def test_inside_unknown_rim_error_is_not_suppressed(monkeypatch, tmp_path):
    config = synthetic_config()
    table = SyntheticTable(config, 'normal')
    table.sources = []
    scene = dict(snapshot(table, 1), rim_error='injected_unexpected_error')
    code, result, _, selected = run_fake(monkeypatch, tmp_path, [scene], True, clear_table=config)
    assert code == 1 and result['error'] == 'localization_rejected:injected_unexpected_error'
    assert not selected


def test_remember_source_with_no_candidate_until_handled(monkeypatch, tmp_path):
    config = synthetic_config()
    scenes = successful_scenes(config)
    scene = scenes[0]
    # Only a single source is visible on the first observation; then it vanishes.
    table = SyntheticTable(config, 'normal')
    table.sources = table.sources[:1]
    scene = snapshot(table, 1)
    scene['rim_rejected_ids'] = [scene['ids'][0]]
    table.sources = []
    empty = snapshot(table, 2)
    code, result, _, selected = run_fake(monkeypatch, tmp_path, [scene, empty], True, clear_table=config)
    assert code == 1 and result['error'] in ('remaining_unknown', 'observation_objects_missing')
    assert not selected and result['remaining_sources']


def test_clear_table_motion_failure_stops_without_new_observation(monkeypatch, tmp_path):
    config = synthetic_config()
    scene = dict(snapshot(SyntheticTable(config, 'normal'), 1), executor_error='motion_timeout', command_attempted=True)
    code, result, _, selected = run_fake(monkeypatch, tmp_path, [scene], True, clear_table=config)
    assert code == 1 and result['error'] == 'motion_timeout' and len(selected) == 1
    assert result['observation_count'] == 1 and not result['returned_home']


def test_source_geometry_rejection_does_not_block_other_sources_forever(monkeypatch, tmp_path):
    config = synthetic_config()
    scenes = successful_scenes(config)
    # A farther source lacks geometry in round 1 but has it on the next fresh view.
    scenes[0]['rim_rejected_ids'] = [scenes[0]['ids'][2]]
    code, result, _, selected = run_fake(monkeypatch, tmp_path, scenes, True, clear_table=config)
    assert code == 0 and result['verified_placed_count'] == 3 and len(selected) == 3


def test_clear_table_vision_tf_failure_stays_fatal(monkeypatch, tmp_path):
    config = synthetic_config()
    scene = dict(snapshot(SyntheticTable(config, 'normal'), 1), vision_error='input_timeout')
    code, result, _, selected = run_fake(monkeypatch, tmp_path, [scene], True, clear_table=config)
    assert code == 1 and result['error'] == 'input_timeout' and not selected


def test_successful_motion_keeps_pending_slot_when_preview_delivery_fails(monkeypatch, tmp_path):
    config = synthetic_config()
    scene = dict(snapshot(SyntheticTable(config, 'normal'), 1), previews=0)
    code, result, _, selected = run_fake(monkeypatch, tmp_path, [scene], True, clear_table=config)
    assert code == 1 and result['error'] == 'preview_batch_mismatch'
    assert result['outcome'] == 'partial' and result['motion_completed_count'] == 1
    assert len(result['used_positions']) == 1 and result['last_placement'] is not None
    assert len(selected) == 1
