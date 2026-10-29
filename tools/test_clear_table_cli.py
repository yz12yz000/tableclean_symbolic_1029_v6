"""Hardware-free public entry-point acceptance checks."""

import json
import os
from pathlib import Path
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[1]
ENTRY = ROOT / 'run_clear_table.sh'


def invoke(tmp_path, *args):
    env = dict(os.environ, ROS_DOMAIN_ID='intentionally_invalid_for_offline',
               MOVEIT_WS='/no/such/moveit', UR_DRIVER_WS='/no/such/ur_driver',
               UR_MODEL_WS='/no/such/ur_model', PYTHONPATH='', PYTHONNOUSERSITE='1')
    return subprocess.run([str(ENTRY), *map(str, args)], cwd=tmp_path, env=env,
                          capture_output=True, text=True, timeout=20)


def test_help_does_not_load_ros_environment(tmp_path):
    result = invoke(tmp_path, '--help')
    assert result.returncode == 0, result.stderr
    assert '--offline-demo' in result.stdout and '--execute' in result.stdout


def test_offline_clear_table_reuses_each_round_observation_and_original_motion(tmp_path):
    output = tmp_path / 'normal'
    process = invoke(tmp_path, '--offline-demo', '--output-dir', output)
    assert process.returncode == 0, process.stdout + process.stderr
    result = json.loads((output / 'result.json').read_text())
    assert result['success'] and result['simulated'] and not result['hardware_connected']
    assert result['motion_completed_count'] == result['verified_placed_count'] == 3
    assert result['observation_count'] >= 4  # Last placement also needs a fresh look.
    executed = [row for row in result['rounds'] if row['motion_count']]
    assert len(executed) == 3
    assert [row['motion_count'] for row in executed] == [7, 7, 7]
    assert [row['gripper_count'] for row in executed] == [3, 3, 3]
    assert len({row['decision']['slot']['id'] for row in executed}) == 3
    assert all(row['decision']['slot']['id'] != 'A' for row in executed)
    assert all(len(row['decision']['selected']['objects']) == 1 for row in executed)
    assert all(row['decision']['source']['center_xy'][1] > 0 for row in executed)
    assert len({row['decision']['source']['object_id'] for row in executed}) == 3
    observations = [json.loads(path.read_text()) for path in sorted(output.glob('round_*/synthetic_observation.json'))]
    assert observations[0]['tray']['center_xy'] != observations[1]['tray']['center_xy']
    assert observations[0]['source_stamp_ns'] < observations[-1]['source_stamp_ns']


@pytest.mark.parametrize('scenario, motions', [('tray_full', 0), ('missing', 0), ('unverified', 1)])
def test_offline_uncertain_scenarios_stop_without_next_pick(tmp_path, scenario, motions):
    output = tmp_path / scenario
    process = invoke(tmp_path, '--offline-demo', '--scenario', scenario, '--output-dir', output)
    assert process.returncode == 1, process.stdout + process.stderr
    result = json.loads((output / 'result.json').read_text())
    assert not result['success'] and result['reason']
    assert result['verified_placed_count'] == 0
    assert result['motion_completed_count'] == motions
    assert result['observation_count'] <= 4
    assert result['simulated_motion_count'] == motions * 7


def test_offline_rejects_hardware_flag_and_preserves_existing_output(tmp_path):
    rejected = invoke(tmp_path, '--offline-demo', '--execute')
    assert rejected.returncode == 2
    assert 'unrecognized arguments' in rejected.stderr
    existing = tmp_path / 'existing'
    existing.mkdir()
    sentinel = existing / 'keep.txt'
    sentinel.write_text('original')
    rejected = invoke(tmp_path, '--offline-demo', '--output-dir', existing)
    assert rejected.returncode == 2
    assert sentinel.read_text() == 'original'
    assert not (existing / 'result.json').exists()


def test_visual_demo_uses_variable_sizes_grid_and_two_frame_confirmation(tmp_path):
    output=tmp_path/'visual'
    p=invoke(tmp_path,'--offline-demo','--scenario','visual','--output-dir',output)
    assert p.returncode==0,p.stdout+p.stderr
    r=json.loads((output/'result.json').read_text())
    assert r['success'] and r['verified_placed_count']==3 and r['observation_count']==8
    motions=[row for row in r['rounds'] if row['motion_count']]
    assert [row['motion_count'] for row in motions]==[7,7,7]
    assert [row['decision']['source']['radius'] for row in motions]==[.075,.06,.05]
    assert all(row['decision']['slot']['id'].startswith('grid:') for row in motions)
    assert not r['hardware_connected']
