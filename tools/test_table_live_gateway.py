"""Gateway preview -> execute contract through the real runner, mocked devices."""
from copy import deepcopy
from dataclasses import replace
import importlib.util
import json
from pathlib import Path
import sys
import uuid

import pytest

from clear_table_offline import synthetic_config
from table_agent_protocol import policy_hash
from table_runtime_metadata import build_preflight
from test_clear_table_runner import successful_scenes
from test_place_runner import run_fake
import test_live_place_pipeline as runner

ROOT = Path(__file__).resolve().parents[1]
GATEWAYS = [ROOT / 'gat_local_new', ROOT / '6_GAT_Edu', ROOT / '6_GAT_Edu/gat-arm-minimal']


def helper(gateway):
    spec = importlib.util.spec_from_file_location('gateway_table', gateway / 'generated/table/table_task.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_policy(directory, gateway, mode):
    policy = json.loads((gateway / 'generated/table/task_policy.json').read_text())
    policy.update(plan_id=str(uuid.uuid4()), mode=mode)
    policy['model']['plan_hash'] = policy_hash(policy)
    path = directory / (mode + '_policy.json')
    path.write_text(json.dumps(policy))
    return path, policy


@pytest.mark.parametrize('gateway', GATEWAYS, ids=['local_v2', 'legacy_edu', 'legacy_minimal'])
def test_gateway_online_preview_then_new_execute_and_result(monkeypatch, tmp_path, gateway):
    config = synthetic_config()
    tray = tmp_path / 'tray.json'
    tray.write_text(json.dumps(config))
    preview_path, preview_policy = write_policy(tmp_path, gateway, 'preview')
    execute_path, execute_policy = write_policy(tmp_path, gateway, 'execute')
    preview_dir, execute_dir = tmp_path / 'preview', tmp_path / 'execute'
    preview_dir.mkdir()
    execute_dir.mkdir()
    scenes = successful_scenes(config)
    code, preview, commands, selected = run_fake(
        monkeypatch, preview_dir, scenes, clear_table=config,
        extra=['--tray-config', str(tray), '--task-policy', str(preview_path)])
    assert code == 0 and preview['outcome'] == 'planned'
    assert preview['completed_count'] == 0
    assert not any('position_control' in c[0] for c in commands)
    baseline = preview_dir / 'output/result.json'
    code, result, commands, selected = run_fake(
        monkeypatch, execute_dir, scenes, True, clear_table=config,
        extra=['--tray-config', str(tray), '--task-policy', str(execute_path),
               '--baseline-policy', str(preview_path), '--baseline-result', str(baseline)])
    assert code == 0 and result['verified_placed_count'] == len(selected) == 3
    assert preview['preflight']['config_digest'] == result['preflight']['config_digest']
    assert preview['plan_id'] != result['plan_id']
    validator = helper(gateway)
    if gateway.name != 'gat_local_new':
        # Unmodified v1 deployments must reject the upgraded runner evidence.
        with pytest.raises(ValueError, match='runner_revision_mismatch'):
            validator.validate_preview_baseline(execute_policy, preview_policy, preview)
        return
    assert validator.validate_preview_baseline(execute_policy, preview_policy, preview)
    assert validator.validate_runner_result(execute_policy, result, baseline=preview)
    assert validator.main(['--task-policy', str(execute_path),
                           '--baseline-policy', str(preview_path), '--baseline-result', str(baseline),
                           '--result', str(execute_dir / 'output/result.json'),
                           '--require-deployment-metadata']) == 0


@pytest.mark.parametrize('tamper', ['missing', 'config', 'model', 'policy', 'simulated', 'failed'])
def test_bad_baseline_is_rejected_before_ros_or_control(monkeypatch, tmp_path, tamper):
    gateway = GATEWAYS[1]
    config = synthetic_config()
    tray = tmp_path / 'tray.json'
    tray.write_text(json.dumps(config))
    preview_path, preview_policy = write_policy(tmp_path, gateway, 'preview')
    execute_path, execute_policy = write_policy(tmp_path, gateway, 'execute')
    preview_dir = tmp_path / 'preview'
    preview_dir.mkdir()
    code, preview, _, _ = run_fake(
        monkeypatch, preview_dir, successful_scenes(config), clear_table=config,
        extra=['--tray-config', str(tray), '--task-policy', str(preview_path)])
    assert code == 0
    baseline = tmp_path / 'baseline.json'
    if tamper in ('config', 'model'):
        preview['preflight'][tamper + '_digest'] = '0' * 64
    elif tamper == 'policy':
        preview['preflight']['policy_hash'] = '0' * 64
    elif tamper == 'simulated':
        preview['simulated'] = True
    elif tamper == 'failed':
        preview['passed'] = False
    baseline.write_text(json.dumps(preview))
    argv = ['runner', '--clear-table', '--execute', '--tray-config', str(tray),
            '--task-policy', str(execute_path), '--output-dir', str(tmp_path / 'execute')]
    if tamper != 'missing':
        argv += ['--baseline-policy', str(preview_path), '--baseline-result', str(baseline)]
    monkeypatch.setattr(sys, 'argv', argv)
    monkeypatch.setattr(runner.rclpy, 'init', lambda **kw: pytest.fail('must reject before ROS'))
    monkeypatch.setattr(runner.subprocess, 'Popen', lambda *a, **kw: pytest.fail('must not start controls'))
    with pytest.raises(SystemExit) as error:
        runner.main()
    assert error.value.code == 2
    assert not (tmp_path / 'execute').exists()


def test_config_digest_ignores_mode_but_detects_motion_parameter_change():
    grasp = runner.load_grasp_config(ROOT / 'src/grasp_executor/config/place_config.json')
    a = build_preflight(root=ROOT, grasp=replace(grasp, plan_only=True))
    b = build_preflight(root=ROOT, grasp=replace(grasp, plan_only=False))
    c = build_preflight(root=ROOT, grasp=replace(grasp, place_drop_distance=.1))
    assert a['config_digest'] == b['config_digest']
    assert a['config_digest'] != c['config_digest']


def test_unapproved_motion_stays_blocked_before_ros(monkeypatch, tmp_path, capsys):
    config = synthetic_config()
    config['motion_approved'] = False
    tray = tmp_path / 'tray.json'
    tray.write_text(json.dumps(config))
    preview_path, _ = write_policy(tmp_path, GATEWAYS[1], 'preview')
    execute_path, _ = write_policy(tmp_path, GATEWAYS[1], 'execute')
    preview_dir = tmp_path / 'preview'
    preview_dir.mkdir()
    code, _, _, _ = run_fake(
        monkeypatch, preview_dir, successful_scenes(config), clear_table=config,
        extra=['--tray-config', str(tray), '--task-policy', str(preview_path)])
    assert code == 0
    monkeypatch.setattr(sys, 'argv', [
        'runner', '--clear-table', '--execute', '--tray-config', str(tray),
        '--task-policy', str(execute_path), '--output-dir', str(tmp_path / 'execute'),
        '--baseline-policy', str(preview_path),
        '--baseline-result', str(preview_dir / 'output/result.json')])
    monkeypatch.setattr(runner.rclpy, 'init', lambda **kw: pytest.fail('unapproved motion must not start ROS'))
    with pytest.raises(SystemExit) as error:
        runner.main()
    assert error.value.code == 2
    assert 'tray_motion_not_approved' in capsys.readouterr().err


@pytest.mark.parametrize('gateway', GATEWAYS, ids=['local_v2', 'legacy_edu', 'legacy_minimal'])
def test_result_cannot_omit_plan_identity_or_policy_hash(gateway):
    validator = helper(gateway)
    policy = json.loads((gateway / 'generated/table/task_policy.json').read_text())
    result = dict(plan_id=policy['plan_id'], preflight=dict(
        runner_protocol_version='table-runner-v1', runner_revision='table-clean-runner-v1',
        workspace_revision=policy['workspace_revision'], config_digest='a' * 64,
        model_digest='b' * 64, policy_hash=policy_hash(policy)))
    for key in ('plan_id', 'policy_hash'):
        changed = deepcopy(result)
        (changed if key == 'plan_id' else changed['preflight']).pop(key)
        with pytest.raises(validator.PolicyError):
            validator.validate_runner_result(policy, changed)
