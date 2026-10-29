"""Local two-workspace contract checks; no ROS, camera, network, or robot."""

import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

from table_agent_protocol import (WORKSPACE_REVISION, policy_hash,
                                  validate_task_policy)


ROOT = Path(__file__).resolve().parents[1]
POLICY = ROOT / '6_GAT_Edu/generated/table/task_policy.json'
EDU_VALIDATOR = ROOT / '6_GAT_Edu/generated/table/table_task.py'
ENTRY = ROOT / 'run_clear_table.sh'


def _env():
    env = dict(os.environ)
    env.update(PYTHONPATH=str(ROOT / 'tools'), PYTHONNOUSERSITE='1')
    return env


def test_edu_policy_is_accepted_by_both_validators():
    policy = json.loads(POLICY.read_text(encoding='utf-8'))
    accepted = validate_task_policy(policy, expected_mode='preview',
                                    require_deployment_metadata=True,
                                    expected_workspace_revision=WORKSPACE_REVISION)
    assert policy_hash(accepted) == policy['model']['plan_hash']
    checked = subprocess.run(
        [sys.executable, str(EDU_VALIDATOR), '--task-policy', str(POLICY),
         '--validate', '--hash', '--require-deployment-metadata'],
        cwd=ROOT, env=_env(), capture_output=True, text=True, check=False)
    assert checked.returncode == 0, checked.stderr
    assert policy['model']['plan_hash'] in checked.stdout


def test_offline_runner_result_requires_updated_gateway(tmp_path):
    validator = ROOT/'gat_local_new/generated/table/table_task.py'
    output = tmp_path / 'run'
    process = subprocess.run(
        [str(ENTRY), '--offline-demo', '--task-policy', str(POLICY),
         '--output-dir', str(output)],
        cwd=ROOT, env=_env(), capture_output=True, text=True, timeout=30,
        check=False)
    assert process.returncode == 0, process.stdout + process.stderr
    result = json.loads((output / 'result.json').read_text(encoding='utf-8'))
    assert result['plan_id'] == json.loads(POLICY.read_text())['plan_id']
    preflight = result['preflight']
    assert preflight['policy_hash'] == policy_hash(json.loads(POLICY.read_text()))
    checked = subprocess.run(
        [sys.executable, str(validator), '--task-policy', str(POLICY),
         '--result', str(output / 'result.json'), '--require-deployment-metadata'],
        cwd=ROOT, env=_env(), capture_output=True, text=True, check=False)
    assert checked.returncode == 0, checked.stderr

    baseline = tmp_path / 'baseline.json'
    baseline.write_text(json.dumps(result), encoding='utf-8')
    checked = subprocess.run(
        [sys.executable, str(validator), '--task-policy', str(POLICY),
         '--result', str(output / 'result.json'), '--baseline-result', str(baseline),
         '--require-deployment-metadata'],
        cwd=ROOT, env=_env(), capture_output=True, text=True, check=False)
    assert checked.returncode == 0, checked.stderr

    changed = json.loads(baseline.read_text(encoding='utf-8'))
    changed['preflight']['config_digest'] = 'f' * 64
    baseline.write_text(json.dumps(changed), encoding='utf-8')
    checked = subprocess.run(
        [sys.executable, str(validator), '--task-policy', str(POLICY),
         '--result', str(output / 'result.json'), '--baseline-result', str(baseline),
         '--require-deployment-metadata'],
        cwd=ROOT, env=_env(), capture_output=True, text=True, check=False)
    assert checked.returncode == 2


def test_tampered_policy_and_result_are_rejected(tmp_path):
    policy = json.loads(POLICY.read_text(encoding='utf-8'))
    tampered_policy = tmp_path / 'tampered-policy.json'
    policy['reason'] = 'tampered'
    tampered_policy.write_text(json.dumps(policy), encoding='utf-8')
    checked = subprocess.run(
        [sys.executable, str(EDU_VALIDATOR), '--task-policy', str(tampered_policy),
         '--require-deployment-metadata'],
        cwd=ROOT, env=_env(), capture_output=True, text=True, check=False)
    assert checked.returncode == 2
    with pytest.raises(ValueError, match='model.plan_hash_mismatch'):
        validate_task_policy(policy, require_deployment_metadata=True,
                             expected_workspace_revision=WORKSPACE_REVISION)

    result = {'plan_id': policy['plan_id'], 'preflight': {
        'runner_protocol_version': 'table-runner-v1',
        'runner_revision': 'table-clean-runner-v1',
        'workspace_revision': WORKSPACE_REVISION,
        'config_digest': '0' * 64,
        'model_digest': '0' * 64,
        'policy_hash': policy_hash(policy),
    }}
    result_path = tmp_path / 'result.json'
    result_path.write_text(json.dumps(result), encoding='utf-8')
    checked = subprocess.run(
        [sys.executable, str(EDU_VALIDATOR), '--task-policy', str(POLICY),
         '--result', str(result_path), '--require-deployment-metadata'],
        cwd=ROOT, env=_env(), capture_output=True, text=True, check=False)
    assert checked.returncode == 2
