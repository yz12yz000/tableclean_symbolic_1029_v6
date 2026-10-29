#!/usr/bin/env python3
"""Bounded local sessions. One supervised runner for the entire table task."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import uuid
import local_runner

ROOT = local_runner.ROOT
WORKSPACE = local_runner.WORKSPACE
sys.path.insert(0, str(ROOT/'generated/table'))
from table_round_protocol import validate_bundle, validate_decision, IDENTITY


def read(path):
    def unique(pairs):
        result = {}
        for k, v in pairs:
            if k in result:
                raise ValueError('duplicate_json_key')
            result[k] = v
        return result
    return json.loads(Path(path).read_text(), object_pairs_hook=unique)


def atomic(path, value, replace=False):
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(dir=path.parent, prefix='.'+path.name)
    try:
        with os.fdopen(fd, 'w') as f:
            json.dump(value, f, ensure_ascii=False, allow_nan=False, indent=2)
            f.flush()
            os.fsync(f.fileno())
        if replace:
            os.replace(name, path)
        else:
            os.link(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def directory(plan_id):
    if str(uuid.UUID(plan_id)) != plan_id:
        raise ValueError('invalid_plan_id')
    return WORKSPACE/'.agent_tasks'/plan_id


def start(policy, baseline_policy=None, baseline_result=None, confirmed=False):
    prepared = local_runner.prepare('table', policy, baseline_policy, baseline_result)
    if prepared['mode'] == 'execute' and not confirmed:
        raise ValueError('execute_requires_existing_user_authorization')
    d = directory(prepared['plan_id'])
    d.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(policy, d/'task_policy.json')
    if baseline_policy:
        shutil.copyfile(baseline_policy, d/'preview_policy.json')
        shutil.copyfile(baseline_result, d/'preview_result.json')
    (d/'decisions').mkdir()
    argv = prepared['argv'] + ['--agent-decision-dir', str(d/'decisions'), '--agent-round-protocol', 'v2']
    atomic(d/'session.json', dict(plan_id=prepared['plan_id'], argv=argv, mode=prepared['mode']))
    with (d/'worker.log').open('x') as log:
        process = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), '_worker', '--plan-id', prepared['plan_id']],
            cwd=ROOT, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    atomic(d/'worker.json', dict(pid=process.pid, start_ticks=process_ticks(process.pid)))
    return dict(state='running', plan_id=prepared['plan_id'], session=str(d), mode=prepared['mode'])


def process_ticks(pid):
    try:
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        return fields[19] if fields[0] != 'Z' else None
    except (OSError, IndexError):
        return None


def worker(plan_id):
    d = directory(plan_id)
    session = read(d/'session.json')
    # Reconstruct the fixed command; never execute argv supplied in a report.
    policy = read(d/'task_policy.json')
    baseline = d/'preview_policy.json' if policy['mode'] == 'execute' else None
    prepared = local_runner.prepare('table', d/'task_policy.json', baseline, d/'preview_result.json' if baseline else None)
    argv = prepared['argv'] + ['--agent-decision-dir', str(d/'decisions'), '--agent-round-protocol', 'v2']
    try:
        with (d/'process.log').open('x') as log:
            process = subprocess.Popen(argv, cwd=WORKSPACE, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT)
            code = process.wait()
        report = dict(state='finished', plan_id=plan_id, exit_code=code, mode=session['mode'])
        target = ROOT/'reports/table'/plan_id
        target.mkdir(parents=True, exist_ok=True)
        for source in sorted((d/'output').glob('round_*')):
            if not source.is_dir():
                continue
            dest = target/source.name
            dest.mkdir(exist_ok=True)
            for name in ('camera_rgb.jpg', 'agent_overlay.jpg', 'scene.json', 'agent_request.json',
                         'agent_decision.json', 'result.json', 'execution.json', 'verification.json'):
                if (source/name).is_file() and not (dest/name).exists():
                    shutil.copyfile(source/name, dest/name)
        if (d/'output/result.json').exists():
            shutil.copyfile(d/'output/result.json', target/'result.json')
        result = read(d/'output/result.json')
        prepared['validator'].validate_runner_result(policy, result,
            baseline=read(d/'preview_result.json') if baseline else None)
        report.update(result_identity_valid=True, outcome=result.get('outcome'), passed=result.get('passed'),
                      result=str(ROOT/'reports/table'/plan_id/'result.json'))
        target = ROOT/'reports/table'/plan_id
        target.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(d/'output/result.json', target/'result.json')
        shutil.copyfile(d/'task_policy.json', target/'task_policy.json')
        if code == 0 and session['mode'] == 'preview' and result.get('passed') and result.get('outcome') == 'planned':
            shutil.copyfile(d/'task_policy.json', ROOT/'generated/table/preview_policy.json')
            shutil.copyfile(d/'output/result.json', ROOT/'reports/table_preview_result.json')
    except Exception as exc:
        report = dict(state='uncertain', plan_id=plan_id, error=str(exc), exit_code=locals().get('code'))
    atomic(d/'worker_result.json', report)
    atomic(ROOT/'reports/table_last_run.json', report, replace=True)
    return report


def poll(plan_id):
    d = directory(plan_id)
    if (d/'worker_result.json').exists():
        return read(d/'worker_result.json')
    owner = read(d/'worker.json')
    if owner['start_ticks'] is None or process_ticks(owner['pid']) != owner['start_ticks']:
        return dict(state='uncertain', plan_id=plan_id, reason='worker_lost_do_not_restart')
    if not (d/'agent_status.json').exists():
        return dict(state='running', plan_id=plan_id)
    status = read(d/'agent_status.json')
    if status['plan_id'] != plan_id:
        raise ValueError('status_plan_id')
    if status['state'] != 'waiting_agent':
        return dict(state='running', plan_id=plan_id)
    import re
    name = status.get('round', '')
    if not re.fullmatch(r'round_[0-9]{4,}', name):
        raise ValueError('status_round')
    source = d/'output'/name
    request = read(source/'agent_request.json')
    if any(request[k] != status[k] for k in IDENTITY):
        raise ValueError('status_identity')
    validate_bundle(source, request)
    dest = ROOT/'reports/table'/plan_id/name
    dest.mkdir(parents=True, exist_ok=True)
    for file in ['camera_rgb.jpg', 'agent_overlay.jpg', 'scene.json', 'agent_request.json']:
        if not (dest/file).exists():
            shutil.copyfile(source/file, dest/file)
    validate_bundle(dest, request)
    return dict(status, request=str(dest/'agent_request.json'),
                images=[str(dest/n) for n in ('camera_rgb.jpg', 'agent_overlay.jpg')],
                scene=str(dest/'scene.json'))


def submit(plan_id, decision_path):
    d = directory(plan_id)
    decision = read(decision_path)
    status = poll(plan_id)
    if status['state'] != 'waiting_agent':
        raise ValueError('request_not_waiting_or_consumed')
    request = read(status['request'])
    validate_decision(decision, request)
    path = d/'decisions'/(status['round']+'.json')
    if path.exists():
        if read(path) != decision:
            raise ValueError('conflicting_decision')
        return dict(state='submitted', duplicate=True, plan_id=plan_id)
    # Recheck live pointer immediately before publishing. Runner independently checks identity.
    if read(d/'agent_status.json') != {k: status[k] for k in ('state', *IDENTITY, 'round')}:
        raise ValueError('request_changed')
    try:
        atomic(path, decision)
    except FileExistsError:
        if read(path) != decision:
            raise ValueError('conflicting_decision')
    saved = ROOT/'generated/table'/plan_id/'decisions'/path.name
    if not saved.exists():
        atomic(saved, decision)
    return dict(state='submitted', plan_id=plan_id, observation_id=decision['observation_id'])


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('action', choices=['start', 'poll', 'submit', 'cancel', '_worker'])
    p.add_argument('--policy', type=Path)
    p.add_argument('--baseline-policy', type=Path)
    p.add_argument('--baseline-result', type=Path)
    p.add_argument('--confirmed', action='store_true')
    p.add_argument('--plan-id')
    p.add_argument('--decision', type=Path)
    a = p.parse_args()
    try:
        if a.action == 'start':
            value = start(a.policy, a.baseline_policy, a.baseline_result, a.confirmed)
        elif a.action == 'poll':
            value = poll(a.plan_id)
        elif a.action == 'submit':
            value = submit(a.plan_id, a.decision)
        elif a.action == '_worker':
            value = worker(a.plan_id)
        else:
            d = directory(a.plan_id)
            read(d/'session.json')
            if not (d/'cancel.json').exists():
                atomic(d/'cancel.json', dict(plan_id=a.plan_id, cancel=True))
            value = dict(state='cancel_requested', plan_id=a.plan_id,
                         note='Poll the original worker; cancellation is not proof of physical stop.')
        print(json.dumps(value, ensure_ascii=False))
    except (ValueError, OSError, KeyError, TypeError) as exc:
        print(json.dumps(dict(error=str(exc)), ensure_ascii=False))
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
