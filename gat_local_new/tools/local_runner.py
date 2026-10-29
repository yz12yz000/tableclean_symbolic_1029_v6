#!/usr/bin/env python3
"""Local transport only; authentication/model calls remain owned by OpenCode."""
import argparse
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = Path('/home/rob/table_clean_ws')


def module(name, relative):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative)
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')


def prepare(kind, source, baseline_policy=None, baseline_result=None):
    """Pure validation and argv construction: no robot processes or file writes."""
    if kind == 'table':
        config = (ROOT/'config/robots/table.yaml').read_text(encoding='utf-8')
        values = {}
        for line in config.splitlines():
            text = line.split('#', 1)[0].strip()
            if ':' in text:
                key, value = text.split(':', 1)
                values[key] = value.strip().strip('\"\'')
        if (values.get('transport') != 'local' or values.get('workspace') != str(WORKSPACE)
                or values.get('wrapper') != './run_clear_table.sh'
                or values.get('execution_mode') != 'local_subprocess'):
            raise ValueError('local_table_config_mismatch')
        validator = module('gat_local_table_validator', 'generated/table/table_task.py')
        policy = validator.load_policy(Path(source), require_deployment_metadata=True)
        if bool(baseline_policy) != bool(baseline_result):
            raise ValueError('baseline_requires_policy_and_result')
        if policy['mode'] == 'execute':
            if not baseline_policy:
                raise ValueError('execute_requires_successful_online_preview')
            preview_policy = validator.load_policy(Path(baseline_policy), require_deployment_metadata=True)
            preview_result = validator.load_result(Path(baseline_result))
            validator.validate_preview_baseline(policy, preview_policy, preview_result)
        elif baseline_policy:
            raise ValueError('preview_cannot_have_execute_baseline')
        # This existing validator checks UUID syntax before it reaches a path.
        identifier = policy['plan_id']
        directory = WORKSPACE / '.agent_tasks' / identifier
        command = ['bash', str(WORKSPACE/'run_clear_table.sh'), '--task-policy',
                   str(directory/'task_policy.json'), '--output-dir', str(directory/'output')]
        if policy['mode'] == 'execute':
            command += ['--execute', '--baseline-policy', str(directory/'preview_policy.json'),
                        '--baseline-result', str(directory/'preview_result.json')]
        return dict(kind=kind, mode=policy['mode'], plan_id=identifier, directory=directory,
                    argv=command, policy=policy, validator=validator)
    validator = module('gat_local_arm_validator', 'generated/arm/arm_task.py')
    plan = validator.load_plan(Path(source))
    if baseline_policy or baseline_result:
        raise ValueError('arm_does_not_use_table_baseline')
    execution = plan['local_execution']
    directory = WORKSPACE / '.agent_tasks' / plan['plan_id']
    command = ['bash', execution['parameterized_program'], *execution['program_arguments'],
               '--output-dir', str(directory/'output')]
    return dict(kind=kind, mode=plan['mode'], plan_id=plan['plan_id'], directory=directory,
                argv=command, plan=plan, validator=validator)


def run(prepared, source, baseline_policy=None, baseline_result=None):
    directory = prepared['directory']
    directory.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(source, directory/('task_policy.json' if prepared['kind']=='table' else 'task_plan.json'))
    if baseline_policy:
        shutil.copyfile(baseline_policy, directory/'preview_policy.json')
        shutil.copyfile(baseline_result, directory/'preview_result.json')
    report = dict(plan_id=prepared['plan_id'], transport='local', mode=prepared['mode'],
                  argv=prepared['argv'], execution_status='running',
                  log=str(directory/'process.log'), result_path=str(directory/'output/result.json'))
    report_path = ROOT/'reports'/f"{prepared['kind']}_last_run.json"
    if prepared['kind']=='arm':
        report['placement_resolution'] = prepared['plan']['placement_resolution']
        report['deployment'] = dict(transport='local',
            program_command=prepared['plan']['local_execution']['program_command'])
    write(directory/'transport_report.json', report)
    write(report_path, report)
    # The child inherits ROS_DOMAIN_ID and user DDS settings, as the local
    # wrapper expects. No login shell, shell interpolation, SSH or credentials.
    with (directory/'process.log').open('x') as log:
        try:
            process = subprocess.Popen(prepared['argv'], cwd=WORKSPACE,
                                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                       text=True, encoding='utf-8', errors='replace')
            try:
                for line in process.stdout:
                    log.write(line)
                    log.flush()
                    print(line, end='', flush=True)
                code = process.wait()
            except KeyboardInterrupt:
                # Child shares the foreground process group; Ctrl+C already
                # reaches its existing signal/cancel handlers. Wait, don't retry.
                code = process.wait()
                report['interrupted'] = True
        except OSError as exc:
            code = 127
            report['process_error'] = str(exc)
    report.update(exit_code=code, execution_status='exited')
    if prepared['kind']=='arm':
        report['deployment']['exit_code'] = code
    result_file = directory/'output/result.json'
    if result_file.is_file():
        try:
            result = json.loads(result_file.read_text(encoding='utf-8'))
            if not isinstance(result, dict):
                raise ValueError('runner_result_not_object')
        except (ValueError, OSError) as exc:
            report['validation_error'] = 'runner_result_invalid:' + str(exc)
            report['transport_exit_code'] = code or 2
            write(directory/'transport_report.json', report)
            write(report_path, report)
            return code or 2
        write(ROOT/'reports'/f"{prepared['kind']}_{'preview' if prepared['mode']=='preview' else 'last'}_result.json", result)
        report['runner_outcome'] = result.get('outcome')
        report['runner_passed'] = result.get('passed')
        if prepared['kind']=='table':
            try:
                baseline = prepared['validator'].load_result(Path(baseline_result)) if baseline_result else None
                prepared['validator'].validate_runner_result(prepared['policy'], result, baseline=baseline)
                report['result_identity_valid'] = True
                if (prepared['mode']=='preview' and result.get('passed') is True
                        and result.get('outcome')=='planned' and not result.get('simulated',False)):
                    # Preserve the policy that actually produced the baseline,
                    # rather than relying on a later edit of task_policy.json.
                    shutil.copyfile(directory/'task_policy.json', ROOT/'generated/table/preview_policy.json')
            except ValueError as exc:
                report['result_identity_valid'] = False
                report['validation_error'] = str(exc)
                code = code or 2
    elif code == 0:
        report['validation_error'] = 'runner_result_missing'
        code = 2
    report['transport_exit_code'] = code
    write(directory/'transport_report.json', report)
    write(report_path, report)
    return code


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('kind', choices=['arm', 'table'])
    parser.add_argument('--plan', type=Path, required=True, help='arm plan or table task_policy.json')
    parser.add_argument('--baseline-policy', type=Path)
    parser.add_argument('--baseline-result', type=Path)
    parser.add_argument('--run', action='store_true', help='without this flag, only print the validated command')
    parser.add_argument('--confirmed', action='store_true', help='record existing explicit user authorization for execute mode')
    args = parser.parse_args(argv)
    try:
        prepared = prepare(args.kind, args.plan, args.baseline_policy, args.baseline_result)
        if not args.run:
            print(json.dumps(dict(transport='local', mode=prepared['mode'], argv=prepared['argv']), ensure_ascii=False))
            return 0
        if prepared['mode']=='execute' and not args.confirmed:
            raise ValueError('execute_requires_explicit_user_confirmation')
        if prepared['directory'].exists():
            raise ValueError('task_directory_already_exists_do_not_retry')
        return run(prepared, args.plan, args.baseline_policy, args.baseline_result)
    except (ValueError, OSError, KeyError) as exc:
        print(str(exc), file=sys.stderr)
        return 2


if __name__=='__main__':
    raise SystemExit(main())
