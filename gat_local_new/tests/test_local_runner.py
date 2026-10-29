"""Local transport checks use mocked processes only; never dispatch ROS/motion."""
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import uuid

PROJECT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('local_runner', PROJECT/'tools/local_runner.py')
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


class LocalRunnerTests(unittest.TestCase):
    def test_real_local_subprocess_transport_with_synthetic_result(self):
        prepared=runner.prepare('table',PROJECT/'generated/table/task_policy.json')
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);prepared['directory']=root/'task'
            result=dict(plan_id=prepared['plan_id'],passed=False,outcome='failed',error='synthetic_device_unavailable',
                preflight=dict(runner_protocol_version='table-runner-v1',runner_revision='table-clean-runner-v2',
                    workspace_revision=prepared['policy']['workspace_revision'],config_digest='a'*64,
                    model_digest='b'*64,policy_hash=prepared['validator'].policy_hash(prepared['policy'])))
            script="from pathlib import Path; import sys; p=Path(sys.argv[1]); p.parent.mkdir(); p.write_text(sys.argv[2]); print('synthetic transport only'); sys.exit(1)"
            prepared['argv']=[__import__('sys').executable,'-c',script,
                              str(prepared['directory']/'output/result.json'),json.dumps(result)]
            with patch.object(runner,'ROOT',root/'client'),patch.object(runner,'WORKSPACE',root):
                self.assertEqual(runner.run(prepared,PROJECT/'generated/table/task_policy.json'),1)
            report=json.loads((root/'client/reports/table_last_run.json').read_text())
            self.assertTrue(report['result_identity_valid'])
            self.assertFalse(report['runner_passed'])
            self.assertEqual(report['exit_code'],1)
            self.assertIn('synthetic transport only',(prepared['directory']/'process.log').read_text())

    def test_start_script_only_forwards_to_opencode_in_project(self):
        with tempfile.TemporaryDirectory() as folder:
            fake=Path(folder)/'opencode'
            fake.write_text('#!/bin/bash\npwd\nprintf "%s\\n" "$@"\n')
            fake.chmod(0o755)
            env=dict(__import__('os').environ)
            env['PATH']=folder+':'+env['PATH']
            completed=__import__('subprocess').run([str(PROJECT/'start.sh'),'--help'],
                env=env,cwd='/tmp',text=True,capture_output=True,check=True)
            self.assertEqual(completed.stdout.splitlines(),[str(PROJECT),'--help'])

    def test_table_execute_builds_local_argv_with_matching_preview_baseline(self):
        table=runner.module('execute_table_validator','generated/table/table_task.py')
        preview=table.load_policy(PROJECT/'generated/table/task_policy.json',require_deployment_metadata=True)
        execute=json.loads(json.dumps(preview));execute.update(mode='execute',plan_id=str(uuid.uuid4()))
        execute['model']['plan_hash']=table.policy_hash(execute)
        result=dict(plan_id=preview['plan_id'],passed=True,outcome='planned',mode='plan_only',
            selection='clear_table',hardware_motion_requested=False,preflight=dict(
                runner_protocol_version='table-runner-v1',runner_revision='table-clean-runner-v2',
                workspace_revision=preview['workspace_revision'],config_digest='a'*64,model_digest='b'*64,
                policy_hash=table.policy_hash(preview)))
        with tempfile.TemporaryDirectory() as folder:
            folder=Path(folder)
            for name,value in [('execute.json',execute),('preview.json',preview),('result.json',result)]:
                (folder/name).write_text(json.dumps(value))
            prepared=runner.prepare('table',folder/'execute.json',folder/'preview.json',folder/'result.json')
            self.assertIn('--execute',prepared['argv'])
            self.assertIn('--baseline-policy',prepared['argv'])
            self.assertNotIn('ssh',prepared['argv'])

    def test_default_print_has_no_subprocess_or_task_creation(self):
        with patch.object(runner.subprocess, 'Popen', side_effect=AssertionError('must not launch')):
            for kind, path in [('arm','generated/arm/task_plan.json'),('table','generated/table/task_policy.json')]:
                with patch('sys.stdout', new=io.StringIO()):
                    self.assertEqual(runner.main([kind,'--plan',str(PROJECT/path)]),0)
                prepared=runner.prepare(kind,PROJECT/path)
                self.assertNotIn('ssh',prepared['argv'])
                self.assertEqual(prepared['argv'][0],'bash')
                self.assertNotIn('--execute',prepared['argv'])

    def test_arm_preview_selects_plan_only_wrapper(self):
        prepared=runner.prepare('arm',PROJECT/'generated/arm/task_plan.json')
        self.assertEqual(prepared['argv'][1],'/home/rob/table_clean_ws/run_pipeline_place_test.sh')
        self.assertEqual(prepared['plan']['robot']['transport'],'local')
        self.assertNotIn('ssh_command',prepared['plan']['local_execution'])

    def test_execute_requires_authorization_without_starting_process(self):
        arm=runner.module('test_local_arm','generated/arm/arm_task.py')
        plan=arm.create_plan('右侧',provider='test',model_name='test',mode='execute')
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'plan.json';path.write_text(json.dumps(plan))
            with patch.object(runner.subprocess,'Popen',side_effect=AssertionError('must not launch')):
                with patch('sys.stderr',new=io.StringIO()):
                    self.assertEqual(runner.main(['arm','--plan',str(path),'--run']),2)

    def test_table_execute_requires_online_preview(self):
        table=runner.module('test_local_table','generated/table/table_task.py')
        policy=table.load_policy(PROJECT/'generated/table/task_policy.json',require_deployment_metadata=True)
        policy.update(mode='execute',plan_id=str(uuid.uuid4()))
        policy['model']['plan_hash']=table.policy_hash(policy)
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'policy.json';path.write_text(json.dumps(policy))
            with self.assertRaisesRegex(ValueError,'execute_requires_successful_online_preview'):
                runner.prepare('table',path)

    def test_local_copy_process_failure_and_report_preserve_exit_code(self):
        prepared=runner.prepare('table',PROJECT/'generated/table/task_policy.json')
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);prepared['directory']=root/'workspace/.agent_tasks'/prepared['plan_id']
            fake=type('Process',(),{'stdout':io.StringIO('mock failure\n'),'wait':lambda self:7})()
            with patch.object(runner,'ROOT',root/'client'), patch.object(runner,'WORKSPACE',root/'workspace'):
                with patch.object(runner.subprocess,'Popen',return_value=fake) as popen:
                    self.assertEqual(runner.run(prepared,PROJECT/'generated/table/task_policy.json'),7)
                    self.assertEqual(popen.call_args.kwargs['cwd'],root/'workspace')
                    self.assertNotIn('shell',popen.call_args.kwargs)
                report=json.loads((root/'client/reports/table_last_run.json').read_text())
                self.assertEqual(report['exit_code'],7)
                self.assertEqual(report['transport'],'local')
                self.assertTrue((prepared['directory']/'task_policy.json').is_file())

    def test_success_without_result_is_not_reported_as_success(self):
        prepared=runner.prepare('table',PROJECT/'generated/table/task_policy.json')
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);prepared['directory']=root/'task'
            fake=type('Process',(),{'stdout':io.StringIO(''),'wait':lambda self:0})()
            with patch.object(runner,'ROOT',root/'client'),patch.object(runner.subprocess,'Popen',return_value=fake):
                self.assertEqual(runner.run(prepared,PROJECT/'generated/table/task_policy.json'),2)
                report=json.loads((root/'client/reports/table_last_run.json').read_text())
                self.assertEqual(report['validation_error'],'runner_result_missing')

    def test_duplicate_task_does_not_launch(self):
        with tempfile.TemporaryDirectory() as folder:
            prepared=runner.prepare('table',PROJECT/'generated/table/task_policy.json')
            prepared['directory']=Path(folder)/'existing'
            prepared['directory'].mkdir()
            with patch.object(runner,'prepare',return_value=prepared):
                with patch.object(runner.subprocess,'Popen',side_effect=AssertionError('must not launch')):
                    with patch('sys.stderr',new=io.StringIO()):
                        self.assertEqual(runner.main(['table','--plan',str(PROJECT/'generated/table/task_policy.json'),'--run']),2)

    def test_successful_local_result_is_validated_and_preview_policy_preserved(self):
        prepared=runner.prepare('table',PROJECT/'generated/table/task_policy.json')
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);prepared['directory']=root/'task'
            result=dict(plan_id=prepared['plan_id'],passed=True,outcome='planned',preflight=dict(
                runner_protocol_version='table-runner-v1',runner_revision='table-clean-runner-v2',
                workspace_revision=prepared['policy']['workspace_revision'],config_digest='a'*64,
                model_digest='b'*64,policy_hash=prepared['validator'].policy_hash(prepared['policy'])))
            def launch(*args,**kwargs):
                output=prepared['directory']/'output';output.mkdir()
                (output/'result.json').write_text(json.dumps(result))
                return type('Process',(),{'stdout':io.StringIO(''),'wait':lambda self:0})()
            (root/'client/generated/table').mkdir(parents=True)
            with patch.object(runner,'ROOT',root/'client'),patch.object(runner.subprocess,'Popen',side_effect=launch):
                self.assertEqual(runner.run(prepared,PROJECT/'generated/table/task_policy.json'),0)
            self.assertEqual(json.loads((root/'client/reports/table_preview_result.json').read_text()),result)
            self.assertEqual(json.loads((root/'client/generated/table/preview_policy.json').read_text()),prepared['policy'])

    def test_invalid_result_keeps_report_and_nonzero_status(self):
        prepared=runner.prepare('table',PROJECT/'generated/table/task_policy.json')
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);prepared['directory']=root/'task'
            def launch(*args,**kwargs):
                output=prepared['directory']/'output';output.mkdir()
                (output/'result.json').write_text('{broken')
                return type('Process',(),{'stdout':io.StringIO(''),'wait':lambda self:0})()
            with patch.object(runner,'ROOT',root/'client'),patch.object(runner.subprocess,'Popen',side_effect=launch):
                self.assertEqual(runner.run(prepared,PROJECT/'generated/table/task_policy.json'),2)
            report=json.loads((root/'client/reports/table_last_run.json').read_text())
            self.assertIn('runner_result_invalid',report['validation_error'])

    def test_opencode_model_and_auth_are_not_overridden(self):
        config=json.loads((PROJECT/'opencode.json').read_text())
        self.assertNotIn('model',config)
        self.assertNotIn('provider',config)
        self.assertTrue(all('model' not in v for v in config['agent'].values()))


if __name__=='__main__':
    unittest.main()
