"""Exercise the actual runner with fake ROS, processes and a virtual clock."""
import json
import signal
import sys
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
import test_live_place_pipeline as runner


def run_fake(monkeypatch, tmp_path, scenes, execute=False, extra=(), new_run=False,
             overlay=None, busy=(), stale_callbacks=False, cleanup_failure=False,
             single_zone=None, release_xyz=None, controller_failure=None, clear_table=None, v2_images=False):
    clock, subscriptions, processes, commands, selected = [1.], {}, [], [], []
    root = Path(runner.__file__).resolve().parents[1]
    override = tmp_path / 'vision.json'
    override.write_text(json.dumps(overlay or {'multi_instance': False}))
    single_bowl = single_zone is not None or release_xyz is not None
    zone_args = ['--legacy-multi'] if not single_bowl else []
    if clear_table is not None:
        tray_path = tmp_path / 'tray.json'
        tray_path.write_text(json.dumps(clear_table))
        zone_args = ['--clear-table', '--tray-config', str(tray_path)]
    if release_xyz is not None:
        zone_args = [item for flag, value in zip(('--x', '--y', '--z'), release_xyz)
                     for item in (flag, str(value))]
    if single_zone is not None:
        zone_path = tmp_path / 'zones.json'
        zones = json.loads((root / 'src/grasp_executor/config/place_zones.json').read_text())
        grasp_values = json.loads((root / 'src/grasp_executor/config/place_config.json').read_text())
        for index, name in enumerate(('左上', '左下', '中间', '右上', '右下', 'blue')):
            taught = zones['zones'][name]
            taught.update(enabled=True,
                place_drop_distance={"左上":.11,"左下":.1,"中间":.1,"右上":.05,"右下":.12,"blue":.1}[name],
                place_reference_position=dict(grasp_values['place_reference_position'],
                    x=grasp_values['place_reference_position']['x'] + index * .01),
                place_reference_orientation=grasp_values['place_reference_orientation'],
                place_ready_position=dict(grasp_values['place_ready_position'],
                    x=grasp_values['place_ready_position']['x'] + index * .01),
                place_ready_orientation=grasp_values['place_ready_orientation'])
        zone_path.write_text(json.dumps(zones))
        zone_args = ['--target-zone', single_zone, '--place-zones-config', str(zone_path)]
    original_paths = [root / f'src/{name}/config/place_config.json'
                      for name in ('yolo_vision', 'rim_locator', 'grasp_executor')] + [override]
    originals = {p: p.read_bytes() for p in original_paths}
    monkeypatch.chdir(root / 'New_run' if new_run else root)
    states, old_callbacks, publishers = {}, [], []

    def ros_now(): return int((1000. + clock[0]) * 1e9)

    def send_status(state, name, value, **details):
        payload = {'status': value, 'task_id': state['task_id'],
                   'observation_id': state['observation_id'], **details}
        if v2_images and name == 'yolo_vision' and value in ('success', 'no_targets', 'failed'):
            stamp = payload['source_stamp_ns']
            payload['rgbd_pair'] = dict(rgb_stamp_ns=stamp, depth_stamp_ns=stamp,
                rgb_frame_id='camera', depth_frame_id='camera', width=200, height=200, sync_slop_s=.1)
            if 'tray' in payload:
                payload['tray'] = dict(payload['tray'], base_xy_to_pixel=[[100,0,50],[0,100,50],[0,0,1]],
                    tray_polygon_px=[[0,0],[199,0],[199,199],[0,199]])
            subscriptions[state['namespace'] + '/yolo_vision/camera_rgb'](NS(
                header=NS(stamp=NS(sec=stamp//10**9, nanosec=stamp%10**9), frame_id='camera'),
                width=200, height=200, step=600, encoding='rgb8', data=bytes(120000)))
        subscriptions[state['namespace'] + '/' + name + '/status'](NS(data=json.dumps(payload)))

    def batch_for(state):
        scene = state['scene']
        positions = scene.get('positions', [[.3 + .1*i, .1, .2] for i in range(scene.get('count', 1))])
        ids = scene.get('ids', list(range(len(positions))))
        return {'schema_version': 1, 'task_id': state['task_id'], 'observation_id': state['observation_id'],
                'source_stamp_ns': ros_now(), 'frame_id': 'base_link', 'tool_frame': 'grasp_center',
                'objects': [{'object_id': oid, 'distance': runner.math.hypot(*pos[:2]),
                    'candidates': [{'candidate_id': j, 'position': [pos[0], pos[1]+j*.005, pos[2]],
                        'orientation': [0., 1., 0., 0.], 'geometry': {}} for j in range(2)]}
                    for oid, pos in zip(ids, positions)], 'rejected': []}

    class Publisher:
        def __init__(self, topic, qos):
            self.topic, self.qos, self.alive = topic, qos, True
            publishers.append(self)
        def publish(self, message):
            assert self.alive
            payload = json.loads(message.data)
            state = states[payload['observation_id']]
            assert state.get('executor_ready')
            assert self.qos.depth == 1
            assert self.qos.durability == runner.DurabilityPolicy.TRANSIENT_LOCAL
            selected.append(deepcopy(payload))
            scene = state['scene']
            if scene.get('executor_error'):
                send_status(state, 'grasp_executor', 'failed', reason=scene['executor_error'],
                    command_attempted=scene.get('command_attempted', False), object_count=len(payload['objects']),
                    completed_count=0, candidate_attempt_count=1, source_stamp_ns=payload['source_stamp_ns'])
                return
            if scene.get('exhausted'):
                send_status(state, 'grasp_executor', 'candidates_exhausted', reason='all_candidates_failed',
                    command_attempted=False, completed_count=0, object_count=len(payload['objects']),
                    candidate_attempt_count=2, source_stamp_ns=payload['source_stamp_ns'])
                return
            preview_count = scene.get('previews', len(payload['objects']))
            preview_topic = state['namespace'] + '/task_preview'
            if scene.get('intermediate'):
                subscriptions[preview_topic](NS(trajectory=[NS(joint_trajectory=NS(points=[1, 2]))]))
            for _ in range(preview_count):
                subscriptions[preview_topic](NS(trajectory=[NS(joint_trajectory=NS(points=[1, 2]))]
                                                * (7 if single_bowl or clear_table is not None else 6)))
            send_status(state, 'grasp_executor', 'success' if execute else 'planned',
                object_count=len(payload['objects']), completed_count=1 if execute else 0,
                candidate_attempt_count=len(payload['objects'])*2,
                candidate_failures=[{'candidate_id': 0, 'reason': 'test_rejected'}],
                command_attempted=execute, source_stamp_ns=payload['source_stamp_ns'])

    class Node:
        def __init__(self, *args): pass
        def get_node_names_and_namespaces(self): return list(busy)
        def get_clock(self): return NS(now=lambda: NS(nanoseconds=ros_now()))
        def create_subscription(self, kind, topic, callback, qos):
            subscriptions[topic] = callback
            return topic
        def create_publisher(self, kind, topic, qos): return Publisher(topic, qos)
        def destroy_subscription(self, token):
            old_callbacks.append(subscriptions.pop(token))
        def destroy_publisher(self, publisher): publisher.alive = False
        def destroy_node(self): pass

    class Process:
        def __init__(self, command, **kwargs):
            self.pid, self.returncode = 1000 + len(processes), None
            processes.append(self)
            commands.append(command)
            self.command = command
            if command[0].endswith('run_grasp_position_control.sh'):
                assert execute
                self.returncode = 1 if controller_failure else 0
                if controller_failure:
                    kwargs['stdout'].write(controller_failure + '\n')
                    kwargs['stdout'].flush()
                return
            params = dict(arg.split(':=', 1) for arg in command if ':=' in arg)
            observation_id = params['observation_id']
            if observation_id not in states:
                index = int(observation_id.rsplit('_', 1)[-1]) - 1
                states[observation_id] = {**params, 'scene': scenes[min(index, len(scenes)-1)],
                                         'observed': False, 'started': clock[0]}
            state = states[observation_id]
            if command[0].endswith('run_rim_locator.sh'):
                send_status(state, 'rim_locator', 'waiting_input')
            elif command[0].endswith('run_yolo_vision.sh'):
                state['vision_started'] = True
                send_status(state, 'yolo_vision', 'ready')
            elif command[0].endswith('run_grasp_executor.sh'):
                assert state['observed']
                assert int(params['supervisor_pid']) == runner.os.getpid()
                assert int(params['supervisor_start_ticks']) == runner.process_start_ticks(runner.os.getpid())
                assert params['task_preview_topic'] == state['namespace'] + '/task_preview'
                assert int(params['source_not_before_ns']) < ros_now()
                assert int(params['task_deadline_monotonic_ns']) > int(clock[0] * 1e9)
                state['executor_ready'] = True
                send_status(state, 'grasp_executor', 'waiting_target')
        def poll(self): return self.returncode
        def wait(self, timeout):
            if self.returncode is None:
                raise runner.subprocess.TimeoutExpired(self.command, timeout)
            return self.returncode

    def spin(node, timeout_sec):
        clock[0] += max(.05, timeout_sec)
        if v2_images and '/joint_states' in subscriptions:
            home = json.loads((root/'src/grasp_executor/config/place_zones.json').read_text())['return_home']['joint_positions']
            subscriptions['/joint_states'](NS(name=list(home), position=list(home.values()), velocity=[0.]*len(home),
                header=NS(stamp=NS(sec=ros_now()//10**9, nanosec=ros_now()%10**9))))
        for state in list(states.values()):
            if not state.get('vision_started') or state['observed']:
                continue
            scene = state['scene']
            if clock[0] - state['started'] < scene.get('observe_after', .1):
                continue
            state['observed'] = True
            tray_details = {'tray': scene['tray']} if 'tray' in scene else {}
            if stale_callbacks:
                for callback in old_callbacks:
                    callback(NS(data='not valid JSON'))  # Retired closures must do nothing.
            if scene.get('vision_error'):
                send_status(state, 'yolo_vision', 'failed', reason=scene['vision_error'],
                            source_stamp_ns=ros_now(), detected_count=1,
                            object_count=0, **tray_details,
                            rejected=scene.get('vision_rejected', [{'object_id': 0, 'reason': 'insufficient_pointcloud'}]))
                continue
            if scene.get('count') == 0:
                send_status(state, 'yolo_vision', 'no_targets', detected_count=0, object_count=0,
                            source_stamp_ns=ros_now(), **tray_details)
                continue
            batch = batch_for(state)
            state['batch'] = batch
            count = len(batch['objects'])
            extra_rejected = scene.get('vision_rejected',
                [{'object_id': 99, 'reason': 'insufficient_pointcloud'}] if scene.get('rejected') else [])
            stamp = batch['source_stamp_ns']
            send_status(state, 'yolo_vision', 'success', detected_count=count+len(extra_rejected),
                        object_count=count, objects=scene.get('physical', [{'object_id': g['object_id']} for g in batch['objects']]),
                        rejected=extra_rejected, source_stamp_ns=stamp, **tray_details)
            if scene.get('rim_error'):
                rejected = [{'object_id': g['object_id'], 'reason': scene['rim_error']} for g in batch['objects']]
                batch['objects'], batch['rejected'] = [], rejected
                subscriptions[state['namespace']+'/grasp_candidates'](NS(data=json.dumps(batch)))
                send_status(state, 'rim_locator', 'failed', reason='pose_not_found:no_valid_object',
                            rejected=rejected, source_stamp_ns=stamp)
                continue
            for rejected_id in scene.get('rim_rejected_ids', []):
                batch['objects'] = [g for g in batch['objects'] if g['object_id'] != rejected_id]
                batch['rejected'].append({'object_id': rejected_id, 'reason': 'insufficient_points'})
            if scene.get('old_candidate'):
                batch['source_stamp_ns'] = int(state.get('source_not_before_ns', ros_now())) - 1
            if scene.get('foreign_candidate'):
                foreign = deepcopy(batch)
                foreign['observation_id'] = 'foreign/round_0001'
                subscriptions[state['namespace']+'/grasp_candidates'](NS(data=json.dumps(foreign)))
            subscriptions[state['namespace']+'/grasp_candidates'](NS(data=json.dumps(batch)))
            send_status(state, 'rim_locator', 'success', object_count=len(batch['objects']), source_stamp_ns=stamp,
                        rejected=batch['rejected'])

    def kill(pid, sig):
        clock[0] += .001  # Process shutdown takes time before the next source barrier.
        process = next(p for p in processes if p.pid == pid)
        if cleanup_failure and sig == signal.SIGINT:
            return
        process.returncode = 0

    monkeypatch.setattr(runner, 'Node', Node)
    monkeypatch.setattr(runner, 'task_lock_path', lambda: tmp_path / 'task.lock')
    monkeypatch.setattr(runner.rclpy, 'init', lambda **kwargs: None)
    monkeypatch.setattr(runner.rclpy, 'shutdown', lambda: None)
    monkeypatch.setattr(runner.rclpy, 'spin_once', spin)
    monkeypatch.setattr(runner.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(runner.subprocess, 'Popen', Process)
    monkeypatch.setattr(runner.os, 'killpg', kill)
    monkeypatch.setattr(runner, 'process_group_alive',
                        lambda pid: next(p for p in processes if p.pid == pid).returncode is None)
    monkeypatch.setattr(runner.time, 'sleep', lambda seconds: clock.__setitem__(0, clock[0] + seconds))
    output = tmp_path / 'output'
    monkeypatch.setattr(sys, 'argv', ['test', '--output-dir', str(output), '--vision-config',
        runner.os.path.relpath(override, Path.cwd()), '--timeout', '120'] + (['--execute'] if execute else [])
        + zone_args + list(extra))
    code = runner.main()
    result = json.loads((output/'result.json').read_text())
    assert all(p.read_bytes() == data for p, data in originals.items())
    assert all(p.poll() is not None for p in processes)
    assert all(not p.alive for p in publishers)
    return code, result, commands, selected


@pytest.mark.parametrize('zone,index', [('左上', 0), ('左下', 1), ('中间', 2),
                                        ('右上', 3), ('右下', 4), ('blue', 5)])
@pytest.mark.parametrize('execute', [False, True])
def test_single_bowl_chooses_zone_and_stops_after_first_task(monkeypatch, tmp_path, zone, index, execute):
    code, result, commands, selected = run_fake(monkeypatch, tmp_path,
        [{'count': 1}, {'count': 1}], execute, single_zone=zone)
    assert code == 0 and result['passed']
    assert result['target_zone'] == zone and result['selection'] == 'single_bowl'
    assert result['observation_count'] == 1 and len(selected) == 1
    assert result['completed_count'] == (1 if execute else 0)
    assert result['return_home_requested'] and result['returned_home'] is execute
    grasp = json.loads(Path(result['config_snapshots']['grasp']).read_text())
    assert grasp['place_ready_position']['x'] == pytest.approx(.388824983 + index * .01)
    assert grasp['place_drop_distance'] == {'左上':.11,'左下':.1,'中间':.1,'右上':.05,'右下':.12,'blue':.1}[zone]
    assert grasp['return_home_joint_positions'] is not None
    assert all(p['segments'] == 7 for p in result['preview_batches'])
    assert bool([c for c in commands if 'position_control' in c[0]]) is execute


@pytest.mark.parametrize('execute', [False, True])
def test_xyz_mode_runs_single_bowl_with_blue_orientation(monkeypatch, tmp_path, execute):
    coords = (-.30, -.50, .22)
    code, result, commands, selected = run_fake(
        monkeypatch, tmp_path, [{'count': 1}], execute, release_xyz=coords)
    assert code == 0 and result['passed'] and len(selected) == 1
    assert result['target_zone'] is None
    assert result['release_xyz_base_tool0'] == dict(zip('xyz', coords))
    grasp = json.loads(Path(result['config_snapshots']['grasp']).read_text())
    zones = json.loads((Path(runner.__file__).parents[1] / 'src/grasp_executor/config/place_zones.json').read_text())
    distance = zones['zones']['blue']['place_drop_distance']
    assert grasp['place_reference_position'] == pytest.approx({'x':coords[0], 'y':coords[1], 'z':coords[2]+distance})
    assert grasp['place_reference_orientation'] == zones['zones']['blue']['place_reference_orientation']
    assert grasp['place_drop_distance'] == pytest.approx(distance)
    assert bool([c for c in commands if 'position_control' in c[0]]) is execute


@pytest.mark.parametrize('count', [0, 2])
@pytest.mark.parametrize('execute', [False, True])
def test_single_bowl_rejects_empty_or_multiple_before_controller(monkeypatch, tmp_path, count, execute):
    code, result, commands, selected = run_fake(monkeypatch, tmp_path,
        [{'count': count}], execute, single_zone='左上')
    assert code == 1 and not result['passed'] and not selected
    assert result['completed_count'] == 0
    assert result['observation_count'] == (3 if count == 0 else 1)
    assert not any('position_control' in c[0] or 'run_grasp_executor' in c[0] for c in commands)


@pytest.mark.parametrize('empty_rounds', [1, 2])
@pytest.mark.parametrize('execute', [False, True])
def test_empty_observations_then_target_succeed(monkeypatch, tmp_path, empty_rounds, execute):
    scenes = [{'count': 0}] * empty_rounds + [{'count': 1}]
    code, result, commands, selected = run_fake(monkeypatch, tmp_path, scenes, execute,
        single_zone='blue', stale_callbacks=True)
    assert code == 0 and result['passed'] and result['error'] == ''
    assert result['outcome'] == ('completed' if execute else 'planned')
    assert not result['unresolved_rejections']
    assert result['observation_count'] == empty_rounds + 1 and len(selected) == 1
    assert result['completed_count'] == int(execute)
    assert selected[0]['observation_id'].endswith(f'round_{empty_rounds + 1:04d}')
    stamps = [r['statuses']['yolo_vision']['source_stamp_ns'] for r in result['rounds']]
    for earlier, later in zip(stamps, result['rounds'][1:]):
        assert later['source_not_before_ns'] > earlier
    assert sum('position_control' in c[0] for c in commands) == int(execute)


@pytest.mark.parametrize('extra,rounds', [([], 3), (['--max-reobservations', '0'], 1),
                                        (['--max-reobservations', '1'], 2)])
def test_empty_detection_retries_are_bounded(monkeypatch, tmp_path, extra, rounds):
    code, result, commands, selected = run_fake(monkeypatch, tmp_path, [{'count': 0}], True,
        single_zone='blue', extra=extra)
    assert code == 1 and not result['passed'] and not selected
    assert result['observation_count'] == rounds
    assert result['reobservations_without_progress'] == rounds - 1
    assert result['outcome'] == 'no_targets' and result['error'] == 'single_bowl_required'
    assert not result['unresolved_rejections']
    assert not any('position_control' in c[0] or 'run_grasp_executor' in c[0] for c in commands)


def test_empty_retry_keeps_prior_failure_and_shares_budget(monkeypatch, tmp_path):
    code, result, _, _ = run_fake(monkeypatch, tmp_path,
        [{'exhausted': True}, {'count': 0}, {'count': 1}], True, single_zone='blue')
    assert code == 1 and result['completed_count'] == 1 and result['observation_count'] == 3
    assert result['unresolved_rejections'] and result['error'] == 'prior_rejection_unresolved'


def test_empty_and_geometry_failures_cannot_extend_three_round_limit(monkeypatch, tmp_path):
    code, result, _, selected = run_fake(monkeypatch, tmp_path,
        [{'count': 0}, {'exhausted': True}, {'count': 0}, {'count': 1}], True, single_zone='blue')
    assert code == 1 and result['observation_count'] == 3 and result['completed_count'] == 0
    assert len(selected) == 1


@pytest.mark.parametrize('arguments', [[], ['--target-zone', '左上'],
                                         ['--target-zone', '中间'], ['--target-zone', '其他']])
def test_missing_untaught_or_unknown_zone_fails_before_ros(monkeypatch, tmp_path, arguments):
    def forbidden(*args, **kwargs):
        pytest.fail('ROS or a process started before placement validation')
    monkeypatch.setattr(runner.rclpy, 'init', forbidden)
    monkeypatch.setattr(runner.subprocess, 'Popen', forbidden)
    output = tmp_path / 'out'
    config = json.loads((Path(runner.__file__).parents[1] / 'src/grasp_executor/config/place_zones.json').read_text())
    for zone in config['zones'].values():
        zone['enabled'] = False
    path = tmp_path / 'disabled.json'
    path.write_text(json.dumps(config))
    monkeypatch.setattr(sys, 'argv', ['test', '--output-dir', str(output),
                                     '--place-zones-config', str(path), *arguments])
    with pytest.raises(SystemExit) as error:
        runner.main()
    assert error.value.code == 2 and not output.exists()


@pytest.mark.parametrize('arguments', [
    ['--x', '-0.3'],
    ['--x', '-0.3', '--y', '-0.5'],
    ['--x', '-0.3', '--y', '-0.5', '--z', 'nan'],
    ['--target-zone', 'blue', '--x', '-0.3', '--y', '-0.5', '--z', '0.22'],
])
def test_invalid_xyz_cli_fails_before_ros(monkeypatch, tmp_path, arguments):
    monkeypatch.setattr(runner.rclpy, 'init', lambda *a, **k: pytest.fail('ROS started'))
    monkeypatch.setattr(runner.subprocess, 'Popen', lambda *a, **k: pytest.fail('process started'))
    output = tmp_path / 'out'
    monkeypatch.setattr(sys, 'argv', ['test', '--output-dir', str(output), *arguments])
    with pytest.raises(SystemExit) as error:
        runner.main()
    assert error.value.code == 2 and not output.exists()


@pytest.mark.parametrize('count', [0, 1, 2, 3])
@pytest.mark.parametrize('execute', [False, True])
@pytest.mark.parametrize('new_run', [False, True])
def test_observed_count_drives_fresh_rounds(monkeypatch, tmp_path, count, execute, new_run):
    scenes = ([{'count': n} for n in range(count, -1, -1)] if execute else [{'count': count}])
    code, result, commands, selected = run_fake(monkeypatch, tmp_path, scenes, execute, new_run=new_run)
    assert code == 0 and result['passed']
    assert result['completed_count'] == (count if execute else 0)
    assert result['initial_detected_count'] == count
    assert len(result['preview_batches']) == count
    assert result['observation_count'] == (count+1 if execute else 1)
    assert len(selected) == (count if execute else bool(count))
    if execute:
        assert all(len(batch['objects']) == 1 for batch in selected)
    else:
        assert not any('position_control' in command[0] for command in commands)
    for index, current in enumerate(result['rounds'], 1):
        assert f'round_{index:04d}' in current['namespace']
        assert current['source_not_before_ns'] > 0
        configs = current['config_snapshots']
        g = json.loads(Path(configs['grasp']).read_text())
        assert g['input_mode'] == 'candidate_groups' and g['approach_height'] == .10
        assert g['plan_only'] is (not execute)
        assert g['input_topic'].startswith(current['namespace'])
        assert g['task_preview_topic'].startswith(current['namespace'])


def test_next_object_uses_new_position_and_reassigned_id(monkeypatch, tmp_path):
    scenes = [{'positions': [[.3,.1,.2], [.5,.1,.2]], 'ids': [4, 9]},
              {'positions': [[.62,.05,.21]], 'ids': [0]}, {'count': 0}]
    code, result, _, selected = run_fake(monkeypatch, tmp_path, scenes, True, stale_callbacks=True)
    assert code == 0 and result['completed_count'] == 2
    assert selected[0]['objects'][0]['object_id'] == 4
    assert selected[1]['objects'][0]['object_id'] == 0
    assert selected[1]['objects'][0]['candidates'][0]['position'] == [.62,.05,.21]
    assert selected[0]['source_stamp_ns'] < selected[1]['source_stamp_ns']
    assert selected[0]['observation_id'] != selected[1]['observation_id']


def test_exhausted_candidates_have_bounded_reobservation(monkeypatch, tmp_path):
    code, result, _, selected = run_fake(monkeypatch, tmp_path, [{'exhausted': True}], True)
    assert code == 1 and len(selected) == result['observation_count'] == 3
    assert result['completed_count'] == 0 and result['error'] == 'reobservation_budget_exhausted'
    assert result['candidate_attempt_count'] == 6


@pytest.mark.parametrize('scene', [
    {'vision_error': 'no_valid_depth_for_detected_targets'},
    {'rim_error': 'pose_not_found:insufficient_wall_points'}])
def test_geometry_failure_can_reobserve_and_then_complete(monkeypatch, tmp_path, scene):
    code, result, _, _ = run_fake(monkeypatch, tmp_path, [scene, {'count': 1}, {'count': 0}], True)
    assert code == 1 and result['completed_count'] == 1 and result['observation_count'] == 3
    assert result['reobservations_without_progress'] == 0
    assert result['outcome'] == 'partial' and result['error'] == 'remaining_unknown'


@pytest.mark.parametrize('scene', [
    {'vision_error': 'input_timeout'}, {'rim_error': 'invalid_transform'},
    {'executor_error': 'controller_not_ready'},
    {'executor_error': 'motion_timeout', 'command_attempted': True}])
def test_nonrecoverable_errors_never_start_second_round(monkeypatch, tmp_path, scene):
    code, result, _, _ = run_fake(monkeypatch, tmp_path, [scene, {'count': 0}], True)
    assert code == 1 and result['observation_count'] == 1 and result['completed_count'] == 0


def test_failed_target_disappearing_is_remaining_unknown(monkeypatch, tmp_path):
    code, result, _, _ = run_fake(monkeypatch, tmp_path, [{'exhausted': True}, {'count': 0}], True)
    assert code == 1 and result['error'] == 'remaining_unknown'


def test_prior_unresolved_geometry_is_not_erased_by_another_success(monkeypatch, tmp_path):
    scenes = [{'count': 1, 'rejected': True}, {'count': 1}, {'count': 0}]
    code, result, _, _ = run_fake(monkeypatch, tmp_path, scenes, True)
    assert code == 1 and result['outcome'] == 'partial' and result['completed_count'] == 2
    assert result['error'] == 'remaining_unknown'


def test_preview_does_not_loop_and_reports_rejected_targets(monkeypatch, tmp_path):
    code, result, _, _ = run_fake(monkeypatch, tmp_path, [{'count': 2, 'rejected': True}])
    assert code == 1 and result['outcome'] == 'partial' and result['observation_count'] == 1
    assert result['completed_count'] == 0


def test_missing_complete_preview_is_failure(monkeypatch, tmp_path):
    code, result, _, _ = run_fake(monkeypatch, tmp_path, [{'count': 2, 'previews': 1, 'intermediate': True}])
    assert code == 1 and result['error'] == 'preview_batch_mismatch'


def test_foreign_candidate_and_retired_callbacks_cannot_advance_round(monkeypatch, tmp_path):
    code, result, _, _ = run_fake(monkeypatch, tmp_path,
        [{'count': 1, 'foreign_candidate': True}, {'count': 0}], True, stale_callbacks=True)
    assert code == 0 and result['completed_count'] == 1


def test_old_candidate_is_rejected_before_executor(monkeypatch, tmp_path):
    code, result, commands, _ = run_fake(monkeypatch, tmp_path, [{'old_candidate': True}], True)
    assert code == 1 and not any('run_grasp_executor' in c[0] for c in commands)


@pytest.mark.parametrize('limit,expected', [('--max-completed-objects','max_completed_objects_reached'),
                                          ('--max-observation-rounds','max_observation_rounds_reached')])
def test_total_limits_do_not_reset_on_success(monkeypatch, tmp_path, limit, expected):
    code, result, _, _ = run_fake(monkeypatch, tmp_path, [{'count': 1}], True, extra=[limit, '1'])
    assert code == 1 and result['completed_count'] == 1 and result['error'] == expected


def test_zero_reobservations_and_global_deadline(monkeypatch, tmp_path):
    code, result, _, _ = run_fake(monkeypatch, tmp_path, [{'exhausted': True}], True,
                                extra=['--max-reobservations','0'])
    assert code == 1 and result['observation_count'] == 1


def test_no_frames_times_out_without_controller(monkeypatch, tmp_path):
    code, result, commands, _ = run_fake(monkeypatch, tmp_path, [{'observe_after': 1000}], True,
                                       extra=['--timeout', '3'])
    assert code == 1 and result['error'] == 'pipeline_timeout'
    assert not any('position_control' in c[0] for c in commands)


def test_cleanup_failure_prevents_next_observation(monkeypatch, tmp_path):
    code, result, _, _ = run_fake(monkeypatch, tmp_path, [{'count': 1}], True, cleanup_failure=True)
    assert code == 1 and result['observation_count'] == 1 and result['completed_count'] == 1
    assert 'forced_shutdown' in result['error']


@pytest.mark.parametrize('fault', ['single','multi','string_bool','unknown','bad_json','missing','timeout','limit'])
def test_invalid_config_fails_before_ros_or_processes(monkeypatch, tmp_path, fault):
    def forbidden(*args, **kwargs): pytest.fail('invalid input must fail before ROS or processes')
    monkeypatch.setattr(runner.rclpy, 'init', forbidden)
    monkeypatch.setattr(runner.subprocess, 'Popen', forbidden)
    config = tmp_path/'override.json'
    if fault == 'string_bool': config.write_text('{"multi_instance":"false"}')
    if fault == 'unknown': config.write_text('{"unknown":true}')
    if fault == 'bad_json': config.write_text('{')
    args = ['test', '--output-dir', str(tmp_path/'out'), '--execute', '--legacy-multi']
    args += (['--'+fault] if fault in ('single','multi') else ['--timeout','nan'] if fault == 'timeout'
             else ['--max-reobservations','-1'] if fault == 'limit' else ['--vision-config',str(config)])
    monkeypatch.setattr(sys, 'argv', args)
    with pytest.raises(SystemExit) as error: runner.main()
    assert error.value.code == 2 and not (tmp_path/'out').exists()


@pytest.mark.parametrize('symlink', [False, True])
def test_snapshot_cannot_overwrite_source_config(monkeypatch, tmp_path, symlink):
    output = tmp_path/'out'; (output/'round_0001').mkdir(parents=True)
    snapshot = output/'round_0001/effective_vision_config.json'
    config = tmp_path/'source.json' if symlink else snapshot
    original = '{"multi_instance":false}'; config.write_text(original)
    if symlink: snapshot.symlink_to(config)
    monkeypatch.setattr(sys, 'argv', ['test','--legacy-multi','--output-dir',str(output),
                                     '--vision-config',str(config)])
    with pytest.raises(SystemExit) as error: runner.main()
    assert error.value.code == 2 and config.read_text() == original
    assert not (output/'round_0001/effective_rim_config.json').exists()


def test_unassociated_success_does_not_erase_prior_exhausted_object(monkeypatch, tmp_path):
    scenes = [{'exhausted': True, 'ids': [4]}, {'count': 1, 'ids': [0]}, {'count': 0}]
    code, result, _, _ = run_fake(monkeypatch, tmp_path, scenes, True)
    assert code == 1 and result['completed_count'] == 1
    assert result['outcome'] == 'partial' and result['error'] == 'remaining_unknown'
    assert result['reobservations_without_progress'] == 0


@pytest.mark.parametrize('cloud_arrives', [True, False])
def test_rejected_round_waits_briefly_for_cloud_even_when_batch_already_arrived(monkeypatch, cloud_arrives):
    clock, started = [0.], []
    monkeypatch.setattr(runner.time, 'monotonic', lambda: clock[0])
    state = NS(statuses={'rim_locator': {'status': 'failed',
               'reason': 'pose_not_found:no_valid_object',
               'rejected': [{'object_id': 0, 'reason': 'pose_not_found:circle_insufficient_coverage'}]}},
               batch={'objects': []}, configs={'rim': NS(publish_debug=True)},
               result={}, deadline=10., attach=lambda: None,
               start=lambda name: started.append(name), wait_ready=lambda *args: None)
    def spin():
        clock[0] += .25
        if cloud_arrives and clock[0] >= .5:
            state.result['base_cloud'] = 'captured.npz'
    state.spin = spin
    assert runner.ObservationRound.run(state, False) is False
    assert started == ['rim_locator', 'yolo_vision']
    if cloud_arrives:
        assert state.result['base_cloud'] == 'captured.npz'
        assert clock[0] == .5
    else:
        assert 'base_cloud_delivery_timeout' in state.result['debug_capture_errors']
        assert clock[0] <= 1.5


def test_controller_failure_surfaces_cause_without_starting_executor(monkeypatch, tmp_path, capsys):
    cause = '错误：External Control 未运行；请在示教器启动对应程序'
    code, result, commands, selected = run_fake(monkeypatch, tmp_path, [{'count': 1}],
        execute=True, single_zone='左下', controller_failure=cause)
    assert code == 1 and result['passed'] is False
    assert result['error'] == 'controller_preparation_failed'
    assert result['controller_error_detail'] == cause
    assert result['controller_exit_code'] == 1
    assert Path(result['controller_preparation_log']).read_text().strip() == cause
    assert result['rounds'][0]['controller_error_detail'] == cause
    assert result['completed_count'] == 0 and result['candidate_attempt_count'] == 0
    assert not any('run_grasp_executor.sh' in c[0] for c in commands)
    output = capsys.readouterr().out
    assert cause in output and '任务失败：controller_preparation_failed' in output
    assert '控制器准备日志：' in output
