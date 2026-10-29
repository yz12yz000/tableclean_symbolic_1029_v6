#!/usr/bin/env python3
"""Synthetic clear-table observations using the live policy and grasp engine.

No ROS imports, driver processes, controller calls, network or device access.
All observations and actuator acknowledgements in this module are simulated.
"""

from argparse import ArgumentParser
from dataclasses import asdict, replace
import json
import math
from pathlib import Path
import sys
import tempfile
import time

WORKSPACE = Path(__file__).resolve().parents[1]
for package in ('grasp_executor', 'rim_locator', 'yolo_vision'):
    sys.path.insert(0, str(WORKSPACE / 'src' / package))

from grasp_executor.candidates import run_candidate_group_task
from grasp_executor.config import load_config


class SimulatedInterfaces:
    """Record the existing execution state machine; never create a ROS adapter."""

    def __init__(self):
        self.events = []
        self.motion_count = 0
        self.gripper_count = 0

    def _record(self, event, **fields):
        self.events.append(dict(event=event, **fields))
        return True

    def begin_batch(self):
        self._record('begin_batch')

    def end_batch(self):
        self._record('end_batch')

    def check_ready(self, timeout):
        return self._record('simulated_ready')

    def resolve_place_pose(self):
        return None

    def discard_prepared_plans(self):
        self._record('discard_plans')

    def prepare_plans(self, *poses, deadline=None):
        return self._record('simulate_plans', poses=[asdict(pose) for pose in poses])

    def move_to_pose(self, pose, timeout):
        self.motion_count += 1
        return self._record('simulate_move_to_pose', pose=asdict(pose))

    def move_linear(self, pose, timeout):
        self.motion_count += 1
        return self._record('simulate_move_linear', pose=asdict(pose))

    def is_pose_reached(self, timeout):
        return True

    def open_gripper(self, timeout):
        self.gripper_count += 1
        return self._record('simulate_open')

    def close_gripper(self, timeout):
        self.gripper_count += 1
        return self._record('simulate_close')

    def finish(self):
        self._record('finish')

    def stop_motion(self, timeout):
        return self._record('simulate_stop')


def write_json(path, value):
    with path.open('x', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')


def parse_args(argv=None):
    parser = ArgumentParser(description='收桌面纯软件演示：合成观察 + 正式任务策略 + 模拟抓放接口；不连接硬件。')
    parser.add_argument('--scenario', choices=('normal', 'visual', 'tray_full', 'missing', 'unverified'), default='normal')
    parser.add_argument('--output-dir', type=Path, help='新的输出目录（拒绝覆盖已有目录）')
    parser.add_argument('--task-policy', type=Path, help='可选的 preview 模式 Agent 策略 JSON')
    return parser.parse_args(argv)


def synthetic_config():
    """Deliberately synthetic dimensions/positions, separate from field config."""
    from yolo_vision.tray_utils import validate_tray_config
    config = dict(schema_version=1, calibrated=True, motion_approved=True,
                  calibration_note='SOFTWARE SIMULATION ONLY; these are invented fixture coordinates, not field calibration.',
                  frame_id='base', detector='yolo',
                  yolo=dict(model_path='models/tray-seg.pt', target_class='tray',
                            confidence=.5, imgsz=640, device='cpu'),
                  tray_outer_size=[.64, .48],
                  tray_size=[.60, .44], bowl_radius=.06, edge_margin=.02,
                  clearance=.025, position_tolerance=.015,
                  release_z=None, release_xy_offset=[0., 0.], empty_confirmations=2,
                  source_polygon=[[-1., -1.], [1., -1.], [1., 1.], [-1., 1.]],
                  slots=[dict(id=slot_id, uv=[u, v]) for slot_id, u, v in
                         [('A', -.16, -.10), ('B', .16, -.10),
                          ('C', -.16, .10), ('D', .16, .10)]])
    return validate_tray_config(config, require_calibrated=True)


class SyntheticTable:
    def __init__(self, config, scenario):
        self.config, self.scenario = config, scenario
        self.sources = [dict(key=i, xy=[x, .30]) for i, x in enumerate((-.25, -.50, -.75))]
        self.inside = [] if scenario == 'visual' else [dict(id=config['slots'][0]['id'], uv=list(config['slots'][0]['uv']))]
        if scenario == 'visual':
            for item, radius in zip(self.sources, (.05, .075, .06)):
                item['radius'] = radius
        if scenario == 'tray_full':
            self.inside = [dict(id=slot['id'], uv=list(slot['uv'])) for slot in config['slots']]
        self.motion_rounds = 0
        self.ids_to_source = {}

    def observation(self, round_index, task_id):
        from yolo_vision.tray_utils import slot_xy
        tray = dict(status='ok', frame_id='base',
                    center_xy=[-.45 + .015 * self.motion_rounds, -.40 - .010 * self.motion_rounds],
                    yaw=.02 * self.motion_rounds, size=list(self.config['tray_size']),
                    identity='yolo:' + self.config['yolo']['target_class'])
        objects = []
        self.ids_to_source = {}
        for item in self.sources:
            object_id = round_index * 100 + len(objects)
            self.ids_to_source[object_id] = item['key']
            objects.append(dict(object_id=object_id, position_valid=True,
                                frame_id='base', center_xy=list(item['xy']), radius=item.get('radius', .06)))
        for item in self.inside:
            objects.append(dict(object_id=round_index * 100 + len(objects), position_valid=True,
                                frame_id='base', center_xy=slot_xy(tray, item, self.config),
                                radius=item.get('radius', .06)))
        stamp = round_index * 1_000_000_000
        observation_id = task_id + '/round_' + str(round_index)
        vision = dict(status='success' if objects else 'no_targets', task_id=task_id,
                      observation_id=observation_id, source_stamp_ns=stamp,
                      object_count=len(objects), detected_count=len(objects), objects=objects, rejected=[], tray=tray)
        if self.scenario == 'missing':
            vision['tray'] = dict(status='not_found', frame_id='base', reason='synthetic_tray_not_visible')
        groups = []
        for obj in objects:
            if self.scenario == 'visual':
                obj.update(position_reason='visual_rim_circle', center_z=.20,
                           estimated_radius=obj['radius']-.004, size_uncertainty=.004,
                           observed_rim_radius=obj['radius']-.004)
            # Existing coordinate mode uses base/tool0 for release; candidates
            # use base_link/grasp_center. UR base_link -> base is a yaw of pi.
            x, y = (-value for value in obj['center_xy'])
            if self.scenario == 'visual':
                x -= obj['observed_rim_radius']
            groups.append(dict(object_id=obj['object_id'], distance=math.hypot(x, y),
                               candidates=[dict(candidate_id=0, position=[x, y, .20],
                                                orientation=[0., 1., 0., 0.],
                                                geometry=dict(wall_count=30, wall_residual=.001))]))
            if self.scenario == 'visual':
                groups[-1]['candidates'].append(dict(candidate_id=1,
                    position=[-obj['center_xy'][0],y-obj['observed_rim_radius'],.20],
                    orientation=[math.sqrt(.5),math.sqrt(.5),0.,0.],geometry=dict(wall_count=30)))
        batch = dict(schema_version=1, task_id=task_id, observation_id=observation_id,
                     source_stamp_ns=stamp, frame_id='base_link', tool_frame='grasp_center',
                     objects=groups, rejected=[])
        return vision, batch

    def simulate_placement(self, decision):
        if self.scenario == 'unverified':
            return
        source_key = self.ids_to_source[decision['source']['object_id']]
        self.sources = [item for item in self.sources if item['key'] != source_key]
        self.inside.append(dict(id=decision['slot']['id'], uv=list(decision['slot']['uv']),
                                radius=decision['source']['radius']))
        # Tray and its contents move together only after the simulated arm has
        # returned. The following observation must compute the new world XY.
        self.motion_rounds += 1


def run_demo(output_dir, scenario, task_policy=None):
    from place_rounds import (ClearTableSession, RoundBudget, RoundLimits, load_place_zone,
                             tray_release_config, visual_tray_grasp_config)
    from table_runtime_metadata import build_preflight

    tray_config = synthetic_config()
    if scenario == 'visual':
        from yolo_vision.tray_utils import validate_tray_config
        tray_config.update(object_geometry='visual_rim',placement_mode='grid',slots=[],
                           bowl_radius=.04,compensate_bowl_xy=True,placement_confirmations=2,
                           placement_match_tolerance=.05)
        tray_config = validate_tray_config(tray_config)
    session = ClearTableSession(tray_config, policy=task_policy)
    table = SyntheticTable(tray_config, scenario)
    limits = RoundLimits(
        max_reobservations=task_policy['limits'].get('max_reobservations', 2) if task_policy else 2,
        max_observation_rounds=task_policy['limits'].get('max_observation_rounds', 12) if task_policy else 12,
        max_completed_objects=task_policy['limits'].get('max_completed_objects', 20) if task_policy else 20)
    budget = RoundBudget(time.monotonic() + (task_policy['limits']['deadline_s'] if task_policy else 30.), limits)
    task_id = 'offline/' + output_dir.name
    write_json(output_dir / 'synthetic_tray_config.json', tray_config)
    if task_policy is not None:
        write_json(output_dir / 'task_policy.json', task_policy)
    grasp_template = load_config(WORKSPACE / 'src/grasp_executor/config/place_config.json')
    zones_path = WORKSPACE / 'src/grasp_executor/config/place_zones.json'
    grasp_template = load_place_zone(zones_path, 'blue', grasp_template)
    preflight = build_preflight(root=WORKSPACE, grasp=grasp_template,
                                tray=tray_config, policy=task_policy)
    rounds = []
    success, status, reason = False, 'failed', 'max_observation_rounds_reached'
    while True:
        try:
            round_index = budget.begin_round(time.monotonic())
        except (RuntimeError, TimeoutError) as exc:
            reason = str(exc)
            break
        round_dir = output_dir / ('round_%04d' % round_index)
        round_dir.mkdir()
        vision, batch = table.observation(round_index, task_id)
        write_json(round_dir / 'synthetic_observation.json', vision)
        write_json(round_dir / 'candidate_groups.json', batch)
        before_verified = session.verified_placed_count
        decision = session.observe(vision, batch)
        if session.verified_placed_count > before_verified:
            budget.complete_object()
        write_json(round_dir / 'decision.json', decision)
        round_result = dict(round_index=round_index, decision=decision,
                            simulated=True, motion_count=0, gripper_count=0)
        rounds.append(round_result)
        print('模拟第 %d 轮：%s；归位确认 %d；%s' %
              (round_index, decision['kind'], session.verified_placed_count, decision.get('reason', '')),
              flush=True)
        if decision['kind'] == 'complete':
            success, status, reason = True, 'completed', ''
        elif decision['kind'] == 'stop':
            reason = decision.get('reason', 'stopped')
        elif decision['kind'] == 'observe_again':
            if (decision.get('reason') in ('confirming_source_empty','confirming_placement')
                    or budget.retry_observation(time.monotonic(), record_failure=False)):
                write_json(round_dir / 'result.json', round_result)
                continue
            reason = decision.get('reason', 'observation_unresolved')
        elif decision['kind'] == 'ready':
            if scenario == 'visual':
                configured = visual_tray_grasp_config(decision,grasp_template,tray_config)
                coordinates = None  # Candidate-specific poses are in simulated_events.
            else:
                configured = tray_release_config(decision['slot'], tray_config, grasp_template)
                coordinates = [configured.place_reference_position['x'], configured.place_reference_position['y'],
                               configured.place_reference_position['z'] - configured.place_drop_distance]
            configured = replace(configured, plan_only=False, place_dwell_time=.001,
                                 execution_journal=str(round_dir / 'simulated_claims'))
            write_json(round_dir / 'effective_grasp_config.json', asdict(configured))
            interfaces = SimulatedInterfaces()
            states = []
            execution = run_candidate_group_task(decision['selected'],
                lambda: vision['source_stamp_ns'] + 1_000_000, interfaces, configured,
                on_state=lambda state: states.append(state), expected_task_id=task_id,
                expected_observation_id=vision['observation_id'])
            round_result.update(execution=asdict(execution), states=states,
                                motion_count=interfaces.motion_count, gripper_count=interfaces.gripper_count,
                                simulated_events=interfaces.events, release_coordinates=coordinates)
            if execution.status == 'success':
                session.record_motion(decision)
                table.simulate_placement(decision)
                write_json(round_dir / 'result.json', round_result)
                continue
            reason = execution.reason or execution.status
        else:
            raise ValueError('unknown_session_decision:' + str(decision['kind']))
        write_json(round_dir / 'result.json', round_result)
        break
    if not success and session.motion_completed_count:
        status = 'partial'
    result = dict(session.summary(), success=success, status=status, reason=reason,
                  mode='offline_demo', simulated=True, hardware_connected=False,
                  scenario=scenario, task_id=task_id,
                  plan_id=task_policy.get('plan_id') if task_policy else None,
                  task_policy=task_policy,
                  observation_count=len(rounds),
                  initial_source_count=3, initial_occupied_count=(0 if scenario=='visual' else 4 if scenario=='tray_full' else 1),
                  motion_completed_count=session.motion_completed_count,
                  verified_placed_count=session.verified_placed_count,
                  simulated_motion_count=sum(item['motion_count'] for item in rounds),
                  simulated_gripper_count=sum(item['gripper_count'] for item in rounds),
                  preflight=preflight,
                  runner_protocol_version=preflight['runner_protocol_version'],
                  runner_revision=preflight['runner_revision'],
                  workspace_revision=preflight['workspace_revision'],
                  config_digest=preflight['config_digest'],
                  model_digest=preflight['model_digest'],
                  rounds=rounds)
    write_json(output_dir / 'result.json', result)
    print('纯软件演示：%s；动作完成 %d，观察确认 %d；结果 %s' %
          (status, session.motion_completed_count, session.verified_placed_count, output_dir / 'result.json'))
    return 0 if success else 1


def main(argv=None):
    args = parse_args(argv)
    task_policy = None
    if args.task_policy is not None:
        from table_agent_protocol import WORKSPACE_REVISION, validate_task_policy
        try:
            with args.task_policy.expanduser().resolve().open(encoding='utf-8') as stream:
                task_policy = validate_task_policy(
                    json.load(stream), expected_mode='preview',
                    require_deployment_metadata=True,
                    expected_workspace_revision=WORKSPACE_REVISION)
        except (OSError, TypeError, ValueError, json.JSONDecodeError) as exc:
            print('策略文件无效：' + str(exc), file=sys.stderr)
            return 2
    if args.output_dir is None:
        base = WORKSPACE / 'debug_output/clear_table_offline'
        base.mkdir(parents=True, exist_ok=True)
        output_dir = Path(tempfile.mkdtemp(prefix='demo_', dir=base))
    else:
        output_dir = args.output_dir.expanduser().resolve()
        try:
            output_dir.mkdir(parents=True, exist_ok=False)
        except FileExistsError:
            print('错误：输出目录已存在，请指定新目录：' + str(output_dir), file=sys.stderr)
            return 2
    print('纯软件演示；没有相机、机械臂、ROS 或硬件指令。输出目录：' + str(output_dir), flush=True)
    try:
        return run_demo(output_dir, args.scenario, task_policy)
    except Exception as exc:
        print('离线演示失败：' + str(exc), file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
