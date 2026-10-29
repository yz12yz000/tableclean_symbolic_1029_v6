"""Cross-boundary v2 tests: image integrity, local candidate retention, exact references."""
from copy import deepcopy
import json
from pathlib import Path
import sys
import uuid

import pytest
from clear_table_offline import SyntheticTable, synthetic_config
from place_rounds import ClearTableSession
from table_round_io import publish_request, atomic_json
from table_round_protocol import validate_request, validate_decision, validate_bundle, request_revision

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'gat_local_new/tools'))
import table_session


def bundle(tmp_path, *, completion=False):
    import cv2
    import numpy as np
    plan_id = str(uuid.uuid4())
    scene = dict(plan_id=plan_id, task_id='test', observation_id='test/round_0001')
    atomic_json(tmp_path/'scene.json', scene)
    cv2.imwrite(str(tmp_path/'camera_rgb.jpg'), np.zeros((200, 200, 3), dtype=np.uint8))
    vision = dict(rgbd_pair=dict(rgb_stamp_ns=11, depth_stamp_ns=12), objects=[],
                  tray=dict(base_xy_to_pixel=[[100, 0, 0], [0, 100, 0], [0, 0, 1]],
                            tray_polygon_px=[[0, 0], [100, 0], [100, 100], [0, 100]]))
    options = [] if completion else [dict(object_id=3, candidate_id=i, slot_id='a', slot=dict(xy=[.5, .5])) for i in (0, 1)]
    r = publish_request(tmp_path, vision, scene, options, {}, True)
    d = dict(schema_version=2, **{k:r[k] for k in ('plan_id','task_id','observation_id','scene_revision')},
             action='finish' if completion else 'select')
    if not completion:
        d['selection'] = dict(object_id=3, slot_id='a')
    return r, d


def test_same_protocol_on_both_sides():
    assert (ROOT/'tools/table_round_protocol.py').read_bytes() == (ROOT/'gat_local_new/generated/table/table_round_protocol.py').read_bytes()


def test_images_bound_to_request_and_atomic_publication(tmp_path):
    r, d = bundle(tmp_path)
    assert validate_bundle(tmp_path, r) == r
    assert validate_decision(d, r) == d
    with pytest.raises(FileExistsError):
        atomic_json(tmp_path/'agent_request.json', r)
    (tmp_path/'camera_rgb.jpg').write_bytes(b'changed')
    with pytest.raises(ValueError, match='bundle_digest'):
        validate_bundle(tmp_path, r)


@pytest.mark.parametrize('change', ['candidate_id', 'wrong_object', 'wrong_slot', 'old_revision', 'finish', 'geometry', 'old_version'])
def test_reject_invalid_selections(tmp_path, change):
    r, d = bundle(tmp_path)
    if change == 'candidate_id': d['selection']['candidate_id'] = 0
    if change == 'wrong_object': d['selection']['object_id'] = 4
    if change == 'wrong_slot': d['selection']['slot_id'] = 'b'
    if change == 'old_revision': d['scene_revision'] = 'old'
    if change == 'finish': d['action'] = 'finish'
    if change == 'geometry': d['xyz'] = [0, 0, 0]
    if change == 'old_version': d['schema_version'] = 1
    with pytest.raises(ValueError): validate_decision(d, r)


def test_completion_review_only_after_local_confirmation(tmp_path):
    r, d = bundle(tmp_path, completion=True)
    validate_decision(d, r)
    d['action'] = 'select'
    d['selection'] = dict(object_id=3, slot_id='a')
    with pytest.raises(ValueError): validate_decision(d, r)
    d.pop('selection')
    for action in ('stop', 'reobserve'):
        d['action'] = action
        validate_decision(d, r)


def test_all_objects_offered_and_all_usable_candidates_preserved():
    config = synthetic_config()
    session = ClearTableSession(config, agent_mode=True)
    vision, batch = SyntheticTable(config, 'normal').observation(1, 'test')
    # Two local contacts per object; selecting the destination must not freeze contact 0.
    for group in batch['objects']:
        group['candidates'].append(dict(deepcopy(group['candidates'][0]), candidate_id=1))
    session.observe(vision, batch)
    options = session.list_options()
    assert len({o['object_id'] for o in options}) == 3
    for oid in {o['object_id'] for o in options}:
        assert len({o['slot_id'] for o in options if o['object_id'] == oid}) <= 3
    option = options[-1]
    selected = session.apply_object_slot({k:option[k] for k in ('object_id','slot_id')})
    assert len(selected['selected']['objects']) == 1
    assert len([c for c in selected['selected']['objects'][0]['candidates'] if not c.get('disabled')]) == 2
    with pytest.raises(ValueError):
        session.apply_object_slot({k:option[k] for k in ('object_id','slot_id')})
    new, batch2 = SyntheticTable(config, 'normal').observation(2, 'test')
    session.observe(new, batch2)
    assert session.current_scene_revision() != selected['scene_revision']


def test_session_atomic_submit_duplicate_conflict_and_consumed(monkeypatch, tmp_path):
    source = tmp_path/'bundle'
    source.mkdir()
    r, d = bundle(source)
    monkeypatch.setattr(table_session, 'ROOT', tmp_path/'client')
    monkeypatch.setattr(table_session, 'WORKSPACE', tmp_path/'robot')
    directory = table_session.directory(r['plan_id'])
    status = dict(state='waiting_agent', **{k:r[k] for k in ('plan_id','task_id','observation_id','scene_revision')}, round='round_0001')
    table_session.atomic(directory/'agent_status.json', status)
    monkeypatch.setattr(table_session, 'poll', lambda _:dict(status, request=str(source/'agent_request.json')))
    path = tmp_path/'decision.json'
    path.write_text(json.dumps(d))
    assert table_session.submit(r['plan_id'], path)['state'] == 'submitted'
    assert table_session.submit(r['plan_id'], path)['duplicate']
    d['reason'] = 'changed'
    path.write_text(json.dumps(d))
    with pytest.raises(ValueError, match='conflicting_decision'):
        table_session.submit(r['plan_id'], path)
    status['state'] = 'running'
    with pytest.raises(ValueError, match='consumed'):
        table_session.submit(r['plan_id'], path)


def test_partial_decision_is_not_accepted(monkeypatch, tmp_path):
    path = tmp_path/'partial.json'
    path.write_text('{"schema_version":')
    with pytest.raises(ValueError): table_session.read(path)


def test_request_digest_covers_choices_and_safe_filenames(tmp_path):
    r, _ = bundle(tmp_path)
    r['choices'][0]['slots'][0]['slot_id'] = 'changed'
    with pytest.raises(ValueError, match='request_revision'): validate_request(r)
    r['files'][0]['file'] = '../elsewhere'
    r['scene_revision'] = request_revision(r)
    with pytest.raises(ValueError, match='request_files'): validate_request(r)


def live_policy(tmp_path):
    from table_agent_protocol import policy_hash
    policy = json.loads((ROOT/'gat_local_new/generated/table/task_policy.json').read_text())
    policy.update(plan_id=str(uuid.uuid4()), mode='preview')
    policy['model']['plan_hash'] = policy_hash(policy)
    path = tmp_path/'policy.json'
    path.write_text(json.dumps(policy))
    return path


def agent_reply(monkeypatch, action=None):
    import test_live_place_pipeline as runner
    original = runner.publish_request
    seen = []
    def publish(directory, *args, **kwargs):
        r = original(directory, *args, **kwargs)
        seen.append(r)
        d = dict(schema_version=2, **{k:r[k] for k in ('plan_id','task_id','observation_id','scene_revision')},
                 action=action or ('select' if r['choices'] else 'finish'))
        if d['action'] == 'select':
            c = r['choices'][0]
            d['selection'] = dict(object_id=c['object_id'], slot_id=c['slots'][0]['slot_id'])
        atomic_json(directory.parent.parent/'decisions'/(directory.name+'.json'), d)
        return r
    monkeypatch.setattr(runner, 'publish_request', publish)
    return seen


def test_real_runner_v2_preview_consumes_visual_bundle_without_motion(monkeypatch, tmp_path):
    from test_place_runner import run_fake
    from test_clear_table_runner import successful_scenes
    config = synthetic_config()
    seen = agent_reply(monkeypatch)
    code, result, commands, selected = run_fake(monkeypatch, tmp_path, successful_scenes(config),
        clear_table=config, v2_images=True, extra=['--task-policy', str(live_policy(tmp_path)),
        '--agent-decision-dir', str(tmp_path/'decisions'), '--agent-round-protocol', 'v2'])
    assert code == 0, result
    assert result['outcome'] == 'planned'
    assert len(seen) == len(selected) == 1
    assert len(seen[0]['choices']) == 3
    assert len([c for c in selected[0]['objects'][0]['candidates'] if not c.get('disabled')]) == 2
    assert not any('--execute' in command for command in commands)
    assert result['hardware_motion_requested'] is False
    assert (tmp_path/'output/round_0001/agent_decision.json').exists()


def test_real_runner_empty_scene_requires_vlm_finish(monkeypatch, tmp_path):
    from test_place_runner import run_fake
    from test_clear_table_runner import snapshot
    config = synthetic_config()
    table = SyntheticTable(config, 'normal')
    scene = snapshot(table, 1)
    scene.update(count=0, physical=[], ids=[], positions=[])
    seen = agent_reply(monkeypatch, 'stop')
    code, result, _, selected = run_fake(monkeypatch, tmp_path, [scene], clear_table=config,
        v2_images=True, extra=['--task-policy', str(live_policy(tmp_path)),
        '--agent-decision-dir', str(tmp_path/'decisions'), '--agent-round-protocol', 'v2'])
    assert code == 1 and not result['passed']
    assert len(seen) == 1 and seen[0]['phase'] == 'completion_review'
    assert not selected
    assert result['error'].startswith('agent_stop')
    assert result['observation_count'] == 2


def test_real_runner_v2_timeout_never_dispatches(monkeypatch, tmp_path):
    from test_place_runner import run_fake
    from test_clear_table_runner import successful_scenes
    config = synthetic_config()
    code, result, _, selected = run_fake(monkeypatch, tmp_path, successful_scenes(config),
        clear_table=config, v2_images=True, extra=['--task-policy', str(live_policy(tmp_path)),
        '--agent-decision-dir', str(tmp_path/'decisions'), '--agent-round-protocol', 'v2'])
    assert code == 1 and not selected
    assert 'timeout' in result['error']


def test_home_check_rejects_stale_moving_and_wrong_pose():
    from types import SimpleNamespace as NS
    from time import monotonic
    from test_live_place_pipeline import ObservationRound
    r = ObservationRound.__new__(ObservationRound)
    r.result = {}
    r.configs = dict(grasp=NS(return_home_joint_positions={'joint':1.}, feedback_timeout=.5,
        future_tolerance=.1, start_joint_tolerance=.01))
    r.node = NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=10**9)))
    m = NS(header=NS(stamp=NS(sec=1,nanosec=0)), name=['joint'],position=[1.],velocity=[0.])
    r.joint_feedback = (m, monotonic())
    assert r.check_home()
    for position, velocity in [(1.1,0.),(1.,.1),(float('nan'),0.)]:
        m.position,m.velocity=[position],[velocity]
        with pytest.raises(RuntimeError): r.check_home()
    m.position,m.velocity=[1.],[0.]
    r.joint_feedback=(m,monotonic()-1.)
    with pytest.raises(RuntimeError,match='stale'):r.check_home()


def test_v2_multi_object_execution_two_frame_verification_and_finish(monkeypatch, tmp_path):
    """Exercise actual runner/state machine; all ROS and actuator calls are fake."""
    from test_place_runner import run_fake
    from test_clear_table_runner import snapshot
    from table_agent_protocol import policy_hash
    config = synthetic_config()
    config['placement_confirmations'] = 2
    table = SyntheticTable(config, 'normal')
    ledger = ClearTableSession(config)
    scenes = []
    for number in range(1, 20):
        scenes.append(snapshot(table, number))
        vision, batch = table.observation(number, 'test')
        decision = ledger.observe(vision, batch)
        if decision['kind'] == 'ready':
            ledger.record_motion(decision)
            table.simulate_placement(decision)
        if decision['kind'] == 'complete': break
    shared_tray = tmp_path/'shared_tray.json'
    shared_tray.write_text(json.dumps(config))
    preview = tmp_path/'preview'
    preview.mkdir()
    preview_policy = live_policy(preview)
    with monkeypatch.context() as mp:
        agent_reply(mp)
        code, result, _, _ = run_fake(mp, preview, scenes, clear_table=config,
            v2_images=True, extra=['--tray-config', str(shared_tray), '--task-policy', str(preview_policy),
            '--agent-decision-dir', str(preview/'decisions'), '--agent-round-protocol', 'v2'])
        assert code == 0, result
    execute = tmp_path/'execute'
    execute.mkdir()
    policy = json.loads(preview_policy.read_text())
    policy.update(mode='execute', plan_id=str(uuid.uuid4()))
    policy['model']['plan_hash'] = policy_hash(policy)
    policy_path = execute/'policy.json'
    policy_path.write_text(json.dumps(policy))
    with monkeypatch.context() as mp:
        seen = agent_reply(mp)
        code, result, commands, selected = run_fake(mp, execute, scenes, execute=True, clear_table=config,
            v2_images=True, extra=['--tray-config', str(shared_tray), '--task-policy', str(policy_path),
            '--baseline-policy', str(preview_policy), '--baseline-result', str(preview/'output/result.json'),
            '--agent-decision-dir', str(execute/'decisions'), '--agent-round-protocol', 'v2'])
        assert code == 0, result
        assert result['verified_placed_count'] == result['motion_completed_count'] == len(selected) == 3
        assert result['observation_count'] > len(seen)
        assert [r['phase'] for r in seen] == ['select', 'select', 'select', 'completion_review']
        assert [r['previous_action']['verified_placed_count'] for r in seen] == [0, 1, 2, 3]
        assert all(r['previous_action']['returned_home'] is True for r in seen[1:])
        assert all(r['previous_action']['grasp_verified'] is False for r in seen)
        assert len({r['scene_revision'] for r in seen}) == 4


def test_cancel_waiting_runner_exits_without_dispatch(monkeypatch, tmp_path):
    import test_live_place_pipeline as runner
    from test_place_runner import run_fake
    from test_clear_table_runner import successful_scenes
    config = synthetic_config()
    original = runner.publish_request
    def cancel(directory, *args, **kwargs):
        request = original(directory, *args, **kwargs)
        atomic_json(directory.parent.parent/'cancel.json', dict(cancel=True))
        return request
    monkeypatch.setattr(runner, 'publish_request', cancel)
    code, result, _, selected = run_fake(monkeypatch, tmp_path, successful_scenes(config),
        clear_table=config, v2_images=True, extra=['--task-policy', str(live_policy(tmp_path)),
        '--agent-decision-dir', str(tmp_path/'decisions'), '--agent-round-protocol', 'v2'])
    assert code == 1 and not selected and result['error'] == 'interrupted'


def test_changed_config_rejected_before_binding(monkeypatch, tmp_path):
    import test_live_place_pipeline as runner
    from test_place_runner import run_fake
    from test_clear_table_runner import successful_scenes
    config = synthetic_config()
    agent_reply(monkeypatch)
    original = runner.ObservationRound.wait_for_v2
    def change(self, *args):
        # Mutate only this test's policy after the source guard was captured.
        path = next(Path(p) for p in self.source_guard if p.endswith('/policy.json'))
        path.write_text(path.read_text()+'\n')
        return original(self, *args)
    monkeypatch.setattr(runner.ObservationRound, 'wait_for_v2', change)
    code, result, _, selected = run_fake(monkeypatch, tmp_path, successful_scenes(config),
        clear_table=config, v2_images=True, extra=['--task-policy', str(live_policy(tmp_path)),
        '--agent-decision-dir', str(tmp_path/'decisions'), '--agent-round-protocol', 'v2'])
    assert code == 1 and not selected and 'agent_configuration_changed' in result['error']


def test_home_refresh_requires_new_sample_after_expensive_scene_work(monkeypatch):
    import test_live_place_pipeline as runner
    from types import SimpleNamespace as NS
    clock = [10.]
    monkeypatch.setattr(runner.time, 'monotonic', lambda:clock[0])
    r = runner.ObservationRound.__new__(runner.ObservationRound)
    r.result = {}
    r.deadline = 100.
    r.configs = dict(grasp=NS(return_home_joint_positions={'joint':1.}, feedback_timeout=.5,
        future_tolerance=.1, start_joint_tolerance=.01))
    r.node = NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=int(clock[0]*10**9))))
    old = NS(header=NS(stamp=NS(sec=8,nanosec=0)), name=['joint'], position=[1.], velocity=[0.])
    r.joint_feedback = (old, 8.)
    with pytest.raises(RuntimeError,match='stale'):r.check_home()
    def fresh():
        clock[0] += .1
        old.header.stamp = NS(sec=10, nanosec=100000000)
        r.joint_feedback = (old, clock[0])
    r.spin = fresh
    assert r.refresh_home()
    old.position = [1.2]
    with pytest.raises(RuntimeError,match='not_stationary_at_home'):r.refresh_home()
