import copy
import pytest

from clear_table_offline import SyntheticTable, synthetic_config
from place_rounds import ClearTableSession

from table_agent_protocol import (scene_revision, validate_round_decision,
                                  validate_task_policy)


def policy(mode='preview'):
    return {
        'schema_version': 1,
        'plan_id': 'plan-1',
        'task_type': 'clear_table',
        'selector': {'kind': 'all_bowls'},
        'priority': {'order': 'largest_first', 'regions': []},
        'limits': {'max_observation_rounds': 3, 'max_replans': 2, 'deadline_s': 30.},
        'mode': mode,
    }


def decision(revision='rev-1'):
    return {
        'schema_version': 1,
        'plan_id': 'plan-1',
        'task_id': 'task-1',
        'observation_id': 'task-1/round_0001',
        'scene_revision': revision,
        'selection': {'object_id': 2, 'candidate_id': 1, 'slot_id': 'A'},
    }


def test_policy_and_decision_accept_only_references():
    assert validate_task_policy(policy())['plan_id'] == 'plan-1'
    assert validate_round_decision(decision(), task_id='task-1', plan_id='plan-1',
                                   observation_id='task-1/round_0001', scene_revision='rev-1')['selection']['slot_id'] == 'A'


@pytest.mark.parametrize('field', ['x', 'pose', 'trajectory', 'command'])
def test_policy_rejects_robot_payload(field):
    value = policy()
    value[field] = 1
    with pytest.raises(ValueError, match='invalid_table_agent:fields'):
        validate_task_policy(value)


def test_policy_rejects_unsupported_selector_and_mode_mismatch():
    value = policy()
    value['selector'] = {'kind': 'green'}
    with pytest.raises(ValueError, match='selector_unsupported'):
        validate_task_policy(value)
    with pytest.raises(ValueError, match='mode_mismatch'):
        validate_task_policy(policy('preview'), expected_mode='execute')


def test_decision_rejects_stale_scene_and_geometry():
    with pytest.raises(ValueError, match='scene_revision_mismatch'):
        validate_round_decision(decision(), task_id='task-1', plan_id='plan-1',
                                observation_id='task-1/round_0001', scene_revision='new')
    value = decision()
    value['selection']['target_xy'] = [1., 2.]
    with pytest.raises(ValueError, match='fields'):
        validate_round_decision(value)


def test_scene_revision_is_canonical_and_does_not_mutate_input():
    value = {'b': [2, 1], 'a': {'x': 1}}
    original = copy.deepcopy(value)
    assert scene_revision(value) == scene_revision({'a': {'x': 1}, 'b': [2, 1]})
    assert value == original


def test_session_binds_one_exact_option_and_rejects_duplicate_selection():
    config = synthetic_config()
    session = ClearTableSession(config, policy=policy())
    vision, batch = SyntheticTable(config, 'normal').observation(1, 'task-1')
    automatic = session.observe(vision, batch)
    option = session.list_options()[-1]
    selected = session.apply_selection({
        **decision(session.current_scene_revision()),
        'observation_id': vision['observation_id'],
        'selection': {key: option[key] for key in ('object_id', 'candidate_id', 'slot_id')},
    })
    assert selected['agent_selection']['candidate_id'] == option['candidate_id']
    candidates = selected['selected']['objects'][0]['candidates']
    assert [c['candidate_id'] for c in candidates if not c.get('disabled')] == [option['candidate_id']]
    with pytest.raises(ValueError, match='decision_already_applied'):
        session.apply_selection({
            **decision(session.current_scene_revision()),
            'observation_id': vision['observation_id'],
            'selection': {key: option[key] for key in ('object_id', 'candidate_id', 'slot_id')},
        })


def test_failed_pre_motion_candidates_are_ledgered_across_fresh_observation():
    config = synthetic_config()
    session = ClearTableSession(config, policy=policy())
    table = SyntheticTable(config, 'normal')
    vision, batch = table.observation(1, 'task-1')
    first = session.observe(vision, batch)
    failures = [{'candidate_id': candidate['candidate_id'], 'object_id': first['source']['object_id'],
                 'stage': 'workspace', 'reason': 'pose_outside_workspace'}
                for candidate in first['selected']['objects'][0]['candidates']]
    session.register_plan_failure(first, {'command_attempted': False, 'candidate_failures': failures})
    vision2, batch2 = table.observation(2, 'task-1')
    second = session.observe(vision2, batch2)
    assert session.replans_used == 1
    assert second['kind'] == 'ready'
    assert second['source']['object_id'] != first['source']['object_id'] or second['slot']['id'] != first['slot']['id']
    assert all(item['scene_revision'] == first.get('scene_revision', session.failed_options[0]['scene_revision'])
               for item in session.failed_options)
