"""Tray compensation follows the actually planned contact, without hardware."""
from copy import deepcopy
from dataclasses import replace
import math
import numpy as np
import pytest
from rim_locator.rim_detection import transform_points
from grasp_executor.candidates import run_candidate_group_task
from grasp_executor.config import GraspConfig
from test_candidate_execution import batch, Candidates, rejection


def setup():
    b = batch()
    contacts = b['objects'][0]['candidates']
    contacts[0].update(position=[.30, .10, .20], orientation=[0., 1., 0., 0.])
    contacts[1].update(position=[.36, .04, .20], orientation=[math.sqrt(.5), math.sqrt(.5), 0., 0.])
    p = dict(object_id=0, source_stamp_ns=b['source_stamp_ns'], center=dict(x=.36,y=.1,z=.2),
             target_xy=dict(x=.45,y=.35), rim_radius=.06, contact_tolerance=.015)
    c = GraspConfig(input_mode='candidate_groups', tray_placement=p)
    return b, c


def test_rejected_first_contact_recomputes_second_placement_and_keeps_grasp(tmp_path):
    b,c = setup(); original = deepcopy(b)
    c = replace(c, plan_only=False, execution_journal=str(tmp_path), place_dwell_time=.001,
                return_home_joint_positions=dict.fromkeys(('shoulder_pan_joint','shoulder_lift_joint',
                'elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'),0.),
                return_home_position=dict(x=.3,y=.2,z=.4),return_home_orientation=dict(x=0.,y=1.,z=0.,w=0.))
    interface = Candidates([rejection('place_ready')])
    result = run_candidate_group_task(b,lambda:2_000_000_000,interface,c)
    assert result.status == 'success',result
    assert result.candidate_id == 1 and result.candidate_attempt_count == 2
    assert len(interface.moves) == 7 and interface.events.count('close') == 1
    assert b == original
    assert interface.plans[0][2].position != interface.plans[1][2].position
    center = np.array([.36,.1,.2])
    for candidate,poses in zip(b['objects'][0]['candidates'],interface.plans):
        approach,grasp,ready,drop,_ = poses
        assert list(grasp.position) == candidate['position']
        rg = transform_points(np.eye(3),[0.,0.,0.],grasp.orientation).T
        rr = transform_points(np.eye(3),[0.,0.,0.],ready.orientation).T
        placed_center = np.asarray(drop.position) + rr @ rg.T @ (center - grasp.position)
        assert placed_center[:2] == pytest.approx([.45,.35])
        assert ready.position[2] == c.place_ready_position['z']
        assert drop.position[2] == pytest.approx(ready.position[2]-c.place_drop_distance)
    assert interface.moves[3] == interface.plans[1][2]


def test_geometry_rejected_contact_can_try_next_contact():
    b,c = setup();b['objects'][0]['candidates'][0]['position'][0] -= .06
    b['objects'][0]['distance']=math.hypot(*b['objects'][0]['candidates'][0]['position'][:2])
    interface = Candidates()
    r=run_candidate_group_task(b,lambda:2_000_000_000,interface,c)
    assert r.status=='planned' and r.candidate_id==1
    assert r.candidate_failures[0]['stage']=='placement_geometry'
    assert not interface.moves


@pytest.mark.parametrize('field,value',[('object_id',1),('source_stamp_ns',5)])
def test_observation_binding_blocks_mismatched_geometry(field,value):
    b,c=setup();p=deepcopy(c.tray_placement);p[field]=value
    interface=Candidates()
    r=run_candidate_group_task(b,lambda:2_000_000_000,interface,replace(c,tray_placement=p))
    assert r.reason=='tray_placement_observation_mismatch' and not interface.plans and not interface.moves


@pytest.mark.parametrize('field,value',[('rim_radius',float('nan')),('contact_tolerance',.1),
                                        ('object_id',True),('source_stamp_ns',0)])
def test_invalid_placement_config_is_rejected(field,value):
    _,c=setup();p=deepcopy(c.tray_placement);p[field]=value
    with pytest.raises(ValueError,match='tray_placement'):
        replace(c,tray_placement=p)
