"""Fresh-frame verification and the public runner's visual-geometry binding."""
from copy import deepcopy
import json
import pytest
from clear_table_offline import synthetic_config,SyntheticTable
from place_rounds import ClearTableSession
from test_clear_table_runner import snapshot
from test_place_runner import run_fake


def scene():
    c=synthetic_config()
    c.update(object_geometry='visual_rim',known_bowl_radii=[],compensate_bowl_xy=True,
             placement_mode='grid',slots=[],bowl_radius=.04,
             placement_confirmations=2,placement_match_tolerance=.05)
    # Table simulator uses old slots only to construct the initial occupancy.
    t=SyntheticTable(synthetic_config(),'normal');t.inside=[]
    return c,t


def visual_observation(t,n):
    v,b=t.observation(n,'test')
    for obj,group in zip(v['objects'],b['objects']):
        obj.update(position_reason='visual_rim_circle',center_z=.2,
                   estimated_radius=obj['radius'],observed_rim_radius=obj['radius'],size_uncertainty=.004)
        x,y=obj['center_xy'];r=obj['radius']
        group['candidates'][0]['position']=[-x-r,-y,.2]
        group['distance']=(group['candidates'][0]['position'][0]**2+y*y)**.5
    return v,b


def test_visual_sizes_choose_largest_with_all_original_contacts():
    c,t=scene();v,b=visual_observation(t,1);v['objects'][1]['radius']=.095
    original=deepcopy(b['objects'][1])
    d=ClearTableSession(c).observe(v,b)
    assert d['kind']=='ready' and d['source']['object_id']==original['object_id']
    assert d['selected']['objects']==[original]


def verification_setup():
    c,t=scene();c.update(placement_mode='slots',slots=synthetic_config()['slots'])
    t.sources=t.sources[:1]
    s=ClearTableSession(c);d=s.observe(*visual_observation(t,1));s.record_motion(d);t.simulate_placement(d)
    return c,t,s,d


def test_offset_release_needs_two_stable_frames_and_repeat_invocation_skips_it():
    c,t,s,d=verification_setup()
    # Recorded visual placement can differ from planned slot, while still
    # entirely inside the tray. IDs are different in every observation.
    def observed(n):
        v,b=visual_observation(t,n)
        v['objects'][0]['center_xy'][0]+=.025
        return v,b
    first=s.observe(*observed(2))
    assert first['reason']=='confirming_placement' and s.verified_placed_count==0
    second=s.observe(*observed(3))
    assert s.verified_placed_count==1 and second['reason']=='confirming_source_empty'
    assert s.observe(*observed(4))['kind']=='complete'
    repeated=ClearTableSession(c)
    assert repeated.observe(*observed(5))['reason']=='confirming_source_empty'
    assert repeated.observe(*observed(6))['kind']=='complete'
    assert repeated.motion_completed_count==0


def test_missing_or_uncertain_frame_breaks_confirmation_streak():
    _,t,s,_=verification_setup()
    assert s.observe(*visual_observation(t,2))['reason']=='confirming_placement'
    v,b=visual_observation(t,3);v['tray']=dict(status='not_found',frame_id='base',reason='tray_missing')
    assert s.observe(v,b)['kind']=='observe_again'
    assert s.placement_confirmation_count==0
    assert s.observe(*visual_observation(t,4))['reason']=='confirming_placement'
    assert s.verified_placed_count==0
    s.observe(*visual_observation(t,5));assert s.verified_placed_count==1


def test_empty_grasp_motion_never_verifies_placement():
    c,t=scene();s=ClearTableSession(c);d=s.observe(*visual_observation(t,1));s.record_motion(d)
    for n in [2,3,4]:
        assert s.observe(*visual_observation(t,n))['reason']=='placement_unverified'
        assert s.verified_placed_count==0


def test_edge_occupant_confirms_then_allows_next_bowl_and_new_session():
    import math
    from yolo_vision.tray_utils import classify_object

    c,t=scene()
    c.update(edge_margin=.005,position_tolerance=.005)
    t.config=c
    t.sources=t.sources[:2]
    s=ClearTableSession(c,agent_mode=True)
    d=s.observe(*visual_observation(t,1))
    s.record_motion(d)
    t.simulate_placement(d)
    # Synthetic landing: the measured center is inside, but its conservative
    # radius crosses the short edge by 4.2 mm, as in the field failure.
    landed=t.inside[0]
    landed['uv'][1]=math.copysign(c['tray_size'][1]/2-landed['radius']+.0042,
                                landed['uv'][1])
    landed['id']='grid:' + ':'.join(f'{value:.6f}' for value in landed['uv'])
    v,b=visual_observation(t,2)
    occupant=v['objects'][-1]
    assert classify_object(occupant,v['tray'],c)=='unknown'
    assert s.observe(v,b)['reason']=='confirming_placement'
    assert s.verified_placed_count==0
    assert s.observe(*visual_observation(t,3))['kind']=='ready'
    assert s.verified_placed_count==1 and s.last_placement is None
    assert s.known_inside[0]['radius']==landed['radius']

    # A new task also sees the occupant without inheriting the previous ledger.
    fresh=ClearTableSession(c,agent_mode=True)
    v,b=visual_observation(t,4)
    assert fresh.observe(v,b)['kind']=='ready'
    assert fresh.verified_placed_count==fresh.motion_completed_count==0
    assert len(fresh.known_inside)==1
    for option in fresh.list_options():
        assert option['object_id']==v['objects'][0]['object_id']
        slot=option['slot']
        assert all(abs(uv)+slot['reserved_radius']+c['edge_margin'] <= size/2+1e-9
                   for uv,size in zip(slot['uv'],c['tray_size']))
        assert math.dist(slot['xy'],v['objects'][-1]['center_xy']) > (
            slot['reserved_radius']+landed['radius']+c['clearance'])


def test_observed_occupancy_still_requires_valid_center_within_tray():
    from yolo_vision.tray_utils import classify_object, tray_polygon

    c,t=scene()
    v,_=visual_observation(t,1)
    tray=v['tray']
    tray['observed_polygon_xy']=tray_polygon(tray['center_xy'],tray['yaw'],tray['size'])
    obj=dict(object_id=0,position_valid=True,frame_id='base',radius=.06,
             center_xy=[tray['center_xy'][0]+tray['size'][0]/2-.01,tray['center_xy'][1]])
    assert classify_object(obj,tray,c,observed_occupancy=True)=='inside'
    obj['center_xy'][0]+=.02
    assert classify_object(obj,tray,c,observed_occupancy=True)=='unknown'
    obj.update(position_valid=False,center_xy=None,radius=None)
    assert classify_object(obj,tray,c,observed_occupancy=True)=='unknown'


def test_visual_runner_preserves_candidates_and_binds_current_snapshot(monkeypatch,tmp_path):
    c,t=scene();v,b=visual_observation(t,1)
    shot=dict(count=len(v['objects']),ids=[o['object_id'] for o in v['objects']],
              positions=[g['candidates'][0]['position'] for g in b['objects']],physical=v['objects'],tray=v['tray'])
    code,r,_,selected=run_fake(monkeypatch,tmp_path,[shot],False,clear_table=c)
    assert code==0,r
    assert len(selected[0]['objects'][0]['candidates'])==2
    # Runtime snapshot, not source configuration, contains geometry.
    from pathlib import Path
    cfg=json.loads(Path(r['rounds'][0]['config_snapshots']['grasp']).read_text())
    vision=json.loads(Path(r['rounds'][0]['config_snapshots']['vision']).read_text())
    assert vision['confidence']==.25 and vision['inference_roi']==[330,375,930,595]
    assert vision['scene_classes']==['bowl','tray','cup','plate','spoon'] and vision['inference_tile_size']==0
    assert vision['class_aliases']=={'plate':'bowl','cup':'bowl'}
    assert cfg['tray_placement']['object_id']==selected[0]['objects'][0]['object_id']
    assert cfg['tray_placement']['source_stamp_ns']==selected[0]['source_stamp_ns']
    assert cfg['grasp_offset']==dict(x=0.,y=0.,z=0.)
    assert not r['motion_completed_count']


def test_visual_runner_two_frame_verification_does_not_consume_failure_retries(monkeypatch,tmp_path):
    from yolo_vision.tray_utils import validate_tray_config
    c=synthetic_config();c.update(object_geometry='visual_rim',placement_mode='grid',slots=[],
        bowl_radius=.04,compensate_bowl_xy=True,placement_confirmations=2,placement_match_tolerance=.05)
    c=validate_tray_config(c);t=SyntheticTable(c,'visual');s=ClearTableSession(c);scenes=[]
    for n in range(1,10):
        scenes.append(snapshot(t,n));d=s.observe(*t.observation(n,'test'))
        if d['kind']=='ready':s.record_motion(d);t.simulate_placement(d)
        if d['kind']=='complete':break
    code,r,_,selected=run_fake(monkeypatch,tmp_path,scenes,True,clear_table=c,
                             extra=['--max-reobservations','0'])
    assert code==0,r
    assert r['passed'] and r['verified_placed_count']==3 and r['motion_completed_count']==3
    assert r['observation_count']==8 and len(selected)==3
    assert all(len(b['objects'][0]['candidates'])==2 for b in selected)
