"""Variable-size visual geometry and bounded free-space search, no hardware."""
from pathlib import Path
import numpy as np
import pytest
from yolo_vision.tray_utils import (load_tray_config,validate_tray_config,estimate_object_position,
                                  choose_slot,classify_object,slot_xy,tray_polygon)


def config():
    return load_tray_config(Path(__file__).parents[1]/'config/clear_table_live.json')


def tray(c,yaw=0.,center=(.5,.2)):
    return dict(status='ok',frame_id='base',identity='yolo:tray',size=c['tray_size'],yaw=yaw,
                center_xy=list(center),observed_polygon_xy=tray_polygon(center,yaw,c['tray_size']))


def cloud(radius,coverage=270):
    rng=np.random.default_rng(18)
    theta=np.linspace(0,np.deg2rad(coverage),500)
    points=np.concatenate([np.c_[.5+(radius+rng.normal(0,.0005,len(theta)))*np.cos(theta),
                                 .2+(radius+rng.normal(0,.0005,len(theta)))*np.sin(theta),
                                 np.full(len(theta),z)] for z in [.83,.84,.85]])
    return points


@pytest.mark.parametrize('radius',[.045,.060,.065,.0925,.12])
def test_unlisted_radii_measured_from_points(radius):
    c=config();r=estimate_object_position(cloud(radius),np.eye(4),c)
    assert r['position_valid'],r
    assert r['position_reason']=='visual_rim_circle'
    assert r['center_xy']==pytest.approx([.5,.2],abs=.002)
    assert radius <= r['estimated_radius'] < radius+.009
    assert r['radius']==pytest.approx(r['estimated_radius']+r['size_uncertainty'])
    assert not c['known_bowl_radii']


def test_short_arc_has_no_known_size_fallback():
    r=estimate_object_position(cloud(.065,35),np.eye(4),config())
    assert not r['position_valid'] and r['position_reason']=='visual_rim_no_supported_circle'


@pytest.mark.parametrize('yaw',[0.,.7,-1.4])
def test_grid_fits_rotated_tray_and_leaves_other_end_for_next_bowl(yaw):
    c=config();c['position_tolerance']=.015;t=tray(c,yaw)
    first=choose_slot(t,[],set(),c,incoming_radius=.069,remaining_radii=[.064])
    assert first is not None
    assert abs(first['uv'][0])>.055
    occupant=dict(object_id=1,frame_id='base',position_valid=True,center_xy=first['xy'],radius=.069)
    second=choose_slot(t,[occupant],{first['id']},c,incoming_radius=.064)
    assert second is not None
    assert np.linalg.norm(np.array(first['xy'])-second['xy'])>.069+.064+c['position_tolerance']+c['clearance']
    for s in [first,second]:
        assert s['xy']==pytest.approx(slot_xy(t,s,c))
        obj=dict(object_id=2,frame_id='base',position_valid=True,center_xy=s['xy'],radius=s['reserved_radius'])
        assert classify_object(obj,t,c)=='inside'
    moved=tray(c,yaw+.03,(.53,.18))
    assert slot_xy(moved,first,c)!=pytest.approx(first['xy'])


def test_grid_rejects_oversized_bowl_and_observed_tray_cutout():
    c=config();t=tray(c)
    assert choose_slot(t,[],set(),c,incoming_radius=min(t['size'])/2) is None
    t['observed_polygon_xy']=[[.49,.19],[.51,.19],[.51,.21],[.49,.21]]
    assert choose_slot(t,[],set(),c,incoming_radius=.065) is None


@pytest.mark.parametrize('change',[{'grid_step':0.},{'placement_confirmations':0},
 {'placement_confirmations':True},{'known_bowl_radii':[.065]},{'rim_outer_margin':.1},
 {'placement_match_tolerance':float('nan')},{'object_geometry':'guess'}])
def test_invalid_visual_configuration(change):
    c=config();c.update(change)
    with pytest.raises(ValueError,match='invalid_tray'):
        validate_tray_config(c)


def test_large_release_allowance_can_correctly_exhaust_small_tray():
    c=config();c['position_tolerance']=.025;t=tray(c)
    first=choose_slot(t,[],set(),c,incoming_radius=.085)
    assert first is not None
    occupant=dict(object_id=1,frame_id='base',position_valid=True,center_xy=first['xy'],radius=.085)
    assert choose_slot(t,[occupant],{first['id']},c,incoming_radius=.085) is None


@pytest.mark.parametrize('yaw',[0.,.7,-1.4])
def test_empty_tray_prefers_supported_corner_with_five_mm_allowance(yaw):
    c=config();t=tray(c,yaw)
    c['position_tolerance']=.005
    radius=.06
    chosen=choose_slot(t,[],set(),c,incoming_radius=radius)
    limits=np.asarray(t['size'])/2-radius-c['position_tolerance']-c['edge_margin']
    assert np.abs(chosen['uv'])==pytest.approx(limits)
    assert chosen['boundary_clearance']==pytest.approx(c['edge_margin'])
    assert chosen['reserved_radius']==pytest.approx(.065)
    assert classify_object(dict(object_id=0,frame_id='base',position_valid=True,
                               center_xy=chosen['xy'],radius=chosen['reserved_radius']),t,c)=='inside'


def test_edge_preference_respects_observed_cutout_and_existing_bowl():
    from yolo_vision.tray_utils import _disk_inside_polygon
    c=config();t=tray(c)
    t['observed_polygon_xy']=[[.30,.06],[.60,.06],[.70,.16],[.70,.34],[.30,.34]]
    occupied=dict(object_id=1,frame_id='base',position_valid=True,
                  center_xy=[.36,.12],radius=.04)
    chosen=choose_slot(t,[occupied],set(),c,incoming_radius=.04)
    assert chosen is not None
    assert _disk_inside_polygon(chosen['xy'],chosen['reserved_radius']+c['edge_margin'],
                                t['observed_polygon_xy'])
    assert np.linalg.norm(np.asarray(chosen['xy'])-occupied['center_xy'])>chosen['reserved_radius']+.04+c['clearance']


def test_corner_projection_uncertainty_does_not_expand_observed_disk_support(monkeypatch):
    from test_tray_utils import synthetic_tray
    import yolo_vision.tray_utils as module
    c=config();c.update(tray_outer_size=[.20,.14],tray_size=[.18,.12],
                        refine_boundary=False,source_polygon=None)
    image,xyz,_,transform,detections=synthetic_tray(c)
    yy,xx=np.indices(image.shape[:2]);rows,cols=np.nonzero(detections[0].mask)
    detections[0].mask &= (xx-cols.min())+(yy-rows.min())>=8
    exact=module.detect_tray(image,xyz,c,camera_to_base=transform,detections=detections)
    assert exact['reason']=='tray_outline_incomplete'
    original=module._mask_plane
    def measured_error(*args,**kwargs):
        mapping,count,_=original(*args,**kwargs);return mapping,count,.006
    monkeypatch.setattr(module,'_mask_plane',measured_error)
    noisy=module.detect_tray(image,xyz,c,camera_to_base=transform,detections=detections)
    assert noisy['status']=='ok',noisy
    detections[0].mask &= (xx-cols.min())+(yy-rows.min())>=24
    assert module.detect_tray(image,xyz,c,camera_to_base=transform,detections=detections)['status']=='invalid'
