"""Selection provenance and camera projection for persisted grasp annotations."""
from copy import deepcopy
from dataclasses import replace
import json
import math
from types import SimpleNamespace as NS

import cv2
import numpy as np
import pytest

from grasp_executor.config import GraspConfig
from grasp_pose_overlay import selected_grasp_poses, project_grasp, render_grasp_overlay
from yolo_vision.diagnostics import projection_metadata
from test_live_place_pipeline import ObservationRound


def fixture():
    batch = dict(task_id='task', observation_id='task/round_0001', source_stamp_ns=100,
        frame_id='base_link', tool_frame='grasp_center', objects=[dict(object_id=3, candidates=[
            dict(candidate_id=i, position=[.1, .2, 1.3], orientation=[0.,0.,0.,1.],
                 geometry={'method':'test', 'outside_finger_axis':'positive_y'}) for i in range(2)])])
    def event(status, cid, **extra):
        return dict(node='grasp_executor', status=dict(task_id='task', observation_id='task/round_0001',
            source_stamp_ns=100, object_id=3, candidate_id=cid, status=status, **extra))
    history = [event('planning_candidate',0), event('candidate_rejected',0),
               event('planning_candidate',1), event('moving_to_approach',1,command_attempted=True),
               event('failed',1,command_attempted=True)]
    camera = dict(model='rectified_pinhole', frame_id='camera', width=640,height=480,
        k=[400.,0.,320.,0.,410.,240.,0.,0.,1.], source_stamp_ns=100,image_stamp_ns=102)
    transform = dict(source_frame='camera', target_frame='base_link', translation=[.1,.2,.3],
                     rotation=[0.,0.,math.sqrt(.5),math.sqrt(.5)],source_stamp_ns=100)
    image_header = dict(frame_id='camera',stamp_ns=102)
    return batch, history, camera, transform, image_header


def test_final_candidate_and_offset_once_even_if_execution_later_fails():
    batch, history, *_ = fixture()
    config = GraspConfig(grasp_offset=dict(x=.01,y=-.02,z=.03))
    poses = selected_grasp_poses(batch,history,config)
    assert len(poses)==1 and poses[0]['candidate_id']==1
    assert poses[0]['position']==pytest.approx([.11,.18,1.33])
    assert poses[0]['selection']=='execution_started'
    assert batch['objects'][0]['candidates'][1]['position']==[.1,.2,1.3]


def test_planning_trials_and_foreign_observations_are_not_selected():
    batch, history, *_ = fixture()
    assert selected_grasp_poses(batch,history[:3],GraspConfig())==[]
    foreign=deepcopy(history);foreign[3]['status']['observation_id']='other'
    foreign[4]['status']['source_stamp_ns']=99
    assert selected_grasp_poses(batch,foreign,GraspConfig())==[]
    history[3]['status'].update(status='candidate_planned',command_attempted=False)
    assert selected_grasp_poses(batch,history[:4],GraspConfig())[0]['selection']=='planned_only'


def test_projection_uses_inverse_camera_transform_and_original_pixels():
    batch, history, camera, transform, header=fixture()
    pose=selected_grasp_poses(batch,history,GraspConfig())[0]
    p=project_grasp(pose,camera,transform,header,(480,640,3))
    assert p['origin_pixel']==pytest.approx([320,240])
    assert p['axis_pixels'][0]==pytest.approx([320,227.7])
    assert p['axis_pixels'][1]==pytest.approx([332,240])


@pytest.mark.parametrize('defect',['stamp','frame','dimensions','behind','outside','missing'])
def test_bad_projection_never_places_a_marker_at_a_guessed_pixel(defect):
    batch,history,camera,transform,header=fixture()
    pose=selected_grasp_poses(batch,history,GraspConfig())[0]
    if defect=='stamp':header['stamp_ns']=103
    if defect=='frame':transform['target_frame']='other'
    if defect=='dimensions':camera['width']=800
    if defect=='behind':pose['position'][2]=-.5
    if defect=='outside':pose['position'][0]=10
    if defect=='missing':camera=None
    image, rows=render_grasp_overlay(np.zeros((480,640,3),np.uint8),[pose],camera,transform,header)
    assert 'projection_error' in rows[0] and 'origin_pixel' not in rows[0]
    assert not np.any(np.all(image==[255,0,255],axis=2))


def test_round_saves_image_and_selected_pose_metadata(tmp_path):
    batch,history,camera,transform,header=fixture()
    source=tmp_path/'detection_overlay.png'
    cv2.imwrite(str(source),np.full((480,640,3),100,np.uint8))
    original=source.read_bytes()
    r=NS(selected=batch,history=history,configs={'grasp':GraspConfig()},
         statuses={'yolo_vision':{'camera_projection':camera},'rim_locator':{'cloud_transform':transform}},
         result={'detection_overlay':str(source),'detection_image_header':header},
         task_id=batch['task_id'],observation_id=batch['observation_id'],output=tmp_path,source_paths=())
    ObservationRound.save_selected_pose_overlay(r)
    assert r.result['grasp_pose_overlay_status']=='annotated'
    data=json.loads((tmp_path/'selected_grasp_pose.json').read_text())
    assert data['poses'][0]['candidate_id']==1
    assert data['poses'][0]['origin_pixel']==pytest.approx([320,240])
    image=cv2.imread(r.result['grasp_pose_overlay'])
    assert np.all(image[240,320]==[255,0,255])
    assert source.read_bytes()==original


def test_projection_model_can_be_recovered_from_organized_xyz():
    yy,xx=np.mgrid[:20,:30]
    z=np.ones(xx.shape)
    xyz=np.stack(((xx-15)/40*z,(yy-10)/41*z,z),axis=-1)
    header=NS(frame_id='camera',stamp=NS(sec=1,nanosec=2))
    metadata=projection_metadata(xyz,header,header)
    assert metadata['k']==pytest.approx([40,0,15,0,41,10,0,0,1])
    assert metadata['image_stamp_ns']==metadata['source_stamp_ns']==1000000002
    xyz[0,0,0]+=.5
    with pytest.raises(ValueError,match='projection_not_rectified_pinhole'):
        projection_metadata(xyz,header,header)


def test_projection_metadata_keeps_camera_info_and_distinct_rgb_depth_stamps():
    xyz=np.zeros((480,640,3))
    image=NS(frame_id='camera',stamp=NS(sec=1,nanosec=2))
    depth=NS(frame_id='camera',stamp=NS(sec=1,nanosec=3))
    info=NS(k=[400.,0.,320.,0.,410.,240.,0.,0.,1.])
    metadata=projection_metadata(xyz,image,depth,info)
    assert metadata['k']==info.k and metadata['origin']=='camera_info'
    assert metadata['image_stamp_ns']==1000000002 and metadata['source_stamp_ns']==1000000003
