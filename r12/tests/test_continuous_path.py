import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts/sensor_runtime'))
from continuous_path import ContinuousPath, integrate_pose, safe_reference, wrap
from arm_practice.motion_envelope import MotionEnvelope


def test_straight_translation_does_not_require_chassis_facing_travel():
    p=ContinuousPath()
    v,s=p.propose([[1,1],[1,3]],[1,1],3*np.pi/4,.2)
    assert abs(v[0])>.05 and abs(v[1])>.05
    assert abs(v[2])<1e-8
    assert s['heading_goal_rad']==3*np.pi/4


def test_target_continuous_across_path_vertex():
    p=ContinuousPath()
    path=[[1,1],[1,1.5],[1.1,1.7],[1.4,1.8]]
    _,a=p.propose(path,[1,1.4999],2.3,.2)
    _,b=p.propose(path,[1,1.5001],2.3,.2)
    assert np.linalg.norm(np.array(a['target_xy'])-b['target_xy'])<.001


def test_shorter_replan_resets_progress_without_endpoint_jump():
    p=ContinuousPath()
    p.propose([[1,1],[1,4]],[1,3],2.3,.2)
    _,s=p.propose([[1,2.9],[1,4]],[1,3],2.3,.2)
    assert abs(s['progress_m']-.1)<1e-8
    assert s['target_xy'][1]<3.5


def test_bend_brakes_in_advance_and_respects_yaw_rate():
    p=ContinuousPath()
    path=[[1,1],[1,2],[1.05,2.15],[1.2,2.2],[2,2.2]]
    v,s=p.propose(path,[1,1.6],3*np.pi/4,.4)
    assert s['speed_cap_m_s']<.4
    assert abs(v[2])<=.2


def test_approach_stops_inside_coarse_region_without_exact_pose():
    p=ContinuousPath(arrival_radius=.1)
    v,_=p.propose([[1,1],[1,2]],[1,1.94],3*np.pi/4,.01)
    assert np.linalg.norm(v[:2])==0


def test_se2_lateral_motion():
    xy,yaw=integrate_pose([1,1],np.pi/2,[0,.3,0],1.)
    np.testing.assert_allclose(xy,[.7,1],atol=1e-10)


def test_near_goal_keeps_entry_heading_across_replans():
    p=ContinuousPath()
    v,_=p.propose([[1,1],[1,2]],[1,1.8],.3,.1)
    assert abs(v[2])<1e-9
    v,_=p.propose([[1.1,1.85],[1,2]],[1.08,1.86],.32,.1)
    assert abs(v[2]+.013)<1e-8
    p.propose([[1,2],[2,2]],[1,2],.32,.1)
    assert p.final_heading is None


def test_single_point_replan_cannot_use_previous_path():
    p=ContinuousPath()
    p.propose([[1,1],[1,2]],[1,1],0)
    v,s=p.propose([[1,1]],[1,1],0)
    assert np.all(v==0) and s['reason']=='empty_path'


def test_preview_rejects_filtered_motion_that_runs_into_wall():
    p=ContinuousPath();e=MotionEnvelope(speed=.6)
    e.velocity=np.array([.5,0,0]);e.initialized=True
    occupied=np.zeros((200,200),bool);occupied[:,65]=True
    fp=[[-.1,-.1],[.1,-.1],[.1,.1],[-.1,.1]]
    v,s=safe_reference(p,[[1,1],[1,2]],[1,1],0,.5,e,[0,0,0],[0,0,9.81],[0,0],{}, {},fp,occupied,.02)
    assert s['preview_scale']<1
    np.testing.assert_array_equal(e.velocity,[.5,0,0])


def test_selected_chassis_heading_survives_bend_and_near_goal():
    selected=np.deg2rad(70)
    path=[[1.,1.],[1.5,1.],[1.7,1.2],[1.7,2.]]
    p=ContinuousPath()
    for xy in ([1.1,1.],[1.55,1.05],[1.7,1.8]):
        command,state=p.propose(path,xy,selected,.2,heading_reference=selected)
        assert abs(wrap(state['heading_goal_rad']-selected))<1e-10
        assert abs(command[2])<1e-10
        assert state['selected_heading_active']
        assert np.linalg.norm(command[:2])>0


def test_selected_heading_tracks_error_not_path_tangent():
    p=ContinuousPath()
    yaw=np.deg2rad(69);selected=np.deg2rad(70)
    command,state=p.propose([[1,1],[2,1]],[1,1],yaw,.2,heading_reference=selected)
    assert abs(command[2]-.65*(selected-yaw))<1e-10
    import pytest
    with pytest.raises(ValueError):
        p.propose([[1,1],[2,1]],[1,1],yaw,heading_reference=float('nan'))
