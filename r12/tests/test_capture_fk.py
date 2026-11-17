from pathlib import Path
import copy,json,importlib.util,shutil
import numpy as np
import pytest
from arm_practice.native_rgbd.capture_fk import CaptureFK,FILES,JOINTS
from arm_practice.kinematics import forward_transform
from arm_practice.native_rgbd.camera_geometry import transforms
ROOT=Path(__file__).resolve().parents[1]
def history(q):
    return dict(physics_step=960,encoder_read_start_ns=100,encoder_read_end_ns=200,joint_names=JOINTS.copy(),encoder_q=q,camera_fk={k:v.tolist()for k,v in transforms(ROOT,dict(zip(JOINTS,q))).items()})
def produce(p,h):return p.produce(run_id='r',frame_index=2,capture_ns=100,history=h,reported_camera_T=h['camera_fk']['arm_camera_left_optical'],now_ns=300)
@pytest.mark.parametrize('q',[[0.]*5,[.2,-.1,.15,-.2,.1],[-.3,.2,-.1,.1,-.2]])
def test_actual_URDF_TCP_camera_and_prepared_rig_agree(q):
    h=history(q);r=produce(CaptureFK(ROOT),h);T=forward_transform(ROOT/FILES[0],'arm_link5',dict(zip(JOINTS,q)))
    assert np.allclose(np.array(r['T_base_tcp'])[:3,3],(T@np.array([.12475,0,0,1]))[:3])
    spec=importlib.util.spec_from_file_location('camera_rig',ROOT/'scripts/sensor_runtime/camera_rig.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    assert np.allclose(r['T_base_camera'],m.PreparedCameraRig(ROOT).transforms(dict(zip(JOINTS,q)))['arm_camera_left_optical'])
    assert r['parent_frame']=='base_link' and r['tcp_frame']=='grasp_tcp' and r['encoder_capture_monotonic_ns']==150
    assert r['physically_calibrated']is False and r['uncertainty_m']is None
@pytest.mark.parametrize('bad',['q_bool','mapping','time','camera','stale'])
def test_mismatched_capture_rejected(bad):
    h=history([0.]*5);p=CaptureFK(ROOT)
    if bad=='q_bool':h['encoder_q'][0]=True
    if bad=='mapping':h['joint_names']=JOINTS[::-1]
    if bad=='time':h['encoder_read_start_ns']=101
    if bad=='camera':h['camera_fk']['arm_camera_left_optical'][0][3]+=.02
    if bad=='stale':
        with pytest.raises(ValueError):p.produce(run_id='r',frame_index=2,capture_ns=100,history=h,reported_camera_T=h['camera_fk']['arm_camera_left_optical'],now_ns=500000101)
        return
    with pytest.raises(ValueError):produce(p,h)
def test_model_mutation_is_not_silent(tmp_path):
    for f in FILES:
        (tmp_path/f).parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/f,tmp_path/f)
    p=CaptureFK(tmp_path);path=tmp_path/FILES[1];path.write_text(path.read_text()+' ')
    with pytest.raises(ValueError,match='geometry_files_changed'):produce(p,history([0.]*5))
def test_wrong_root_frame_rejected(tmp_path):
    for f in FILES:
        (tmp_path/f).parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(ROOT/f,tmp_path/f)
    p=tmp_path/FILES[0];p.write_text(p.read_text().replace('base_link','map'))
    with pytest.raises(ValueError,match='base_or_joint'):CaptureFK(tmp_path)
