"""Synthetic diagnostic fixtures; none represent real calibration evidence."""
from pathlib import Path
import json,sys
import numpy as np
import pytest
from arm_practice.native_rgbd.calibrated_approach import CalibratedApproach,digest
from arm_practice.native_rgbd.terminal_pad_corridor import TerminalPadCorridor
H='a'*64

def fk(q):
 t=np.eye(4);t[:3,3]=q[:3];c,s=np.cos(q[3]),np.sin(q[3]);r=np.array([[c,-s,0],[s,c,0],[0,0,1]]);c,s=np.cos(q[4]),np.sin(q[4]);t[:3,:3]=r@np.array([[c,0,s],[0,1,0],[-s,0,c]]);return t

def fixture():
 ident=np.eye(4).tolist();goal=fk([.021,0,0,0,0]);end=fk([.001,0,0,0,0])
 c=dict(physically_calibrated=True,source='measured_calibration',scope='simulation_sensor_grasp',contains_fixture_offsets=False,validated=True,report_sha256=H,dataset_sha256=H,model_sha256=H,fk_sha256=H,object_frame='observed_cuboid',tcp_frame='grasp_tcp',link_frame='arm_link5',camera_frame='arm_camera_left_optical',T_link_camera=ident,T_link_tcp=ident,T_object_tcp=ident,approach_axis_tcp=[1,0,0])
 p=dict(run_id='diagnostic_only',capture_ns=950000000,accepted=True,source='wrist_rgbd_encoder_fk',identity_verified=True,capture_settled=True,target_id=2,capture_q=[0,0,0,0,0],T_base_camera=ident,T_camera_object=goal.tolist(),T_base_object=goal.tolist())
 m=dict(run_id=p['run_id'],target_id=2,phase='terminal_guarded_approach',model_sha256=H,fk_sha256=H,q_start=[0,0,0,0,0],q_end=[.001,0,0,0,0],approach_axis_base=[1,0,0],standoff_m=.02,T_base_tcp_end=end.tolist(),delta_tcp_m=[.001,0,0]);bind(c,p,m);return c,p,m

def bind(c,p,m):m.update(calibration_sha256=digest(c),pose_sha256=digest(p))
def check(c,p,m):return CalibratedApproach(fk).check(c,p,m,now_ns=1000000000,run_id='diagnostic_only',model_sha256=H,fk_sha256=H)

def test_positive_is_diagnostic_only():
 c,p,m=fixture();r=check(c,p,m);assert r.diagnostic_consistent and r.binding_sha256 and not r.motion_admitted and not r.mission_admitted

def test_actual_existing_tcp_config_never_passes():
 root=Path(__file__).resolve().parents[1]
 c=json.loads((root/'configs/r12_tcp_calibration.json').read_text());_,p,m=fixture();assert check(c,p,m).reason=='calibration_unverified'

@pytest.mark.parametrize('bucket,key,value',[
 ('c','physically_calibrated',False),('c','physically_calibrated',1),('c','contains_fixture_offsets',True),('c','source','fixture_gt'),('c','validated',False),('c','report_sha256',''),('c','dataset_sha256',''),('c','model_sha256','b'*64),('c','fk_sha256','b'*64),('c','object_frame','fixture'),('c','tcp_frame','arm_link5'),('c','approach_axis_tcp',[0,0,0]),
 ('p','accepted',False),('p','capture_settled',False),('p','identity_verified',False),('p','source','fixture'),('p','capture_ns',1),('p','capture_q',[.01,0,0,0,0]),('p','run_id','other'),('p','target_id',True),
 ('m','phase','approach'),('m','target_id',3),('m','run_id','other'),('m','q_end',[.01,0,0,0,0]),('m','q_end',[.001,0,0,.01,0]),('m','q_end',[.001,0,0,0,.01]),('m','approach_axis_base',[-1,0,0]),('m','standoff_m',.04),('m','standoff_m',True),('m','delta_tcp_m',[.002,0,0]),('m','q_start',[.002,0,0,0,0]),('m','q_start',[0,.001,0,0,0])])
def test_rejects(bucket,key,value):
 c,p,m=fixture();{'c':c,'p':p,'m':m}[bucket][key]=value;bind(c,p,m);assert not check(c,p,m).diagnostic_consistent

def test_mutated_pose_binding():
 c,p,m=fixture();p['capture_ns']+=1;assert check(c,p,m).reason=='proposal_binding'

def test_non_rigid_transform():
 c,p,m=fixture();c['T_object_tcp'][0][0]=2;bind(c,p,m);assert not check(c,p,m).diagnostic_consistent

def test_bad_fk_is_rejected():
 c,p,m=fixture();bad=lambda q:np.zeros((4,4));assert not CalibratedApproach(bad).check(c,p,m,now_ns=1000000000,run_id='diagnostic_only',model_sha256=H,fk_sha256=H).diagnostic_consistent

def test_v24_pass_does_not_bypass_v23():
 c,p,m=fixture();assert check(c,p,m).diagnostic_consistent
 # Mapping alone has no fresh whole-sweep collision evidence/error budget.
 r=TerminalPadCorridor().check(p,m,{},now_ns=1000000000,current_model_hash=H,current_world_hash=H,current_pad_hash=H,current_run_id='diagnostic_only')
 assert not r.terminal_geometry_valid and not r.motion_admitted
