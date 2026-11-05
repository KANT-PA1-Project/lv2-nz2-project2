import copy
import pytest
from arm_practice.native_rgbd.terminal_pad_corridor import TerminalPadCorridor,digest,PADS,PHASE
NOW=1_000_000_000
H='a'*64

def fixture():
 p=dict(run_id='run_cpu',current_encoder_q=[0,0,0,0,0],target_id=2,source='wrist_rgbd_encoder_fk',accepted=True,identity_verified=True,capture_settled=True,current_settled=True,capture_ns=950_000_000,feedback_ns=990_000_000,error_bound=dict(validated=True,scope='simulation_sensor_pose',report_sha256=H,translation_m=.0003,rotation_rad=.003,robot_geometry_m=.0001),shape='cuboid',dimensions_m=[.03,.03,.06])
 m=dict(run_id='run_cpu',q_start=[0,0,0,0,0],q_end=[.001,0,0,0,0],phase=PHASE,target_id=2,open_aperture_m=.075,close_during_segment=False,aperture_rate_m_s=0,delta_tcp_m=[.001,0,0],approach_axis_base=[1,0,0],delta_angle_rad=0,duration_s=1,elapsed_cycle_s=10,remaining_plan_s=100)
 c=dict(run_id='run_cpu',phase=PHASE,target_id=2,capture_ns=p['capture_ns'],feedback_ns=p['feedback_ns'],created_ns=995_000_000,model_sha256=H,world_sha256=H,pad_geometry_sha256=H,receipt_id='synthetic_cpu_receipt',exception_bodies=PADS,exception_object_id=2,approach_axis_base=[1,0,0],swept_segment_certified=True,unknown_volume_clear=True,nonpad_target_min_m=.03225,other_objects_min_m=.04,environment_min_m=.035,self_min_m=.01,intended_pad_target_min_m=.0225,joint_rate_and_limits_verified=True)
 bind(p,m,c);return p,m,c

def bind(p,m,c):c.update(pose_sha256=digest(p),proposal_sha256=digest(m))
def check(p,m,c,v=None):return (v or TerminalPadCorridor()).check(p,m,c,now_ns=NOW,current_model_hash=H,current_world_hash=H,current_pad_hash=H,current_run_id='run_cpu')

def test_valid_cpu_only_and_one_use():
 p,m,c=fixture();v=TerminalPadCorridor();d=check(p,m,c,v)
 assert d.terminal_geometry_valid and not d.motion_admitted and not d.mission_admitted
 assert check(p,m,c,v).reason=='receipt_replay'

@pytest.mark.parametrize('bucket,key,value',[
 ('p','accepted',False),('p','source','fixture_gt'),('p','target_id',True),('p','identity_verified',False),('p','capture_settled',False),('p','current_settled',False),('p','capture_ns',1),('p','feedback_ns',1),('p','shape','cylinder'),('p','dimensions_m',[.04,.03,.06]),
 ('m','run_id','other_run'),('m','q_start',[.1,0,0,0,0]),('m','q_end',[.01,0,0,0,0]),('m','phase','approach'),('m','phase','close'),('m','target_id',3),('m','close_during_segment',True),('m','open_aperture_m',.029),('m','open_aperture_m',.08),('m','aperture_rate_m_s',.001),('m','delta_tcp_m',[-.001,0,0]),('m','delta_tcp_m',[.001,.001,0]),('m','delta_tcp_m',[.004,0,0]),('m','delta_angle_rad',.01),('m','remaining_plan_s',175),
 ('c','target_id',3),('c','exception_object_id',3),('c','exception_bodies',(PADS[0],'/World/Spawn/R12/arm_link5/collision')),('c','exception_bodies',(PADS[0],PADS[0])),('c','created_ns',1),('c','model_sha256','b'*64),('c','world_sha256','b'*64),('c','pad_geometry_sha256','b'*64),('c','swept_segment_certified',False),('c','unknown_volume_clear',False),('c','nonpad_target_min_m',.03),('c','other_objects_min_m',.025),('c','environment_min_m',.025),('c','self_min_m',0),('c','intended_pad_target_min_m',0),('c','approach_axis_base',[0,1,0]),('c','joint_rate_and_limits_verified',False)])
def test_negative(bucket,key,value):
 p,m,c=fixture();{'p':p,'m':m,'c':c}[bucket][key]=value;bind(p,m,c)
 assert not check(p,m,c).terminal_geometry_valid

@pytest.mark.parametrize('key,value',[('validated',False),('scope','fixture'),('report_sha256',''),('translation_m',.005),('rotation_rad',.1),('robot_geometry_m',.005),('translation_m',True),('rotation_rad',-.1)])
def test_uncertainty_rejects(key,value):
 p,m,c=fixture();p['error_bound'][key]=value;bind(p,m,c);assert not check(p,m,c).terminal_geometry_valid

def test_modified_proposal_rejected():
 p,m,c=fixture();m['delta_tcp_m']=[.002,0,0];assert check(p,m,c).reason=='proposal_binding'

def test_malformed_fail_closed():
 p,m,c=fixture();del c['exception_bodies'];assert not check(p,m,c).terminal_geometry_valid

def test_other_object_cannot_receive_exemption():
 p,m,c=fixture();c['other_objects_min_m']=.0225;assert check(p,m,c).reason=='three_cm_margin'
