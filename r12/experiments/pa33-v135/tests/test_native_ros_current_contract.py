import copy
import pytest
from arm_practice.native_rgbd.ros_current_contract import CurrentROSGuard

def data():
 q=[0.]*5
 return dict(kind='current_feedback',identity=dict(run_id='r',source_sha256='s',stream_id='ros-current-r'),sequence=1,feedback=dict(position=dict(physics_step=12,q=q,read_start_ns=110,read_end_ns=120),velocity=dict(step=12,q=q,read_start_ns=100,read_end_ns=105,velocity=[0.]*5,velocity_valid=True,sim_time_s=12/960)))
def test_same_sample_delayed_image_plan():
 g=CurrentROSGuard('r','s');d=data();assert g.validate(d,130)==12;g.validate(d,140,commit=False);current=copy.deepcopy(d['feedback']);current['position']['read_start_ns']=130;current['position']['read_end_ns']=135;assert g.match_image_plan(current)
@pytest.mark.parametrize('mutation',['step','timestamp','q','qd'])
def test_image_epoch_mismatch(mutation):
 g=CurrentROSGuard('r','s');d=data();g.validate(d,130);c=copy.deepcopy(d['feedback'])
 if mutation=='step':c['velocity']['step']=13
 if mutation=='timestamp':c['velocity']['read_start_ns']=101
 if mutation=='q':c['position']['q']=[1.]*5
 if mutation=='qd':c['velocity']['velocity']=[1.]*5
 with pytest.raises(ValueError):g.match_image_plan(c)
def test_replay_and_stale_callback():
 g=CurrentROSGuard('r','s');d=data();g.validate(d,130)
 with pytest.raises(ValueError,match='replay'):g.validate(d,140)
 with pytest.raises(ValueError,match='100ms'):g.validate(d,100000101,commit=False)
def test_small_mixed_wire_roundtrip():
 import socket
 from pathlib import Path
 from arm_practice.native_rgbd.mixed_camera_schema import MixedCameraSchema
 from arm_practice.native_rgbd.trusted_capture_ipc import send_packet,receive_packet
 schema=MixedCameraSchema.from_file(Path(__file__).resolve().parents[1]/'configs/mixed_camera_schema.json');a,b=socket.socketpair()
 try:
  send_packet(a,data(),wire_version=schema.wire_schema,camera_schema=schema)
  d,blobs=receive_packet(b,wire_version=schema.wire_schema,camera_schema=schema,allow_control_finish=True);assert d==data()and not blobs
 finally:a.close();b.close()
