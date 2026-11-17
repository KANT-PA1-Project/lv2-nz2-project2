import copy
import json
from pathlib import Path
import pytest
from arm_practice.native_rgbd.packet_timeline import NativePacketTimeline,f32

def streams(start,end):
    return [[dict(physics_step=s,physics_time_s=s*f32(1/960),time=f32(s*f32(1/960)-1/960),valid=True,gyro=[1.,2.,3.],accel=[4.,5.,6.])for s in range(start+8,end+1,8)]for _ in range(2)]

def body(step):return dict(run_id='test',physics_hz=960,capture_physics_step=step,capture_monotonic_lower_bound_ns=123)

def test_regular_packet_has_eight_real_samples_and_separate_capture():
    t=NativePacketTimeline('test',100);t.add(streams(100,148));t.add(streams(148,164));b=body(148);old=copy.deepcopy(b)
    p=t.packet(current_step=164,capture_body=b)
    assert p['stamp_ns']==164*10**9//960
    assert p['capture_stamp_ns']==148*10**9//960
    assert p['imu'][0]['native_sample_count']==8
    assert p['packet_dt_ns']==164*10**9//960-100*10**9//960
    assert b==old and not p['motion_authorized']

def test_failed_capture_frames_remain_in_interval():
    t=NativePacketTimeline('test',100)
    for start,end in [(100,148),(148,196),(196,244),(244,260)]:t.add(streams(start,end))
    p=t.packet(current_step=260,capture_body=body(244))
    assert p['imu'][0]['native_sample_count']==20
    assert p['imu'][0]['native_samples'][0]['physics_step']==108
    assert p['clock_domain']=='native_physics_960hz'

@pytest.mark.parametrize('fault',['repeat','missing','wrong_phase','nan','bad_vector','old_clock','pair_mismatch'])
def test_reject_bad_samples_transactionally(fault):
    t=NativePacketTimeline('test',100);s=streams(100,116)
    if fault=='repeat':s[0][1]=copy.deepcopy(s[0][0])
    if fault=='missing':s[0]=s[0][1:];s[1]=s[1][1:]
    if fault=='wrong_phase':s[0][0]['time']=f32(s[0][0]['physics_time_s'])
    if fault=='nan':s[0][0]['time']=float('nan')
    if fault=='bad_vector':s[0][0]['gyro']=[1,2]
    if fault=='old_clock':s[0][0]['time']=1.05
    if fault=='pair_mismatch':s[1][0]['physics_step']+=8
    with pytest.raises(ValueError):t.add(s)
    assert t.pending==[[],[]]

@pytest.mark.parametrize('fault',['missing_tail','wrong_run','wrong_capture','repeat_packet'])
def test_reject_invalid_packet(fault):
    t=NativePacketTimeline('test',100);t.add(streams(100,164));b=body(148);current=164
    if fault=='missing_tail':current=172;b=body(156)
    if fault=='wrong_run':b['run_id']='other'
    if fault=='wrong_capture':b['capture_physics_step']=100
    if fault=='repeat_packet':current=100
    with pytest.raises(ValueError):t.packet(current_step=current,capture_body=b)
    assert len(t.pending[0])==8

def test_real_v53b_postcapture_samples_match_representation():
    path=Path(__file__).resolve().parent/'data/v53b_frames.json'
    for frame in json.loads(path.read_text()):
        if not frame['captured']:continue
        post=frame['postcapture_feedback'];t=NativePacketTimeline('test',post['start_step'])
        t.add([[dict(row['imus'][i],physics_step=row['encoder']['step'],physics_time_s=row['encoder']['sim_time_s'])for row in post['samples']]for i in range(2)])
        packet=t.packet(current_step=post['end_step'],capture_body=body(post['capture_step']))
        assert packet['imu'][0]['native_sample_count']==2
        assert not packet['observation_admitted']


def test_reject_joint_shift_of_sim_and_imu_time():
    t=NativePacketTimeline('test',100);s=streams(100,116)
    for stream in s:
        for row in stream:
            row['physics_time_s']+=1
            row['time']=f32(row['physics_time_s']-1/960)
    with pytest.raises(ValueError,match='physics_step_time_mismatch'):t.add(s)
    assert t.pending==[[],[]]

def test_pending_budget_rejects_without_dropping_evidence():
    t=NativePacketTimeline('test',100,max_pending_samples=8);t.add(streams(100,164))
    old=copy.deepcopy(t.pending)
    with pytest.raises(ValueError,match='budget_exceeded'):t.add(streams(164,172))
    assert t.pending==old

def test_long_simulation_float32_step_accumulation():
    t=NativePacketTimeline('test',96000)
    t.add(streams(96000,96016))
    assert len(t.pending[0])==2
