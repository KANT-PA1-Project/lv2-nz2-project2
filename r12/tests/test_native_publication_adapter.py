import copy
import hashlib
import json
from pathlib import Path
from fractions import Fraction
import pytest
from arm_practice.native_rgbd.publication_adapter import NativePublicationAdapter

FIXTURE=Path(__file__).resolve().parent/'data/v54_publication_replay.json'
def digest(x):return hashlib.sha256(json.dumps(x,sort_keys=True).encode()).hexdigest()
def setup():
    rows=json.loads(FIXTURE.read_text());pinned={digest(r['certificate'])for r in rows}
    def verify(c):
        # Offline fixture integrity only; not a recovered live HMAC verification.
        if digest(c) not in pinned:raise ValueError('modified_archived_capture')
        return copy.deepcopy(c['body'])
    p=rows[0]['packet'];return rows,NativePublicationAdapter(run_id=p['run_id'],initial_step=p['native_packet_timeline']['previous_packet_step'],verify_capture=verify)
def prepare(a,r,now=None):return a.prepare(packet=r['packet'],certificate=r['certificate'],now_ns=r['packet']['received_ns']if now is None else now)

def test_actual_replay_separates_capture_current_and_each_imu():
    rows,a=setup()
    for row in rows:
        original=copy.deepcopy(row);plan=prepare(a,row);t=row['packet']['native_packet_timeline']
        assert plan['clock']['stamp_ns']==t['stamp_ns']
        assert plan['historical']['stamp_ns']==t['capture_stamp_ns']<plan['clock']['stamp_ns']
        assert plan['historical']['camera_tf']==row['certificate']['body']['encoder_history']['camera_fk']
        assert plan['historical']['perception_request']['stamp_ns']==t['capture_stamp_ns']
        assert len(plan['imu_native_samples'])==16
        for x in plan['imu_native_samples']:
            actual=Fraction.from_float(x['native_time_s'])
            assert actual==Fraction(x['native_time_num'],x['native_time_den'])
            assert 0<=actual*10**9-x['stamp_ns']<1
            assert x['stamp_ns']!=t['stamp_ns']
        assert row==original and not plan['mission_admitted'] and not plan['publication_authorized']

@pytest.mark.parametrize('kind',['clock','packet_stamp','capture_stamp','imu_stamp','imu_reverse','imu_duplicate','imu_mean','encoder_step','old_feedback','capture_mutation','previous_step','wrong_run'])
def test_tampering_rejected_without_advancing_state(kind):
    rows,a=setup();r=copy.deepcopy(rows[0]);p=r['packet'];t=p['native_packet_timeline']
    if kind=='clock':t['clock_domain']='synthetic_packet_20hz'
    if kind=='packet_stamp':t['stamp_ns']+=1
    if kind=='capture_stamp':t['capture_stamp_ns']=t['stamp_ns']
    if kind=='imu_stamp':p['all_IMU_samples'][0]['native_samples'][0]['time']+=1
    if kind=='imu_reverse':p['all_IMU_samples'][0]['native_samples'].reverse()
    if kind=='imu_duplicate':p['all_IMU_samples'][0]['native_samples'][1]=copy.deepcopy(p['all_IMU_samples'][0]['native_samples'][0])
    if kind=='imu_mean':p['all_IMU_samples'][0]['gyro'][0]+=1
    if kind=='encoder_step':p['current_feedback']['position']['physics_step']-=8
    if kind=='old_feedback':p['current_feedback']['position']['read_start_ns']-=100_000_001
    if kind=='capture_mutation':r['certificate']['body']['capture_physics_step']+=16
    if kind=='previous_step':t['previous_packet_step']+=8
    if kind=='wrong_run':p['run_id']='other'
    with pytest.raises(ValueError):prepare(a,r)
    assert a.last_capture==-1
    prepare(a,rows[0])

def test_duplicate_and_reordered_packets_rejected():
    rows,a=setup()
    with pytest.raises(ValueError):prepare(a,rows[1])
    prepare(a,rows[0])
    with pytest.raises(ValueError):prepare(a,rows[0])
    prepare(a,rows[1])
    with pytest.raises(ValueError):prepare(a,rows[0])

def test_wall_age_not_refreshed_by_packet_stamp():
    rows,a=setup()
    with pytest.raises(ValueError):prepare(a,rows[0],rows[0]['packet']['received_ns']+500_000_001)
