import time
import pytest
from arm_practice.native_rgbd.postcapture_feedback import PostCaptureFeedback

def run(mode='good'):
    step=[100]
    completed=time.monotonic_ns()
    sequence=PostCaptureFeedback(capture_step=100,capture_lower_bound_ns=completed-1000,capture_completed_ns=completed,physics_hz=960)
    def advance(sub):step[0]+=0 if mode=='repeat' else 1
    def encoder():return dict(step=step[0],read_start_ns=completed-1 if mode=='old' else time.monotonic_ns(),velocity_valid=mode!='invalid',velocity=[0.1]*5)
    def imus():return [dict(valid=True,time=0 if mode=='imu_repeat' else step[0]/960)]*2
    return sequence.run(stepper=advance,read_step=lambda:step[0],sample_encoder=encoder,read_imus=imus)

def test_distinct_actual_steps():
    result=run()
    assert result['end_step']==116
    assert [x['encoder']['step'] for x in result['samples']]==[108,116]
    assert result['capture_step']==100
    assert not result['motion_authorized']

@pytest.mark.parametrize('mode',['repeat','old','invalid'])
def test_reject_invalid_samples(mode):
    with pytest.raises(ValueError):run(mode)

def test_repeated_imu_is_recorded_but_never_admitted():
    result=run("imu_repeat")
    assert not result["imu_distinct_120hz_valid"]
    assert not result["observation_admitted"]
