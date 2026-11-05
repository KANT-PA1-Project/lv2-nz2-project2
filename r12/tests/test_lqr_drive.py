import numpy as np
import pytest
from arm_practice.discrete_lqr import DiscreteLQR,design_joint_lqr
from arm_practice.lqr_drive import EncoderDriveLQR


def setup():
    w=DiscreteLQR.design([[.99]],[[.01]],[[1]],[[1]],sample_period_s=1/120,model_source='synthetic only')
    s=design_joint_lqr([[1]],[[.1]],np.eye(2),[[1]],sample_period_s=1/120,model_source='synthetic only')
    kw=dict(wheel_target=[1]*4,wheel_velocity=[0]*4,steer_target=[.1]*4,steer_position=[0]*4,steer_velocity=[0]*4,steer_target_velocity=[0]*4,wheel_equilibrium_effort=[0]*4,steer_equilibrium_effort=[0]*4,observed_ns=1_000_000,now_ns=2_000_000,safety_permitted=True)
    return EncoderDriveLQR([w]*4,[s]*4),kw


def test_lqr_effort_sign_caps_and_no_motion_admission():
    c,k=setup();r=c.efforts(**k)
    assert np.all(r['wheel_effort_Nm']>0) and np.all(r['wheel_effort_Nm']<=.108)
    assert np.all(r['steer_effort_Nm']>0) and np.all(r['steer_effort_Nm']<=.54)
    assert not r['motion_authorized']

@pytest.mark.parametrize('mutation',[{'safety_permitted':False},{'safety_permitted':1},{'now_ns':10_000_000},{'observed_ns':3_000_000},{'wheel_target':[30]*4},{'steer_target':[2]*4},{'wheel_velocity':[float('nan')]*4}])
def test_invalid_feedback_or_permission_cannot_produce_effort(mutation):
    c,k=setup();k.update(mutation)
    with pytest.raises(ValueError):c.efforts(**k)


def test_back_emf_no_positive_torque_at_no_load():
    c,k=setup();k.update(wheel_velocity=[29.321531434]*4,wheel_target=[29.321531434]*4,wheel_equilibrium_effort=[.1]*4)
    assert np.all(c.efforts(**k)['wheel_effort_Nm']==0)


def test_partial_or_implicit_common_model_rejected():
    c,_=setup()
    with pytest.raises(ValueError):EncoderDriveLQR(c.wheel[:2],c.steering)
    with pytest.raises(ValueError):EncoderDriveLQR(c.wheel[0],c.steering[0])


def test_separate_module_gains_are_used():
    c,k=setup()
    slower=DiscreteLQR.design([[.99]],[[.01]],[[.01]],[[1]],sample_period_s=1/120,model_source='different synthetic module')
    c=EncoderDriveLQR([slower,*c.wheel[1:]],c.steering)
    k['wheel_target']=[.01]*4
    efforts=c.efforts(**k)['wheel_effort_Nm']
    assert efforts[0]<efforts[1]


def test_moving_reference_does_not_create_spurious_damping():
    c,k=setup()
    k.update(steer_position=[.1]*4,steer_target=[.1]*4,steer_velocity=[.2]*4,steer_target_velocity=[.2]*4,steer_equilibrium_effort=[.01]*4)
    np.testing.assert_allclose(c.efforts(**k)['steer_effort_Nm'],[.01]*4)
    k['steer_target_velocity']=[0]*4
    assert np.all(c.efforts(**k)['steer_effort_Nm']<.01)


def test_invalid_reference_rate_rejected():
    c,k=setup();k['steer_target_velocity']=[8]*4
    with pytest.raises(ValueError):c.efforts(**k)
