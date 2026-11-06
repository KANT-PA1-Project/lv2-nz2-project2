import numpy as np
from arm_practice.ik_start_state import check_start_state

def check(**kw):
    args=dict(home=np.zeros(5),positions=np.zeros(5),velocities=np.zeros(5),now=1.,state_stamp=1.,base_twist=[0,0,0],lower=np.full(5,-.35),upper=np.full(5,.35));args.update(kw);return check_start_state(**args)

def test_transport_requires_return_plan():
    r=check(positions=[0,.33,.33,.33,0]);assert r['reason']=='requires_separate_return_to_home_plan' and not r['execution_authorized']

def test_stale_future_nonfinite_rejected():
    for kw in [dict(state_stamp=.5),dict(state_stamp=2),dict(positions=[0,0,np.nan,0,0]),dict(velocities=[0])]:
        assert not check(**kw)['eligible_for_path_validation']

def test_motion_and_limits_rejected():
    for kw in [dict(velocities=[0,0,.02,0,0]),dict(base_twist=[.02,0,0]),dict(positions=[0,.36,0,0,0])]:
        assert not check(**kw)['eligible_for_path_validation']

def test_home_is_only_path_validation_eligibility():
    r=check();assert r['eligible_for_path_validation'] and not r['execution_authorized']
