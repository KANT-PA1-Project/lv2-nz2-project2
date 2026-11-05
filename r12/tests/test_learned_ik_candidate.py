import numpy as np
from arm_practice.learned_ik_candidate import prepare_candidate

class Model:
    d=dict(home=np.zeros(5),lower=np.full(5,-1.),upper=np.ones(5),tolerance_mm=1)
    fk=staticmethod(lambda q:np.array(q[:3]))
    calls=0
    def solve(self,target):
        self.calls+=1
        return dict(accepted=True,q=[.1,.1,.1,0,0],proposal_error_mm=0)

def test_candidate_gates_before_solver():
    f=dict(frame_index=np.int64(2),stamp_ns=np.int64(1100000000),clock_domain=np.array('synthetic_packet_20hz'),run_id=np.array('a'),capture_monotonic_ns=np.int64(1000000000))
    p=dict(stamp_ns=1100000000,arm_feedback_stamp_ns=1100000000,arm_feedback_valid=True,capture_run_id='a',frame_capture_monotonic_ns=1000000000,packet_received_monotonic_ns=1010000000,arm_measured_position=[0]*5,arm_measured_velocity=[0]*5)
    m=Model();kw=dict(frame_index=2,run_id='a',now_ns=1020000000,base_twist=[0]*3,base_received_ns=1010000000)
    for changes in [dict(run_id='b'),dict(now_ns=1500000000),dict(base_twist=[.1,0,0]),dict(base_received_ns=None)]:
        assert not prepare_candidate(m,[.1]*3,f,p,**dict(kw,**changes))['accepted']
    assert not prepare_candidate(m,[.1]*3,f,dict(p,arm_measured_position=[.03]*5),**kw)['accepted']
    assert m.calls==0
    result=prepare_candidate(m,[.1]*3,f,p,**kw)
    assert result['accepted'] and result['error_mm']<1 and not result['execution_authorized'] and not result['trajectory_validated']
