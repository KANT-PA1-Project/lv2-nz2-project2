import numpy as np
from arm_practice.frame_packet_identity import check_frame_packet_identity as check

def test_identity_never_authorizes_execution():
    f=dict(frame_index=np.int64(2),stamp_ns=np.int64(1100000000),clock_domain=np.array('synthetic_packet_20hz'))
    p=dict(stamp_ns=1100000000,arm_feedback_stamp_ns=1100000000,arm_feedback_valid=True)
    assert check(f,p,2)['matched'] and not check(f,p,2)['execution_authorized']
    assert not check(f,p,3)['matched']
    assert not check(f,dict(p,arm_feedback_stamp_ns=1050000000),2)['matched']
    assert not check(f,dict(p,arm_feedback_valid=False),2)['matched']
    assert not check(dict(f,stamp_ns=np.nan),p,2)['matched']
    assert not check(dict(f,clock_domain=np.array('wall_clock')),p,2)['matched']
    assert not check({},p,2)['matched']

def test_provenance_rejects_other_run_and_stale_capture():
    from arm_practice.frame_packet_identity import check_live_frame_provenance as live
    f=dict(frame_index=np.int64(2),stamp_ns=np.int64(1100000000),clock_domain=np.array('synthetic_packet_20hz'),run_id=np.array('run-a'),capture_monotonic_ns=np.int64(100))
    p=dict(stamp_ns=1100000000,arm_feedback_stamp_ns=1100000000,arm_feedback_valid=True,capture_run_id='run-a',frame_capture_monotonic_ns=100,packet_received_monotonic_ns=110)
    assert live(f,p,2,'run-a',120,50)['matched']
    assert not live(f,p,2,'run-a',120,50)['execution_authorized']
    assert not live(f,p,2,'run-b',120,50)['matched']
    assert not live(f,p,2,'run-a',200,50)['matched']
    assert not live(f,p,2,'run-a',105,50)['matched']
