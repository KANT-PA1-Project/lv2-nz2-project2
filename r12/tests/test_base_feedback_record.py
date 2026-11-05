from arm_practice.base_feedback_record import base_feedback_record
from arm_practice.learned_ik_candidate import prepare_from_sensor_packet

def test_uses_sensor_twist_and_rejects_invalid_inputs():
    p=dict(stamp_ns=10,frame_capture_monotonic_ns=20,capture_run_id='a',command=[9,9,9])
    r=base_feedback_record(p,[.1,.2,99],True,{'gyro':[0,0,.3]},.01)
    assert r['valid'] and r['twist']==[.1,.2,.3]
    assert not base_feedback_record(p,[0,0,0],False,{'gyro':[0,0,0]},0)['valid']
    assert not base_feedback_record(p,[0,0,0],True,None,0)['valid']
    assert not base_feedback_record(p,[0,0,0],True,{'gyro':[0,0,float('nan')]},0)['valid']
    # Missing sensor provenance must reject before accessing a model.
    assert not prepare_from_sensor_packet(None,[0,0,0],{},p,frame_index=0,run_id='a',now_ns=30)['accepted']
