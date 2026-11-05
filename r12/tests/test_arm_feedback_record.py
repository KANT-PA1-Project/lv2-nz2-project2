from arm_practice.arm_feedback_record import arm_feedback_record

def test_order_and_source():
    p=dict(names=['b','a'],position=[.2,.1],velocity=[2,1],stamp_ns=123,velocity_valid=True,arm_target=[9,9])
    r=arm_feedback_record(p,['a','b']);assert r['arm_measured_position']==[.1,.2] and r['arm_measured_velocity']==[1,2] and r['arm_feedback_stamp_ns']==123 and r['arm_feedback_valid']
    assert not r['arm_feedback_live_freshness_verified']

def test_invalid_feedback_not_promoted():
    for change in [dict(velocity_valid=False),dict(position=[float('nan')]),dict(stamp_ns=-1)]:
        p=dict(names=['a'],position=[.1],velocity=[0],stamp_ns=123,velocity_valid=True);p.update(change)
        assert not arm_feedback_record(p,['a'])['arm_feedback_valid']

def test_duplicate_names_rejected():
    try:arm_feedback_record(dict(names=['a','a'],position=[0,0],velocity=[0,0],stamp_ns=1),['a'])
    except ValueError:return
    raise AssertionError('Duplicate names accepted')
