import pytest,gc
from arm_practice.native_rgbd.stationary_gc import collect_stationary

def args():return dict(stationary=True,capture_pending=False,rpc_pending=False,command=[0.,0.,0.],read_state=lambda:(4,.004,[0]*5,[0]*5))
def test_collect_once_no_timestamp_relabel():
 calls=[];times=iter([100,200]);r=collect_stationary(**args(),collect=lambda:calls.append(1)or 0,clock=lambda:next(times));assert calls==[1]and r['start_ns']==100 and r['end_ns']==200 and r['encoder_epoch_unchanged']and not r['motion_authorized']
@pytest.mark.parametrize('change',[{'stationary':False},{'capture_pending':True},{'rpc_pending':True},{'command':[.01,0.,0.]},{'command':[float('nan'),0.,0.]}])
def test_unsafe_barrier_does_not_collect(change):
 a=args();a.update(change);calls=[]
 with pytest.raises(ValueError,match='quiescent'):collect_stationary(**a,collect=lambda:calls.append(1))
 assert not calls
def test_encoder_change_rejected():
 a=args();states=iter([(4,0,[0]*5),(4,0,[.001]*5)]);a['read_state']=lambda:next(states)
 with pytest.raises(ValueError,match='encoder'):collect_stationary(**a,collect=lambda:0)

def test_reused_mutable_state_rejected():
 a=args();state=[4,.004,[0.]*5,[0.]*5];a['read_state']=lambda:state
 def mutate():state[2][0]=.001;return 0
 with pytest.raises(ValueError,match='changed'):collect_stationary(**a,collect=mutate)

@pytest.mark.parametrize('state',[None,[],[4,.004,[0]*5], [4,float('nan'),[0]*5,[0]*5],[True,0,[0]*5,[0]*5],[4,0,[float('inf')]*5,[0]*5],[4,0,[0]*4,[0]*5]])
def test_invalid_state_no_collection(state):
 a=args();a['read_state']=lambda:state;calls=[]
 with pytest.raises(ValueError,match='state_invalid'):collect_stationary(**a,collect=lambda:calls.append(1))
 assert not calls

@pytest.mark.parametrize('key,value',[('stationary',1),('capture_pending',0),('rpc_pending',None)])
def test_boolean_claims_required(key,value):
 a=args();a[key]=value
 with pytest.raises(ValueError,match='quiescent'):collect_stationary(**a,collect=lambda:0)


def test_collector_exception_propagates_without_success():
    def failing_collect():
        raise RuntimeError('collection failed')
    with pytest.raises(RuntimeError, match='collection failed'):
        collect_stationary(**args(), collect=failing_collect)


def test_gc_policy_change_is_not_accepted():
    original = gc.isenabled()
    def change_policy():
        (gc.disable if original else gc.enable)()
        return 0
    try:
        with pytest.raises(ValueError, match='automatic_policy_changed'):
            collect_stationary(**args(), collect=change_policy)
    finally:
        (gc.enable if original else gc.disable)()
