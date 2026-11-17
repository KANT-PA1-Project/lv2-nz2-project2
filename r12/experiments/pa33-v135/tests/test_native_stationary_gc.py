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
