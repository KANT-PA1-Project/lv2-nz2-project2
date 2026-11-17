import copy,hashlib
import pytest
from arm_practice.native_rgbd.live_decode_contract import validate,fresh_at_completion

def example():
 b=dict(run_id='r',capture_physics_step=12,capture_monotonic_lower_bound_ns=100,issued_ns=150,encoder_history={'q':[1]},pixel_sha256={'c':hashlib.sha256(b'abc').hexdigest()})
 v=dict(rgbd_capture_association_verified=True,run_id='r',capture_physics_step=12,encoder_history={'q':[1]})
 return dict(kind='capture',run_id='r',certificate={'body':b},live_issuer_verification=v,arrays=[dict(name='c',kind='rgb')])
def test_good():assert validate(example(),[b'abc'],run_id='r',last_step=11,now_ns=200)['capture_physics_step']==12
@pytest.mark.parametrize('mode',['run','bodyrun','verified','stepbinding','history','duplicate','reverse','future','stale','hash','missing'])
def test_fail_closed(mode):
 m=example();blob=[b'abc'];last=11;now=200
 if mode=='run':m['run_id']='other'
 if mode=='bodyrun':m['certificate']['body']['run_id']='other'
 if mode=='verified':m['live_issuer_verification']['rgbd_capture_association_verified']='true'
 if mode=='stepbinding':m['live_issuer_verification']['capture_physics_step']=13
 if mode=='history':m['live_issuer_verification']['encoder_history']={'q':[2]}
 if mode=='duplicate':last=12
 if mode=='reverse':last=13
 if mode=='future':now=149
 if mode=='stale':now=500000101
 if mode=='hash':blob=[b'abd']
 if mode=='missing':blob=[]
 with pytest.raises(ValueError):validate(m,blob,run_id='r',last_step=last,now_ns=now)
def test_completion_boundary():
 assert fresh_at_completion(100,500000100)
 assert not fresh_at_completion(100,500000101)
 assert not fresh_at_completion(100,99)
