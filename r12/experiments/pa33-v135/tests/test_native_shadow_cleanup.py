from types import SimpleNamespace
import pytest
import arm_practice.native_rgbd.live_shadow_inputs as m

def test_refresh_receipt_remains_discardable_on_join_failure(monkeypatch):
 new=object();old=object();discarded=[]
 def reject(*a,**k):raise ValueError('injected_join_reject')
 receiver=SimpleNamespace(discard_pair=lambda ch,fh:discarded.append(fh),receive_response=lambda *a,**k:({},None,new),local_packet=lambda fh:{},join=reject)
 x=m.LiveShadowInputs(None,1,'r',SimpleNamespace(archive_fields=lambda:{}));x.inputs=receiver;x.session={'publisher_gid':'g'}
 ctx={'fh':old,'ch':object(),'request':{'nonce':'n'},'pixels':{},'depths':{},'arrays':{},'received_ns':1,'msg':{'certificate':{'body':{'native_camera_metadata':{'arm_camera_left_optical':{'referenceTimeNumerator':1,'referenceTimeDenominator':1}},'capture_physics_step':1,'capture_monotonic_lower_bound_ns':1,'encoder_history':{'camera_fk':{}}}}}}
 monkeypatch.setattr(m,'shadow',lambda *a,**k:{});monkeypatch.setattr(m,'send_packet',lambda *a:None)
 with pytest.raises(ValueError,match='injected_join'):x.finish(ctx,[],None,None,{},0)
 assert ctx['fh'] is new
 receiver.discard_pair(ctx['ch'],ctx['fh']);assert discarded==[old,new]
