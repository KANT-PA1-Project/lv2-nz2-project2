from types import SimpleNamespace
import pytest
import arm_practice.native_rgbd.live_decode_bridge as b

def make_bridge(monkeypatch,requests):
 x=b.LiveDecodeBridge.__new__(b.LiveDecodeBridge);x.pending=(0,{});x.conn=object();x.child=SimpleNamespace(pid=7);x.run_id='r';x.expected={'kind':'verify','nonce':'n'};x.verified_once=False;x.refreshed_once=False;x.certificate={'body':{'capture_monotonic_lower_bound_ns':100}};x.issuer=SimpleNamespace(epoch='e',source='s',model='m',joints=['j']);x.stream='s';x.gid='g';x.verification={'matched':True};x.feedback_sequence=0;reads=[];sent=[]
 x.read_feedback=lambda:reads.append(1)or {'q':[0]*5,'read_start_ns':200,'read_end_ns':201,'physics_step':4}
 it=iter(requests);monkeypatch.setattr(b,'receive_packet',lambda sock:(next(it),[]));monkeypatch.setattr(b,'peer_credentials',lambda sock:(7,0));monkeypatch.setattr(b,'require_peer',lambda *a:None);monkeypatch.setattr(b,'send_packet',lambda sock,data:sent.append(data));monkeypatch.setattr(b.time,'monotonic_ns',lambda:300)
 return x,reads,sent

def test_verify_refresh_reply_reads_actual_twice(monkeypatch):
 x,reads,sent=make_bridge(monkeypatch,[{'kind':'verify','nonce':'n'},{'kind':'refresh_local_feedback','nonce':'n'},{'kind':'decoded','frame':0}]);assert x.collect(0)['frame']==0;assert len(reads)==2;assert [r['local_feedback']['sequence']for r in sent]==[1,2];assert all(r['local_feedback']['read_start_ns']==200 for r in sent)
def test_duplicate_verify_rejected(monkeypatch):
 x,reads,sent=make_bridge(monkeypatch,[{'kind':'verify','nonce':'n'},{'kind':'verify','nonce':'n'}])
 with pytest.raises(ValueError,match='replay'):x.collect(0)
 assert len(reads)==1

def test_wrong_nonce_rejected(monkeypatch):
 x,reads,sent=make_bridge(monkeypatch,[{'kind':'verify','nonce':'wrong'}])
 with pytest.raises(ValueError,match='binding'):x.collect(0)
 assert not reads

def test_capture_expired_rejected(monkeypatch):
 x,reads,sent=make_bridge(monkeypatch,[{'kind':'verify','nonce':'n'}]);monkeypatch.setattr(b.time,'monotonic_ns',lambda:500000101)
 with pytest.raises(ValueError,match='expired'):x.collect(0)
 assert not reads

def test_early_verify_then_refresh_retains_identity_and_reads(monkeypatch):
 x,reads,sent=make_bridge(monkeypatch,[{'kind':'verify','nonce':'n'},{'kind':'refresh_local_feedback','nonce':'n'},{'kind':'decoded','frame':0}]);x.verify_before_publication();assert x.verified_once;assert x.collect(0)['frame']==0;assert len(reads)==2;assert [r['local_feedback']['sequence']for r in sent]==[1,2]
@pytest.mark.parametrize('rpc_msg,now,match',[({'kind':'verify','nonce':'wrong'},300,'binding'),({'kind':'refresh_local_feedback','nonce':'n'},300,'replay'),({'kind':'verify'},300,'binding'),({'kind':'verify','nonce':'n'},500000101,'expired'),({'kind':'verify','nonce':'n'},99,'expired')])
def test_early_verify_negative(monkeypatch,rpc_msg,now,match):
 x,reads,sent=make_bridge(monkeypatch,[rpc_msg]);monkeypatch.setattr(b.time,'monotonic_ns',lambda:now)
 with pytest.raises(ValueError,match=match):x.verify_before_publication()
 assert not reads and not sent

def test_early_verify_replay_rejected(monkeypatch):
 x,reads,sent=make_bridge(monkeypatch,[{'kind':'verify','nonce':'n'},{'kind':'verify','nonce':'n'}]);x.verify_before_publication()
 with pytest.raises(ValueError,match='replay'):x.collect(0)
 assert len(reads)==1
