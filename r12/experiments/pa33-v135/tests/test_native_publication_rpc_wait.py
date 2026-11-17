from types import SimpleNamespace
import pytest
from test_native_shadow_rpc import make_bridge
import arm_practice.native_rgbd.live_decode_bridge as b
from arm_practice.native_rgbd.publication_rpc_wait import wait_publication

class Socket:
 def __init__(self):self.timeout=30
 def gettimeout(self):return self.timeout
 def settimeout(self,x):self.timeout=x

def test_queue_refresh_before_ros_and_real_read(monkeypatch):
 x,reads,sent=make_bridge(monkeypatch,[{'kind':'verify','nonce':'n'},{'kind':'refresh_local_feedback','nonce':'n'},{'kind':'decoded','frame':0}]);x.conn=Socket();x.verify_before_publication();ros=object();queue=iter([[x.conn],[ros]]);seen=[]
 def wait(s,*args):seen.append(list(s));return next(queue),[],[]
 wait_publication(ros,x,100,clock=lambda:300,wait=wait)
 assert len(reads)==2 and x.refreshed_once and x.conn.timeout==30
 assert seen[1]==[ros] and sent[-1]['nonce']=='n' and sent[-1]['local_feedback']['read_start_ns']==200
 assert x.collect(0)['producer_transport_timing']['refresh_service']['response_send_end_ns']==300

@pytest.mark.parametrize('msg,now,reason',[({'kind':'refresh_local_feedback','nonce':'wrong'},300,'binding'),({'kind':'verify','nonce':'n'},300,'replay'),({'kind':'refresh_local_feedback','nonce':'n'},500000101,'expired'),({'kind':'refresh_local_feedback','nonce':'n'},99,'expired')])
def test_bad_refresh_no_read(monkeypatch,msg,now,reason):
 x,reads,sent=make_bridge(monkeypatch,[msg]);x.verified_once=True;monkeypatch.setattr(b.time,'monotonic_ns',lambda:now)
 with pytest.raises(ValueError,match=reason):x.refresh_during_publication()
 assert not reads and not sent

def test_read_crossing_deadline_no_response(monkeypatch):
 x,reads,sent=make_bridge(monkeypatch,[{'kind':'refresh_local_feedback','nonce':'n'}]);x.verified_once=True;t=[300]
 monkeypatch.setattr(b.time,'monotonic_ns',lambda:t[0])
 def read():reads.append(1);t[0]=500000101;return {'q':[0]*5}
 x.read_feedback=read
 with pytest.raises(ValueError,match='expired'):x.refresh_during_publication()
 assert reads==[1] and not sent

def test_duplicate_refresh_rejected(monkeypatch):
 x,reads,sent=make_bridge(monkeypatch,[{'kind':'refresh_local_feedback','nonce':'n'}]);x.verified_once=True;x.refresh_during_publication()
 with pytest.raises(ValueError,match='replay'):x.refresh_during_publication()
 assert len(reads)==1

@pytest.mark.parametrize('now,ready',[(500000101,True),(99,True),(300,False)])
def test_wait_deadline(monkeypatch,now,ready):
 ros=object();x=SimpleNamespace(conn=Socket(),refreshed_once=False)
 with pytest.raises(ValueError,match='expired'):wait_publication(ros,x,100,clock=lambda:now,wait=lambda *a:([ros] if ready else [],[],[]))

def test_simultaneous_refresh_first(monkeypatch):
 ros=object();calls=[];x=SimpleNamespace(conn=Socket(),refreshed_once=False)
 x.refresh_during_publication=lambda:calls.append('refresh')
 wait_publication(ros,x,100,clock=lambda:300,wait=lambda *a:([ros,x.conn],[],[]))
 assert calls==['refresh']

@pytest.mark.parametrize('failure',['peer','blob'])
def test_refresh_transport_rejected(monkeypatch,failure):
 x,reads,sent=make_bridge(monkeypatch,[{'kind':'refresh_local_feedback','nonce':'n'}]);x.verified_once=True
 if failure=='peer':
  def reject(*a):raise ValueError('peer')
  monkeypatch.setattr(b,'require_peer',reject)
 else:monkeypatch.setattr(b,'receive_packet',lambda sock:({'kind':'refresh_local_feedback','nonce':'n'},[b'x']))
 with pytest.raises(ValueError):x.refresh_during_publication()
 assert not reads and not sent

def test_partial_rpc_timeout_restores_socket(monkeypatch):
 ros=object();x=SimpleNamespace(conn=Socket(),refreshed_once=False)
 def fail():raise TimeoutError('partial packet')
 x.refresh_during_publication=fail
 with pytest.raises(TimeoutError):wait_publication(ros,x,100,clock=lambda:300,wait=lambda *a:([x.conn],[],[]))
 assert x.conn.timeout==30

def test_ros_publish_services_before_receiving_ack(monkeypatch):
 import arm_practice.native_rgbd.live_publication_bridge as pub
 import arm_practice.native_rgbd.publication_rpc_wait as scheduler
 bridge=pub.LivePublicationBridge.__new__(pub.LivePublicationBridge);bridge.connection=object();bridge.run_id='r';bridge.schema=SimpleNamespace(wire_schema=2)
 order=[];decoder=object()
 monkeypatch.setattr(pub,'send_packet',lambda *a,**k:order.append('send'))
 monkeypatch.setattr(scheduler,'wait_publication',lambda ros,d,lower:order.append(('service',d,lower)))
 monkeypatch.setattr(pub,'receive_packet',lambda sock:(order.append('ack')or {'kind':'publication_result','all_received':True},[]))
 bridge.publish({'body':{'capture_monotonic_lower_bound_ns':100}},{},{},{},{'rgbd_capture_association_verified':True},decoder=decoder)
 assert order==['send',('service',decoder,100),'ack']


def test_early_rejection_terminal_never_reads_feedback(monkeypatch):
 rejection={'kind':'decoded','frame':0,'live_shadow':{'rejected':'missing_or_ambiguous_geometric_pair','adapter_ready':False,'motion_authorized':False}}
 x,reads,sent=make_bridge(monkeypatch,[rejection]);x.verified_once=True;x.conn=Socket();ros=object();queue=iter([[x.conn],[ros]]);seen=[]
 def wait(s,*args):seen.append(list(s));return next(queue),[],[]
 wait_publication(ros,x,100,clock=lambda:300,wait=wait)
 assert seen[1]==[ros]and not reads and not sent and not x.refreshed_once
 assert x.collect(0)['live_shadow']['rejected']=='missing_or_ambiguous_geometric_pair'and x.pending is None

@pytest.mark.parametrize('change',[{'frame':9},{'live_shadow':{}},{'live_shadow':{'rejected':'x','adapter_ready':True,'motion_authorized':False}},{'live_shadow':{'rejected':'x','adapter_ready':False,'motion_authorized':True}}])
def test_invalid_early_terminal_rejected(monkeypatch,change):
 msg={'kind':'decoded','frame':0,'live_shadow':{'rejected':'x','adapter_ready':False,'motion_authorized':False}};msg.update(change)
 x,reads,sent=make_bridge(monkeypatch,[msg]);x.verified_once=True
 with pytest.raises(ValueError,match='terminal'):x.refresh_during_publication()
 assert not reads and not sent
