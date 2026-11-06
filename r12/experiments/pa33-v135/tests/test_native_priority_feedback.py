import copy,json,os,socket
from pathlib import Path
import pytest
from arm_practice.native_rgbd.priority_feedback import PriorityFeedbackReceiver
from arm_practice.native_rgbd.trusted_capture_ipc import send_packet

@pytest.fixture
def channel(monkeypatch):
    rows=json.loads((Path(__file__).resolve().parent/'data/v54_publication_replay.json').read_text());feedback=rows[0]['packet']['current_feedback'];clock=[feedback['velocity']['read_start_ns']+20_000_000]
    monkeypatch.setattr('arm_practice.native_rgbd.priority_feedback.time.monotonic_ns',lambda:clock[0])
    a,b=socket.socketpair();a.settimeout(1);b.settimeout(1);identity=dict(run_id='test',source_sha256='1'*64,stream_id='owned')
    receiver=PriorityFeedbackReceiver(b,producer_pid=os.getpid(),uid=os.getuid(),**identity)
    payload=dict(kind='current_feedback',identity=identity,sequence=1,feedback=copy.deepcopy(feedback))
    yield a,receiver,payload,clock
    a.close();b.close()

def test_actual_feedback_receipt_without_RGB(channel):
    a,r,p,clock=channel;send_packet(a,p);h=r.receive();out=r.consume(h)
    assert out['feedback']==p['feedback'] and out['feedback_fresh'] and not out['motion_authorized']
    with pytest.raises(ValueError):r.consume(h)

def test_original_time_expiry_at_consume(channel):
    a,r,p,clock=channel;send_packet(a,p);h=r.receive();clock[0]+=90_000_000
    with pytest.raises(ValueError,match='100ms'):r.consume(h)
    assert not r.pending

@pytest.mark.parametrize('kind',['old','future','identity','epoch','nan','duplicate','blob','large','public_handle','wrong_peer'])
def test_reject_invalid_feedback(channel,kind):
    a,r,p,clock=channel
    if kind=='old':clock[0]+=90_000_000
    if kind=='future':p['feedback']['position']['read_end_ns']=clock[0]+1
    if kind=='identity':p['identity']['run_id']='other'
    if kind=='epoch':p['feedback']['position']['physics_step']-=8
    if kind=='nan':p['feedback']['velocity']['velocity'][0]=float('nan')
    if kind=='duplicate':send_packet(a,p);r.consume(r.receive())
    if kind=='large':p['padding']='x'*17000
    if kind=='public_handle':
        with pytest.raises(ValueError):r.consume(object())
        return
    if kind=='wrong_peer':r.pid+=1
    if kind=='nan':
        # Wire canonical serializer rejects NaN before the receiver.
        with pytest.raises(ValueError):send_packet(a,p)
        return
    send_packet(a,p,[b'x']if kind=='blob'else [])
    with pytest.raises(ValueError):r.receive()


@pytest.mark.parametrize('kind',['float_velocity_step','negative_step','bool_step','truthy_velocity_valid','bool_position','bool_velocity','q_mismatch','negative_sequence','bool_sequence'])
def test_strict_feedback_contract(channel,kind):
    a,r,p,clock=channel;f=p['feedback']
    if kind=='float_velocity_step':f['velocity']['step']=float(f['velocity']['step'])
    if kind=='negative_step':f['velocity']['step']=f['position']['physics_step']=-1
    if kind=='bool_step':f['velocity']['step']=f['position']['physics_step']=True
    if kind=='truthy_velocity_valid':f['velocity']['velocity_valid']='false'
    if kind=='bool_position':f['position']['q'][0]=False
    if kind=='bool_velocity':f['velocity']['velocity'][0]=False
    if kind=='q_mismatch':f['position']['q'][0]+=.001
    if kind=='negative_sequence':p['sequence']=-1
    if kind=='bool_sequence':p['sequence']=True
    send_packet(a,p)
    with pytest.raises(ValueError):r.receive()
    assert not r.pending and r.last_sequence==-1
