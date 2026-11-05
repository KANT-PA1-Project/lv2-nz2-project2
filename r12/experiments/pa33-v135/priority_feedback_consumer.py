"""Dedicated local feedback diagnostic consumer, no ROS/image/actuator work."""
import argparse,hashlib,json,os,socket,sys,time
from pathlib import Path
root=Path(__file__).resolve().parent;sys.path.insert(0,str(root/'src'))
from arm_practice.native_rgbd.priority_feedback import PriorityFeedbackReceiver
from arm_practice.native_rgbd.trusted_capture_ipc import send_packet,peer_credentials,require_peer
from arm_practice.native_rgbd.renderer_epoch_issuer import canonical
p=argparse.ArgumentParser();p.add_argument('--socket');p.add_argument('--producer-pid',type=int);p.add_argument('--run-id');p.add_argument('--source-sha');p.add_argument('--stream-id');p.add_argument('--out');a=p.parse_args()
s=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);s.settimeout(30);s.connect(a.socket);require_peer(peer_credentials(s),a.producer_pid,os.getuid())
r=PriorityFeedbackReceiver(s,producer_pid=a.producer_pid,uid=os.getuid(),run_id=a.run_id,source_sha256=a.source_sha,stream_id=a.stream_id);out=Path(a.out);records=[]
send_packet(s,dict(kind='priority_ready',run_id=a.run_id,stream_id=a.stream_id))
while True:
    try:
        handle=r.receive();data=r.pending[handle];before=hashlib.sha256(canonical(data['feedback'])).hexdigest();result=r.consume(handle);after=hashlib.sha256(canonical(result['feedback'])).hexdigest()
        start=data['feedback']['velocity']['read_start_ns'];response=dict(kind='feedback_accepted',sequence=data['sequence'],physics_step=data['feedback']['position']['physics_step'],received_at_ns=result['received_at_ns'],consumed_at_ns=result['evaluated_at_ns'],receive_age_ms=(result['received_at_ns']-start)/1e6,consume_age_ms=(result['evaluated_at_ns']-start)/1e6,feedback_sha256=after,hash_unchanged=before==after,motion_authorized=False)
    except ValueError as error:
        if str(error)=='peer_closed':break
        response=dict(kind='feedback_denied',reason=str(error),motion_authorized=False)
    records.append(response);(out/'priority_feedback_results.json').write_text(json.dumps(records,indent=2));send_packet(s,response)
s.close()
