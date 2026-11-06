"""Owned dedicated small Unix channel; image authentication stays separate."""
import atexit,hashlib,json,os,secrets,socket,subprocess,tempfile,time
from pathlib import Path
from .trusted_capture_ipc import send_packet,receive_packet,peer_credentials,require_peer
from .renderer_epoch_issuer import canonical
from .runtime_settings import RuntimeSettings
class PriorityFeedbackBridge:
    def __init__(self,root,out,run_id,source_sha,*,settings=None):
        settings=settings or RuntimeSettings.from_env();python=settings.path("ros_python",executable=True)
        self.out=Path(out);self.sequence=0;self.closed=False;self.stream=secrets.token_hex(16);self.run=run_id;self.source=source_sha
        self.temp=tempfile.TemporaryDirectory(prefix='r12-priority-');path=str(Path(self.temp.name)/'feedback.sock');self.listener=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);self.listener.bind(path);os.chmod(path,0o600);self.listener.listen(1);self.listener.settimeout(15)
        self.log=(self.out/'priority_feedback_consumer.log').open('w');env=settings.child_env()
        for key in ['PYTHONPATH','LD_LIBRARY_PATH','AMENT_PREFIX_PATH','COLCON_PREFIX_PATH','ROS_DISTRO']:env.pop(key,None)
        self.child=subprocess.Popen([str(python),str(Path(root)/'priority_feedback_consumer.py'),'--socket',path,'--producer-pid',str(os.getpid()),'--run-id',run_id,'--source-sha',source_sha,'--stream-id',self.stream,'--out',str(out)],env=env,stdout=self.log,stderr=subprocess.STDOUT)
        self.sock,_=self.listener.accept();self.sock.settimeout(15);peer=peer_credentials(self.sock);require_peer(peer,self.child.pid,os.getuid());hello,blobs=receive_packet(self.sock)
        if blobs or hello!=dict(kind='priority_ready',run_id=run_id,stream_id=self.stream):raise ValueError('priority_handshake_invalid')
        atexit.register(self.close);(self.out/'priority_feedback_session.json').write_text(json.dumps(dict(producer_pid=os.getpid(),consumer_pid=self.child.pid,peer=list(peer),run_id=run_id,stream_id=self.stream)))
    def send(self,feedback):
        self.sequence+=1;data=dict(kind='current_feedback',identity=dict(run_id=self.run,source_sha256=self.source,stream_id=self.stream),sequence=self.sequence,feedback=feedback)
        before=hashlib.sha256(canonical(feedback)).hexdigest();start=time.monotonic_ns();send_packet(self.sock,data);result,blobs=receive_packet(self.sock);end=time.monotonic_ns()
        if blobs or result.get('kind')!='feedback_accepted' or result['sequence']!=self.sequence or result['feedback_sha256']!=before or not result['hash_unchanged']:raise ValueError('priority_feedback_ack_failed:'+str(result))
        if before!=hashlib.sha256(canonical(feedback)).hexdigest():raise ValueError('feedback_changed_during_send')
        result.update(send_start_ns=start,ack_received_ns=end,roundtrip_ms=(end-start)/1e6)
        if self.sequence==1:
            send_packet(self.sock,data);replay,blobs=receive_packet(self.sock)
            if blobs or replay!=dict(kind='feedback_denied',reason='feedback_replay_or_reorder',motion_authorized=False):raise ValueError('priority_replay_not_rejected')
            result['live_replay_rejected']=True
        return result
    def close(self):
        if self.closed:return
        self.closed=True;self.sock.close()
        try:self.child.wait(timeout=5)
        except subprocess.TimeoutExpired:self.child.terminate();self.child.wait(timeout=5)
        (self.out/'priority_feedback_exit.json').write_text(json.dumps(dict(exit_code=self.child.returncode,finished_at=time.time(),pid=self.child.pid)))
        self.listener.close();self.log.close();self.temp.cleanup()
