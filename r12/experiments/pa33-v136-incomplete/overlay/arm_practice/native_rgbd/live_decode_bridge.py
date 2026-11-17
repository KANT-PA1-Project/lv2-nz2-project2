import atexit,json,os,socket,subprocess,tempfile,time,secrets,hashlib
from .renderer_epoch_issuer import canonical
from .trusted_capture_ipc import make_request,request_matches
from pathlib import Path
from .trusted_capture_ipc import receive_packet,require_peer,peer_credentials
from .timed_send import send_packet
from .immutable_blobs import immutable_blob
from .runtime_settings import RuntimeSettings

class LiveDecodeBridge:
    def __init__(self,root,out,run_id,schema,issuer,read_feedback,*,settings=None):
        settings=settings or RuntimeSettings.from_env();paths,_=settings.decoder()
        self.issuer=issuer;self.read_feedback=read_feedback;self.feedback_sequence=0;self.gid=secrets.token_hex(24);self.stream='shadow-local-'+run_id;self.expected=None;self.verified_once=False;self.refreshed_once=False
        self.root=Path(root);self.out=Path(out);self.run_id=run_id;self.schema=schema;self.closed=False;self.child=None;self.conn=None;self.pending=None
        self.temp=tempfile.TemporaryDirectory(prefix='r12-decode-');self.path=str(Path(self.temp.name)/'decode.sock')
        self.listener=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);self.listener.bind(self.path);os.chmod(self.path,0o600);self.listener.listen(1);self.listener.settimeout(60)
        self.log=(self.out/'live_decoder.log').open('w');env=settings.child_env()
        for k in ['PYTHONPATH','LD_LIBRARY_PATH','AMENT_PREFIX_PATH','COLCON_PREFIX_PATH','ROS_DISTRO']:env.pop(k,None)
        env['CUDA_VISIBLE_DEVICES']='0'
        self.child=subprocess.Popen([str(paths['decoder_python']),str(self.root/'native_live_decoder.py'),'--socket',self.path,'--producer-pid',str(os.getpid()),'--run-id',run_id,'--output',str(out)],env=env,stdout=self.log,stderr=subprocess.STDOUT)
        atexit.register(self.close);self.conn,_=self.listener.accept();self.conn.settimeout(30);require_peer(peer_credentials(self.conn),self.child.pid,os.getuid())
        ready,blobs=receive_packet(self.conn)
        if blobs or ready.get('kind')!='ready' or ready.get('run_id')!=run_id or ready.get('gpu_free_MiB',0)<1024:raise ValueError('decoder_not_warmed_or_resource_guard')
        (self.out/'decoder_ready.json').write_text(json.dumps(ready,indent=2))
    def send(self,certificate,pixels,depths,verification,frame):
        if self.pending is not None:raise ValueError('decoder_inflight_limit')
        if verification.get('rgbd_capture_association_verified')is not True:raise ValueError('missing_live_verification')
        self.expected=make_request(certificate,pixels,depths,nonce=secrets.token_hex(16),gid=self.gid,camera_schema=self.schema);self.verification=verification;self.certificate=certificate;self.verified_once=False;self.refreshed_once=False
        t={'blob_copy_start_ns':time.monotonic_ns()}
        desc=[self.schema.descriptor(k,'rgb')for k in sorted(pixels)]+[self.schema.descriptor(k,'depth')for k in sorted(depths)]
        blobs=[immutable_blob(pixels[k])for k in sorted(pixels)]+[immutable_blob(depths[k])for k in sorted(depths)]
        t['blob_copy_end_ns']=time.monotonic_ns()
        t['blob_preparation']='readonly_owning_array_views_no_copy'
        send_packet(self.conn,dict(kind='capture',run_id=self.run_id,frame=frame,certificate=certificate,arrays=desc,live_issuer_verification=verification,receipt_session=dict(nonce=self.expected['nonce'],publisher_gid=self.gid,feedback_stream_id=self.stream,clock_epoch=self.issuer.epoch,source_sha256=self.issuer.source,model_sha256=self.issuer.model)),blobs,wire_version=self.schema.wire_schema,camera_schema=self.schema,timing=t)
        self.pending=(frame,t);self.pending_terminal=None
    def verify_before_publication(self):
        """Service the first bounded RPC before synchronous ROS publication blocks."""
        if self.pending is None or self.verified_once:raise ValueError('shadow_RPC_replay_or_kind')
        start=time.monotonic_ns();result,blobs=receive_packet(self.conn)
        require_peer(peer_credentials(self.conn),self.child.pid,os.getuid())
        if blobs:raise ValueError('shadow_reply_blobs')
        if result.get('kind')!='verify':raise ValueError('shadow_RPC_replay_or_kind')
        request_matches(result,self.expected)
        now=time.monotonic_ns()
        if not 0<=now-self.certificate['body']['capture_monotonic_lower_bound_ns']<=500_000_000:raise ValueError('shadow_RPC_capture_expired')
        self.verified_once=True;local=self.read_feedback();self.feedback_sequence+=1
        local=dict(local,sequence=self.feedback_sequence,identity=dict(run_id=self.run_id,clock_epoch=self.issuer.epoch,source_sha256=self.issuer.source,model_sha256=self.issuer.model,producer_pid=os.getpid(),stream_id=self.stream,kind='current_local_actuator_encoder',joint_names=list(self.issuer.joints)))
        response=dict(kind='verdict',nonce=result['nonce'],request_sha256=hashlib.sha256(canonical(result)).hexdigest(),publisher_gid=self.gid,local_feedback=local,verdict=self.verification)
        send_packet(self.conn,response)
        self.pending[1]['early_verify_start_ns']=start;self.pending[1]['early_verify_end_ns']=time.monotonic_ns()
    def refresh_during_publication(self):
        """Same producer thread; one fresh native read after authenticated request."""
        if self.pending is None or not self.verified_once or self.refreshed_once:raise ValueError('shadow_RPC_replay_or_kind')
        start=time.monotonic_ns();result,blobs=receive_packet(self.conn)
        received=time.monotonic_ns();require_peer(peer_credentials(self.conn),self.child.pid,os.getuid())
        if blobs:raise ValueError('shadow_reply_blobs')
        if result.get('kind')=='decoded':
            rejection=result.get('live_shadow',{})
            if result.get('frame')!=self.pending[0] or not isinstance(rejection,dict) or type(rejection.get('rejected'))is not str or not rejection['rejected'] or rejection.get('adapter_ready')is not False or rejection.get('motion_authorized')is not False:
                raise ValueError('invalid_early_decode_terminal')
            self.pending_terminal=result
            self.pending[1]['rejected_terminal_received_ns']=received
            return
        if result.get('kind')!='refresh_local_feedback':raise ValueError('shadow_RPC_replay_or_kind')
        request_matches(result,dict(self.expected,kind='refresh_local_feedback'))
        lower=self.certificate['body']['capture_monotonic_lower_bound_ns']
        if not 0<=time.monotonic_ns()-lower<=500_000_000:raise ValueError('shadow_RPC_capture_expired')
        self.refreshed_once=True
        read_start=time.monotonic_ns();local=self.read_feedback();read_end=time.monotonic_ns();self.feedback_sequence+=1
        if not 0<=read_end-lower<=500_000_000:raise ValueError('shadow_RPC_capture_expired')
        local=dict(local,sequence=self.feedback_sequence,identity=dict(run_id=self.run_id,clock_epoch=self.issuer.epoch,source_sha256=self.issuer.source,model_sha256=self.issuer.model,producer_pid=os.getpid(),stream_id=self.stream,kind='current_local_actuator_encoder',joint_names=list(self.issuer.joints)))
        response=dict(kind='local_feedback',nonce=result['nonce'],request_sha256=hashlib.sha256(canonical(result)).hexdigest(),publisher_gid=self.gid,local_feedback=local)
        send_start=time.monotonic_ns();send_packet(self.conn,response);send_end=time.monotonic_ns()
        self.pending[1]['refresh_service']=dict(service_start_ns=start,request_received_ns=received,read_start_ns=read_start,read_end_ns=read_end,response_send_start_ns=send_start,response_send_end_ns=send_end)
    def collect(self,frame):
        if self.pending is None or self.pending[0]!=frame:raise ValueError('decode_frame_mismatch')
        terminal=getattr(self,'pending_terminal',None)
        if terminal is not None:
            if terminal.get('frame')!=frame:raise ValueError('invalid_decode_reply')
            terminal['producer_transport_timing']=self.pending[1];self.pending=None;self.pending_terminal=None
            return terminal
        for _ in range(3):
            result,blobs=receive_packet(self.conn)
            require_peer(peer_credentials(self.conn),self.child.pid,os.getuid())
            if blobs:raise ValueError('shadow_reply_blobs')
            if result.get('kind')=='decoded':break
            kind=result.get('kind')
            if kind=='verify' and not self.verified_once:
                request_matches(result,self.expected);self.verified_once=True
            elif kind=='refresh_local_feedback' and self.verified_once and not self.refreshed_once:
                request_matches(result,dict(self.expected,kind=kind));self.refreshed_once=True
            else:raise ValueError('shadow_RPC_replay_or_kind')
            now=time.monotonic_ns();b=self.certificate['body']
            if not 0<=now-b['capture_monotonic_lower_bound_ns']<=500_000_000:raise ValueError('shadow_RPC_capture_expired')
            local=self.read_feedback();self.feedback_sequence+=1
            local=dict(local,sequence=self.feedback_sequence,identity=dict(run_id=self.run_id,clock_epoch=self.issuer.epoch,source_sha256=self.issuer.source,model_sha256=self.issuer.model,producer_pid=os.getpid(),stream_id=self.stream,kind='current_local_actuator_encoder',joint_names=list(self.issuer.joints)))
            response=dict(kind='verdict' if kind=='verify' else 'local_feedback',nonce=result['nonce'],request_sha256=hashlib.sha256(canonical(result)).hexdigest(),publisher_gid=self.gid,local_feedback=local)
            if kind=='verify':response['verdict']=self.verification
            send_packet(self.conn,response)
        else:raise ValueError('shadow_RPC_bound')
        if result.get('kind')!='decoded' or result.get('frame')!=frame:raise ValueError('invalid_decode_reply')
        result['producer_transport_timing']=self.pending[1];self.pending=None;return result
    def close(self):
        if self.closed:return
        self.closed=True
        if self.conn:
            try:send_packet(self.conn,dict(kind='finish'));self.conn.close()
            except Exception:pass
        if self.child and self.child.poll()is None:
            try:self.child.wait(timeout=5)
            except subprocess.TimeoutExpired:self.child.terminate();self.child.wait(timeout=5)
        (self.out/'decoder_exit.json').write_text(json.dumps(dict(exit_code=self.child.returncode if self.child else None,pid=self.child.pid if self.child else None)))
        self.listener.close();self.log.close();self.temp.cleanup()
