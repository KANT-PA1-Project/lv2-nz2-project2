"""Pinned same-host native publication diagnostic; issuer key stays in producer."""
import atexit
import hashlib
import json
import os
from pathlib import Path
import socket
import subprocess
import tempfile
import time
import select
from .trusted_capture_ipc import receive_packet,peer_credentials,require_peer
from .timed_send import send_packet
from .immutable_blobs import immutable_blob
from .runtime_settings import RuntimeSettings
from .ack_metrics import capture_identity, make_ack_record

class LivePublicationBridge:
    def __init__(self,root,out,run_id,schema,*,source_sha,settings=None):
        if type(source_sha)is not str or len(source_sha)!=64 or any(c not in '0123456789abcdef' for c in source_sha):raise ValueError('invalid_source_sha256')
        settings=settings or RuntimeSettings.from_env();self.domain,python,setup=settings.ros()
        self.source_sha=source_sha;self.bundle_sha=hashlib.sha256((Path(root)/'native_entrypoint_manifest.json').read_bytes()).hexdigest()
        self.root=Path(root);self.out=Path(out);self.schema=schema;self.run_id=run_id;self.child=None;self.connection=None;self.closed=False
        self.temp=tempfile.TemporaryDirectory(prefix='r12-native-pub-');self.path=str(Path(self.temp.name)/'publisher.sock')
        self.listener=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);self.listener.bind(self.path);os.chmod(self.path,0o600);self.listener.listen(1);self.listener.settimeout(20)
        self.log=(self.out/'ROS_publication.log').open('w')
        env=settings.child_env()
        for key in ['PYTHONPATH','LD_LIBRARY_PATH','AMENT_PREFIX_PATH','COLCON_PREFIX_PATH','ROS_DISTRO']:env.pop(key,None)
        env.update(ROS_DOMAIN_ID=str(self.domain),ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',ROS_STATIC_PEERS='')
        command=['bash','-c','source "$1" && shift && exec "$@"','native-publisher',str(setup),str(python),str(self.root/'native_ros_publisher.py'),'--socket',self.path,'--producer-pid',str(os.getpid()),'--run-id',run_id,'--source-sha',source_sha,'--output',str(out)]
        self.child=subprocess.Popen(command,env=env,stdout=self.log,stderr=subprocess.STDOUT)
        atexit.register(self.close)
        self.connection,_=self.listener.accept();self.connection.settimeout(20)
        peer=peer_credentials(self.connection);require_peer(peer,self.child.pid,os.getuid())
        hello,blobs=receive_packet(self.connection)
        if blobs or hello!={'kind':'ready','run_id':run_id,'domain':self.domain,'source_sha256':source_sha,'bundle_manifest_sha256':self.bundle_sha}:raise ValueError('invalid_ROS_child_hello')
        (self.out/'ROS_trusted_session.json').write_text(json.dumps(dict(producer_pid=os.getpid(),child_pid=self.child.pid,peer=list(peer),run_id=run_id,domain=self.domain,source_sha256=source_sha,bundle_manifest_sha256=self.bundle_sha,issuer_secret_exported=False)))
    def publish_current(self,feedback,source_sha,sequence):
        if source_sha!=self.source_sha:raise ValueError('ROS_feedback_source_changed')
        data=dict(kind='current_feedback',identity=dict(run_id=self.run_id,source_sha256=source_sha,stream_id='ros-current-'+self.run_id),sequence=sequence,feedback=feedback)
        before=hashlib.sha256(json.dumps(feedback,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
        timing={};send_packet(self.connection,data,wire_version=self.schema.wire_schema,camera_schema=self.schema,timing=timing);result,blobs=receive_packet(self.connection)
        if blobs or result.get('kind')!='current_feedback_published' or result.get('sequence')!=sequence or result.get('feedback_sha256')!=before:raise ValueError('early_ROS_feedback_ack_mismatch')
        if hashlib.sha256(json.dumps(feedback,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()!=before:raise ValueError('early_ROS_feedback_mutation')
        result['producer_transport_timing']=timing
        return result
    def publish(self,certificate,pixels,depths,plan,verification,decoder=None):
        if not verification.get('rgbd_capture_association_verified'):raise ValueError('live_HMAC_verification_required')
        identity=capture_identity(self.run_id,self.source_sha,certificate['body'])
        timing=dict(capture_monotonic_ns=identity['capture_monotonic_ns'],blob_copy_start_ns=time.monotonic_ns())
        descriptors=[self.schema.descriptor(k,'rgb')for k in sorted(pixels)]+[self.schema.descriptor(k,'depth')for k in sorted(depths)]
        blobs=[immutable_blob(pixels[k])for k in sorted(pixels)]+[immutable_blob(depths[k])for k in sorted(depths)]
        timing['blob_copy_end_ns']=time.monotonic_ns()
        timing['blob_preparation']='readonly_owning_array_views_no_copy'
        send_packet(self.connection,dict(kind='capture',run_id=self.run_id,certificate=certificate,arrays=descriptors,plan=plan,live_issuer_verification=verification),blobs,wire_version=self.schema.wire_schema,camera_schema=self.schema,timing=timing)
        if decoder is not None:
            from .publication_rpc_wait import wait_publication
            wait_publication(self.connection,decoder,certificate['body']['capture_monotonic_lower_bound_ns'])
        ack_receive_start=time.monotonic_ns();result,blobs=receive_packet(self.connection);ack_receive_end=time.monotonic_ns()
        if blobs or result.get('kind')!='publication_result' or not result.get('all_received'):raise ValueError('ROS_publication_subscriber_incomplete:'+str(result))
        if result.get('domain')!=self.domain or result.get('capture_stamp_ns')!=identity['capture_stamp_ns']:raise ValueError('ROS_ACK_capture_or_domain_mismatch')
        timing.update(ack_receive_start_ns=ack_receive_start,ack_receive_end_ns=ack_receive_end,ack_validation_end_ns=time.monotonic_ns())
        record=make_ack_record(identity,result,timing)
        # Persist after the measured endpoint and before decoder collection can fail.
        with (self.out/'producer_ack.jsonl').open('a',encoding='utf-8') as journal:
            journal.write(json.dumps(record,sort_keys=True,separators=(',',':'),allow_nan=False)+'\n')
        result['producer_capture_identity']=identity
        result['producer_transport_timing']=timing
        return result
    def coverage_exchange(self, payload, *, expect_rejection=False):
        if self.closed:
            raise ValueError("coverage_bridge_closed")
        require_peer(peer_credentials(self.connection),self.child.pid,os.getuid())
        send_packet(self.connection,payload,wire_version=self.schema.wire_schema,camera_schema=self.schema)
        result, blobs = receive_packet(self.connection)
        if blobs or any(result.get(key)!=payload.get(key) for key in ("nonce","sequence","payload_sha256")):
            raise ValueError("coverage_reply_identity_mismatch")
        if expect_rejection:
            if result.get("kind")!="sensor_coverage_rejected" or result.get("published_samples")!=0:
                raise ValueError("coverage_expected_rejection_missing")
        elif (result.get("kind")!="sensor_coverage_result" or result.get("all_received")is not True
              or result.get("expected")!=12 or result.get("received")!=12
              or result.get("source_sha256")!=self.source_sha):
            raise ValueError("coverage_required_ROS_incomplete:"+str(result))
        return result

    def close(self):
        if self.closed:return
        self.closed=True
        if self.connection:
            try:send_packet(self.connection,dict(kind='finish'));self.connection.close()
            except Exception:pass
        if self.child and self.child.poll()is None:
            try:self.child.wait(timeout=5)
            except subprocess.TimeoutExpired:self.child.terminate();self.child.wait(timeout=5)
        (self.out/'ROS_publisher_exit.json').write_text(json.dumps(dict(exit_code=self.child.returncode,finished_at=time.time(),pid=self.child.pid)))
        self.listener.close();self.log.close();self.temp.cleanup()
