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
from .trusted_capture_ipc import send_packet,receive_packet,peer_credentials,require_peer

class LivePublicationBridge:
    def __init__(self,root,out,run_id,schema):
        if os.environ.get('ROS_DOMAIN_ID')!='33':raise ValueError('pa33_domain_required')
        self.root=Path(root);self.out=Path(out);self.schema=schema;self.run_id=run_id;self.child=None;self.connection=None;self.closed=False
        self.temp=tempfile.TemporaryDirectory(prefix='pa33-native-pub-');self.path=str(Path(self.temp.name)/'publisher.sock')
        self.listener=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);self.listener.bind(self.path);os.chmod(self.path,0o600);self.listener.listen(1);self.listener.settimeout(20)
        self.log=(self.out/'ROS_publication.log').open('w')
        env=os.environ.copy()
        for key in ['PYTHONPATH','LD_LIBRARY_PATH','AMENT_PREFIX_PATH','COLCON_PREFIX_PATH','ROS_DISTRO']:env.pop(key,None)
        env.update(ROS_DOMAIN_ID='33',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',ROS_STATIC_PEERS='')
        command=['bash','-c','source /opt/ros/lyrical/setup.bash; exec /usr/bin/python3 "$@"','native-publisher',str(self.root/'native_ros_publisher.py'),'--socket',self.path,'--producer-pid',str(os.getpid()),'--run-id',run_id,'--output',str(out)]
        self.child=subprocess.Popen(command,env=env,stdout=self.log,stderr=subprocess.STDOUT)
        atexit.register(self.close)
        self.connection,_=self.listener.accept();self.connection.settimeout(20)
        peer=peer_credentials(self.connection);require_peer(peer,self.child.pid,os.getuid())
        hello,blobs=receive_packet(self.connection)
        if blobs or hello!={'kind':'ready','run_id':run_id,'domain':33}:raise ValueError('invalid_ROS_child_hello')
        (self.out/'ROS_trusted_session.json').write_text(json.dumps(dict(producer_pid=os.getpid(),child_pid=self.child.pid,peer=list(peer),run_id=run_id,domain=33,issuer_secret_exported=False)))
    def publish_current(self,feedback,source_sha,sequence):
        data=dict(kind='current_feedback',identity=dict(run_id=self.run_id,source_sha256=source_sha,stream_id='ros-current-'+self.run_id),sequence=sequence,feedback=feedback)
        before=hashlib.sha256(json.dumps(feedback,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
        timing={};send_packet(self.connection,data,wire_version=self.schema.wire_schema,camera_schema=self.schema,timing=timing);result,blobs=receive_packet(self.connection)
        if blobs or result.get('kind')!='current_feedback_published' or result.get('sequence')!=sequence or result.get('feedback_sha256')!=before:raise ValueError('early_ROS_feedback_ack_mismatch')
        if hashlib.sha256(json.dumps(feedback,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()!=before:raise ValueError('early_ROS_feedback_mutation')
        result['producer_transport_timing']=timing
        return result
    def publish(self,certificate,pixels,depths,plan,verification,decoder=None):
        if not verification.get('rgbd_capture_association_verified'):raise ValueError('live_HMAC_verification_required')
        timing=dict(blob_copy_start_ns=time.monotonic_ns())
        descriptors=[self.schema.descriptor(k,'rgb')for k in sorted(pixels)]+[self.schema.descriptor(k,'depth')for k in sorted(depths)]
        blobs=[pixels[k].tobytes()for k in sorted(pixels)]+[depths[k].tobytes()for k in sorted(depths)]
        timing['blob_copy_end_ns']=time.monotonic_ns()
        send_packet(self.connection,dict(kind='capture',run_id=self.run_id,certificate=certificate,arrays=descriptors,plan=plan,live_issuer_verification=verification),blobs,wire_version=self.schema.wire_schema,camera_schema=self.schema,timing=timing)
        if decoder is not None:
            from .publication_rpc_wait import wait_publication
            wait_publication(self.connection,decoder,certificate['body']['capture_monotonic_lower_bound_ns'])
        ack_receive_start=time.monotonic_ns();result,blobs=receive_packet(self.connection);ack_receive_end=time.monotonic_ns()
        if blobs or result.get('kind')!='publication_result' or not result.get('all_received'):raise ValueError('ROS_publication_subscriber_incomplete:'+str(result))
        timing.update(ack_receive_start_ns=ack_receive_start,ack_receive_end_ns=ack_receive_end,ack_validation_end_ns=time.monotonic_ns())
        result['producer_transport_timing']=timing
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
