"""Bounded same-host verifier RPC. No issuer key crosses the process boundary.

The trusted producer launches one pinned consumer and authenticates Linux socket
peer credentials. The consumer authenticates the producer PID/UID in return.
This protects against accidental cross-run/cross-process mixups, not a malicious
same-user debugger or root. DDS GID is a routing identity, not authentication.
"""
from pathlib import Path
import hashlib
import json
import os
import secrets
import socket
import struct
import subprocess
import tempfile
import time

import numpy as np
from .renderer_epoch_issuer import canonical, digest_image, digest_depth

from .mixed_camera_schema import WIRE_V1,WIRE_V2,MixedCameraSchema

MAX_HEADER = 262144
MAX_BYTES = 16 * 1024 * 1024


def exact_read(sock, size):
    parts = []
    while size:
        part = sock.recv(size)
        if not part:
            raise ValueError('peer_closed')
        parts.append(part)
        size -= len(part)
    return b''.join(parts)


def send_packet(sock, data, blobs=(), *, wire_version=WIRE_V1, camera_schema=None, timing=None):
    if timing is not None:timing["serialization_start_ns"]=time.monotonic_ns()
    limit=wire_limit(wire_version,camera_schema)
    lengths = [len(b) for b in blobs]
    if len(lengths) > 12 or sum(lengths) > limit:
        raise ValueError('oversize_payload')
    if camera_schema is not None:camera_schema.validate_wire(data,lengths)
    header = canonical(dict(data, transport_version=wire_version, blob_lengths=lengths))
    if len(header) > MAX_HEADER:
        raise ValueError('oversize_header')
    if timing is not None:timing['serialization_end_ns']=time.monotonic_ns();timing['header_bytes']=len(header);timing['blob_bytes']=sum(lengths);timing['unix_send_start_ns']=time.monotonic_ns()
    sock.sendall(struct.pack('!I', len(header)) + header)
    for blob in blobs:
        sock.sendall(blob)
    if timing is not None:timing['unix_send_end_ns']=time.monotonic_ns()


def wire_limit(version,schema):
    if version==WIRE_V1 and schema is None:return MAX_BYTES
    if version==WIRE_V2 and isinstance(schema,MixedCameraSchema):return 48*1024*1024
    raise ValueError('unsupported_wire_profile')


def receive_packet(sock, *, wire_version=WIRE_V1, camera_schema=None, allow_control_finish=False):
    limit=wire_limit(wire_version,camera_schema)
    size = struct.unpack('!I', exact_read(sock, 4))[0]
    if not 0 < size <= MAX_HEADER:
        raise ValueError('invalid_header_length')
    data = json.loads(exact_read(sock, size))
    actual_wire=data.pop('transport_version',WIRE_V1)
    lengths = data.pop('blob_lengths')
    if data.get('kind')=='finish' and (actual_wire!=WIRE_V1 or lengths!=[]):
        raise ValueError('invalid_terminal_control_profile')
    terminal_control=allow_control_finish and actual_wire==WIRE_V1 and data.get('kind')=='finish' and lengths==[]
    if actual_wire!=wire_version and not terminal_control:raise ValueError('wire_profile_mismatch')
    if (not isinstance(lengths, list) or len(lengths) > 12 or
            any(type(n) is not int or n < 0 for n in lengths) or
            sum(lengths) > limit):
        raise ValueError('invalid_blob_lengths')
    if camera_schema is not None:camera_schema.validate_wire(data,lengths)
    return data, [exact_read(sock, n) for n in lengths]


def peer_credentials(sock):
    return struct.unpack('3i', sock.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12))


def require_peer(peer, expected_pid, expected_uid):
    if peer[0] != expected_pid or peer[1] != expected_uid:
        raise ValueError('untrusted_peer_pid_or_uid')


def valid_gid(gid):
    return (type(gid) is str and len(gid) in (32, 48, 64) and
            all(c in '0123456789abcdef' for c in gid) and any(c != '0' for c in gid))


def make_request(sealed, pixels, depths, *, nonce, gid, camera_schema=None):
    body = sealed['body']
    if camera_schema:camera_schema.verify_body(body)
    elif body.get('wire_schema') is not None:raise ValueError('explicit_mixed_schema_required')
    result = {
        'kind': 'verify', 'nonce': nonce, 'run_id': body['run_id'],
        'clock_epoch': body['clock_epoch'],
        'capture_physics_step': body['capture_physics_step'],
        'source_sha256': body['source_sha256'], 'model_sha256': body['model_sha256'],
        'certificate_sha256': hashlib.sha256(canonical(sealed)).hexdigest(),
        'publisher_gid': gid, 'camera_names': sorted(pixels),
        'depth_camera_map': body['depth_camera_map'],
        'pixel_sha256': {k: camera_schema.digest(v,k,'rgb') if camera_schema else digest_image(v) for k,v in pixels.items()},
        'depth_sha256': {k: camera_schema.digest(v,k,'depth') if camera_schema else digest_depth(v) for k,v in depths.items()},
    }

    if camera_schema:result.update(wire_schema=camera_schema.wire_schema,camera_schema_sha256=camera_schema.sha256,calibration_sha256=camera_schema.calibration_sha256)
    return result

def request_matches(request, expected):
    if request != expected:
        differing = sorted(k for k in set(request) | set(expected)
                           if request.get(k) != expected.get(k))
        raise ValueError('request_binding_mismatch:' + ','.join(differing))


class TrustedCaptureChannel:
    def __init__(self, issuer, source, out, *, domain_id, read_local_feedback):
        self.out = Path(out)
        self.child = self.rogue = self.connection = self.listener = None
        self.log = self.rogue_log = self.path = self.temp = None
        if (type(domain_id) is not int or not 0 <= domain_id <= 232 or
                os.environ.get('ROS_DOMAIN_ID') != str(domain_id)):
            raise ValueError('allocation_domain_mismatch')
        if not callable(read_local_feedback):
            raise ValueError('local_feedback_reader_required')
        self.domain_id, self.read_local_feedback = domain_id, read_local_feedback
        self.recent_requests = {}
        self.feedback_sequence = 0
        self.feedback_stream_id = secrets.token_hex(16)
        try:
            self._initialize(issuer, source, out)
        except BaseException as error:
            cleanup = self._release_resources()
            self.write('ipc_initialization_failure.json', {'type': type(error).__name__,
                       'message': str(error), 'cleanup_errors': cleanup,
                       'owned_child_exit': self.child.returncode if self.child else None,
                       'owned_rogue_exit': self.rogue.returncode if self.rogue else None})
            raise

    def _initialize(self, issuer, source, out):
        self.issuer = issuer
        self.camera_schema=issuer.camera_schema
        self.wire_schema=self.camera_schema.wire_schema if self.camera_schema else WIRE_V1
        self.out = Path(out)
        self.records = []
        self.child = None
        self.connection = None
        self.temp = Path(tempfile.mkdtemp(prefix='r12-rgbd-'))
        os.chmod(self.temp, 0o700)
        self.path = self.temp / 'verify.sock'
        self.listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.listener.bind(str(self.path))
        os.chmod(self.path, 0o600)
        self.listener.listen(2)
        self.listener.settimeout(12)
        env = os.environ.copy()
        for key in ['PYTHONPATH', 'LD_LIBRARY_PATH', 'AMENT_PREFIX_PATH',
                    'COLCON_PREFIX_PATH', 'ROS_DISTRO']:
            env.pop(key, None)
        env.update(ROS_DOMAIN_ID=str(self.domain_id), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
        # A separate unexpected PID must not acquire the verifier connection.
        self.rogue_log = (self.out / 'ipc_rogue.log').open('w')
        self.rogue = subprocess.Popen(
            ['/usr/bin/python3', str(Path(source) / 'rgbd_consumer.py'),
             '--socket', str(self.path), '--rogue'], env=env,
            stdout=self.rogue_log, stderr=subprocess.STDOUT)
        rogue_connection, _ = self.listener.accept()
        rogue_connection.settimeout(60)
        rogue_peer = peer_credentials(rogue_connection)
        require_peer(rogue_peer, self.rogue.pid, os.getuid())
        send_packet(rogue_connection, {'kind': 'denied', 'reason': 'unregistered_consumer_pid'})
        rogue_connection.close()
        self.rejected_peers = [{'pid': rogue_peer[0], 'uid': rogue_peer[1],
                                'reason': 'unregistered_consumer_pid'}]
        self.log = (self.out / 'ipc_consumer.log').open('w')
        command = ['bash', '-c', 'source /opt/ros/lyrical/setup.bash; exec /usr/bin/python3 "$@"',
                   'r12-consumer', str(Path(source) / 'rgbd_consumer.py'),
                   '--socket', str(self.path), '--producer-pid', str(os.getpid()),
                   '--run-id', issuer.run_id, '--domain-id', str(self.domain_id), '--source-hash', issuer.source,
                   '--model-hash', issuer.model, '--output', str(self.out)]
        if self.camera_schema:command+=['--camera-schema-file',str(Path(source)/'configs/mixed_camera_schema.json'),'--wire-schema',self.wire_schema]
        self.child = subprocess.Popen(command, env=env, stdout=self.log,
                                      stderr=subprocess.STDOUT)
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            connection, _ = self.listener.accept()
            connection.settimeout(60)
            peer = peer_credentials(connection)
            try:
                require_peer(peer, self.child.pid, os.getuid())
            except ValueError as error:
                self.rejected_peers.append({'pid': peer[0], 'uid': peer[1], 'reason': str(error)})
                send_packet(connection, {'kind': 'denied', 'reason': str(error)})
                connection.close()
                continue
            self.connection = connection
            break
        if self.connection is None:
            raise RuntimeError('trusted_consumer_not_connected')
        hello, blobs = receive_packet(self.connection)
        if (blobs or hello.get('kind') != 'hello' or hello.get('run_id') != issuer.run_id or
                hello.get('ros_domain_id') != self.domain_id or not valid_gid(hello.get('publisher_gid'))):
            raise ValueError('consumer_handshake_invalid')
        if self.camera_schema and (hello.get('wire_schema')!=self.wire_schema or hello.get('camera_schema_sha256')!=self.camera_schema.sha256):raise ValueError('mixed_handshake_schema_mismatch')
        self.gid = hello['publisher_gid']
        self.hello = hello
        self.peer = peer
        send_packet(self.connection, {'kind': 'ready', 'run_id': issuer.run_id,
                                     'publisher_gid': self.gid, 'feedback_stream_id': self.feedback_stream_id,'wire_schema':self.wire_schema,'camera_schema_sha256':self.camera_schema.sha256 if self.camera_schema else None})
        self.write('ipc_session.json', {
            'producer_pid': os.getpid(), 'consumer_pid': self.child.pid,
            'consumer_peer_credentials': peer, 'consumer_hello': hello,
            'socket_mode': oct(self.path.stat().st_mode & 0o777),
            'directory_mode': oct(self.temp.stat().st_mode & 0o777),
            'rejected_peers': self.rejected_peers, 'key_sent_or_persisted': False,
            'scope': 'Same-host capture verifier RPC and inactive PickSensorAdapter input boundary',
            'allocation_domain_id': self.domain_id, 'feedback_stream_id': self.feedback_stream_id,
        })

    def write(self, name, data):
        path = self.out / name
        temp = path.with_suffix('.tmp')
        temp.write_text(json.dumps(data, indent=2))
        temp.replace(path)

    def transfer(self, sealed, pixels, depths, fault=None):
        nonce = secrets.token_hex(16)  # public one-request correlation, not the issuer key
        descriptors = []
        blobs = []
        for kind, arrays in [('rgb', pixels), ('depth', depths)]:
            for name, array in sorted(arrays.items()):
                descriptors.append({'kind': kind, 'name': name, 'dtype': str(array.dtype),
                                    'shape': list(array.shape)})
                blobs.append(array.tobytes(order='C'))
        expected = make_request(sealed, pixels, depths, nonce=nonce, gid=self.gid,camera_schema=self.camera_schema)
        start = time.monotonic_ns()
        self._register_request(expected, sealed['body']['capture_monotonic_lower_bound_ns'], start)
        send_packet(self.connection, {'kind': 'capture', 'certificate': sealed,
                                     'nonce': nonce, 'arrays': descriptors,
                                     'fault_case': fault}, blobs,wire_version=self.wire_schema,camera_schema=self.camera_schema)
        responses = []
        for _ in range(20):
            request, extra = receive_packet(self.connection)
            if extra:
                raise ValueError('unexpected_request_blobs')
            if request.get('kind') == 'done':
                if request.get('nonce') != nonce:
                    raise ValueError('done_nonce_mismatch')
                record = {'capture_step': sealed['body']['capture_physics_step'],
                          'fault_case': fault, 'responses': responses,
                          'consumer_report': request, 'ipc_roundtrip_ms':
                          (time.monotonic_ns() - start) / 1e6}
                self.records.append(record)
                self.write('ipc_records.json', self.records)
                return record
            if request.get('kind') == 'refresh_local_feedback':
                require_peer(peer_credentials(self.connection), self.child.pid, os.getuid())
                send_packet(self.connection, self._refresh_response(request,time.monotonic_ns()))
                continue
            try:
                require_peer(peer_credentials(self.connection), self.child.pid, os.getuid())
                request_matches(request, expected)
                verdict = self.issuer.verify(sealed, pixels=pixels, depths=depths,
                                             now_ns=time.monotonic_ns(),
                                             expected_current_run=self.issuer.run_id)
            except ValueError as error:
                verdict = {'matched': False, 'image_capture_association_verified': False,
                           'rgbd_capture_association_verified': False,
                           'motion_authorized': False, 'reason': str(error)}
            response = {'kind': 'verdict', 'nonce': nonce,
                        'request_sha256': hashlib.sha256(canonical(request)).hexdigest(),
                        'publisher_gid': self.gid, 'verdict': verdict}
            if verdict['matched']:
                response['local_feedback'] = self._current_feedback()
            send_packet(self.connection, response)
            responses.append({'request_diff': [k for k in expected if request.get(k) != expected[k]],
                              'matched': verdict['matched'], 'reason': verdict['reason'],
                              'consumer_age_ms': (time.monotonic_ns() - sealed['body']['capture_monotonic_lower_bound_ns']) / 1e6})
        raise ValueError('too_many_verification_requests')

    def _register_request(self, expected, capture_ns, now):
        if any(type(x) is not int for x in [capture_ns,now]) or not 0 <= capture_ns <= now:
            raise ValueError('invalid_recent_request_clock')
        self.recent_requests = {k:v for k,v in self.recent_requests.items()
                                if now-v['capture_ns'] <= 500_000_000}
        if expected['nonce'] in self.recent_requests:raise ValueError('recent_request_nonce_reuse')
        if len(self.recent_requests) >= 32:raise ValueError('producer_pending_request_limit')
        self.recent_requests[expected['nonce']] = {'request': expected.copy(), 'capture_ns': capture_ns}

    def _validate_refresh_request(self, request, now):
        recent = self.recent_requests.get(request.get('nonce'))
        if recent is None or not 0 <= now-recent['capture_ns'] <= 500_000_000:
            raise ValueError('refresh_capture_session_missing_or_expired')
        request_matches(request, dict(recent['request'], kind='refresh_local_feedback'))

    def _refresh_response(self, request, now):
        if type(now) is not int or now<0:raise ValueError('invalid_refresh_clock')
        if not isinstance(request,dict) or request.get('kind')!='refresh_local_feedback' or type(request.get('nonce')) is not str or not request['nonce']:
            raise ValueError('invalid_refresh_request')
        binding={'nonce':request['nonce'],'request_sha256':hashlib.sha256(canonical(request)).hexdigest(),'publisher_gid':self.gid}
        try:
            self._validate_refresh_request(request,now)
        except ValueError as error:
            if str(error)!='refresh_capture_session_missing_or_expired':raise
            self.recent_requests.pop(request['nonce'],None)
            return dict(binding,kind='local_feedback_denied',denial={'reason':str(error),'discard_capture':True,'motion_authorized':False})
        return dict(binding,kind='local_feedback',local_feedback=self._current_feedback())

    def _finish_packet(self):
        return {'kind':'finish','run_id':self.issuer.run_id,'clock_epoch':self.issuer.epoch,'source_sha256':self.issuer.source,'model_sha256':self.issuer.model,'publisher_gid':self.gid,'motion_authorized':False}

    def _current_feedback(self):
        feedback = self.read_local_feedback()
        self.feedback_sequence += 1
        return dict(feedback, sequence=self.feedback_sequence, identity={
            'run_id': self.issuer.run_id, 'clock_epoch': self.issuer.epoch,
            'source_sha256': self.issuer.source, 'model_sha256': self.issuer.model,
            'producer_pid': os.getpid(), 'stream_id': self.feedback_stream_id,
            'kind': 'current_local_actuator_encoder',
            'joint_names': list(self.issuer.joints)})

    def _release_resources(self):
        errors = []
        for process in [self.child, self.rogue]:
            if process is not None and process.poll() is None:
                try:
                    process.terminate()
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=3)
                except Exception as error:
                    errors.append('owned_child:' + str(error))
        for resource in [self.connection, self.listener, self.log, self.rogue_log]:
            if resource is not None:
                try:
                    resource.close()
                except Exception as error:
                    errors.append('close:' + str(error))
        for path, method in [(self.path, 'unlink'), (self.temp, 'rmdir')]:
            if path is not None:
                try:
                    if path.exists():
                        getattr(path, method)()
                except Exception as error:
                    errors.append('remove:' + str(error))
        return errors

    def close(self):
        failure = None
        try:
            if self.connection:
                send_packet(self.connection, self._finish_packet())
            if self.child:
                self.child.wait(timeout=10)
            if self.rogue:
                self.rogue.wait(timeout=5)
        except Exception as error:
            failure = str(error)
        finally:
            cleanup = self._release_resources()
            self.write('ipc_shutdown.json', {'consumer_exit': self.child.returncode,
                'rogue_exit': self.rogue.returncode, 'error': failure,
                'cleanup_errors': cleanup, 'socket_removed': not self.path.exists()})
        if failure or cleanup or self.child.returncode or self.rogue.returncode:
            raise RuntimeError('IPC consumer did not complete cleanly')
