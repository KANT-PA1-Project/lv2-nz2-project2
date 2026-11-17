"""Live socket receipts for historical RGBD and independent current feedback.

No public packet flag creates a receipt. Only an authenticated response read from
the pinned producer socket does. Handles are process-local, one-use capabilities;
they are not serializable provenance and never authorize a motion or clearance.
"""
import copy
import hashlib
import time
from fractions import Fraction
import numpy as np
from .renderer_epoch_issuer import canonical, digest_image, digest_depth
from .trusted_capture_ipc import receive_packet, peer_credentials, require_peer

JOINTS = [f'arm_joint{i}' for i in range(1, 6)]


class SeparatedSensorInputs:
    def __init__(self, sock, *, producer_pid, uid, run_id, clock_epoch,
                 source_sha256, model_sha256, publisher_gid, feedback_stream_id,
                 max_pending_captures=32, max_pending_feedback=64,camera_schema=None):
        if (type(max_pending_captures) is not int or not 1 <= max_pending_captures <= 32 or
                type(max_pending_feedback) is not int or not 1 <= max_pending_feedback <= 64):
            raise ValueError('invalid_pending_limits')
        self.camera_schema=camera_schema
        self.max_pending_captures, self.max_pending_feedback = max_pending_captures, max_pending_feedback
        self.sock = sock
        self.pid, self.uid = producer_pid, uid
        self.run, self.clock = run_id, clock_epoch
        self.source, self.model = source_sha256, model_sha256
        self.gid, self.stream = publisher_gid, feedback_stream_id
        self.captures, self.feedback = {}, {}
        self.last_feedback_sequence = -1
        self.session_finished = False

    def _validate_feedback(self, f, now):
        if type(now) is not int or now<0:
            raise ValueError('invalid_current_clock')
        identity = f['identity']
        expected = {'run_id': self.run, 'clock_epoch': self.clock,
                    'source_sha256': self.source, 'model_sha256': self.model,
                    'producer_pid': self.pid, 'stream_id': self.stream,
                    'kind': 'current_local_actuator_encoder', 'joint_names': JOINTS}
        if identity != expected:
            raise ValueError('local_feedback_identity_mismatch')
        start, end, sequence = f['read_start_ns'], f['read_end_ns'], f['sequence']
        if (any(type(n) is not int for n in [start, end, sequence, f['physics_step']]) or
                not 0 <= start <= end <= now or end-start > 1_000_000):
            raise ValueError('invalid_local_feedback_read')
        if now-start > 100_000_000:
            raise ValueError('current_local_feedback_older_than_100ms')
        q = np.asarray(f['q'], float)
        if q.shape != (5,) or not np.isfinite(q).all():
            raise ValueError('invalid_current_local_q')

    def receive_response(self, request, *, sealed, pixels, depths, expected_nonce=None):
        """Read the trusted socket ourselves; caller-supplied JSON is not proof."""
        if self.session_finished:raise ValueError('trusted_session_finished')
        response, extra = receive_packet(self.sock)
        require_peer(peer_credentials(self.sock), self.pid, self.uid)
        if response.get('kind')=='finish':
            if extra:raise ValueError('terminal_control_blobs_forbidden')
            self.accept_finish(response)
            return response,None,None
        if (extra or response.get('kind') not in ('verdict', 'local_feedback', 'local_feedback_denied') or
                response.get('nonce') != (request['nonce'] if expected_nonce is None else expected_nonce) or
                response.get('publisher_gid') != self.gid or
                response.get('request_sha256') != hashlib.sha256(canonical(request)).hexdigest()):
            raise ValueError('trusted_response_binding_mismatch')
        if response['kind']=='local_feedback_denied':
            if request.get('kind')!='refresh_local_feedback' or 'local_feedback'in response or canonical(response.get('denial'))!=canonical({'reason':'refresh_capture_session_missing_or_expired','discard_capture':True,'motion_authorized':False}):raise ValueError('invalid_refresh_denial')
            self._discard_nonce(request['nonce'])
            return response,None,None
        now = time.monotonic_ns()
        for handle, item in list(self.captures.items()):
            if now-item['body']['capture_monotonic_lower_bound_ns']>500_000_000:del self.captures[handle]
        for handle, item in list(self.feedback.items()):
            if now-item['data']['read_start_ns']>100_000_000:del self.feedback[handle]
        capture_handle = feedback_handle = None
        if response['kind'] == 'verdict' and not response['verdict']['matched']:
            return response, None, None
        body = sealed['body']
        if self.camera_schema:self.camera_schema.verify_body(body)
        elif body.get('wire_schema') is not None:raise ValueError('mixed_capture_requires_explicit_schema')
        if response['kind'] == 'verdict':
            v = response['verdict']
            if (v.get('rgbd_capture_association_verified') is not True or
                    v.get('motion_authorized') is not False or
                    body['run_id'] != self.run or body['clock_epoch'] != self.clock or
                    body['source_sha256'] != self.source or body['model_sha256'] != self.model or
                    v['capture_physics_step'] != body['capture_physics_step'] or
                    v['encoder_history'] != body['encoder_history'] or
                    request['pixel_sha256'] != body['pixel_sha256'] or
                    request['depth_sha256'] != body['depth_sha256']):
                raise ValueError('verified_capture_contract_mismatch')
            if not 0 <= now-body['capture_monotonic_lower_bound_ns'] <= 500_000_000:
                raise ValueError('capture_expired_at_boundary')
        local = response['local_feedback']
        self._validate_feedback(local, now)
        if local['sequence'] <= self.last_feedback_sequence:
            raise ValueError('local_feedback_sequence_replay')
        if response['kind'] == 'verdict' and len(self.captures) >= self.max_pending_captures:
            raise ValueError('pending_capture_limit')
        if len(self.feedback) >= self.max_pending_feedback:
            raise ValueError('pending_feedback_limit')
        # Both records are registered atomically after every identity/age/limit check.
        if response['kind'] == 'verdict':
            capture_handle = object()
            self.captures[capture_handle] = {'body': copy.deepcopy(body),
                                           'nonce': request['nonce'], 'received_ns': now}
        self.last_feedback_sequence = local['sequence']
        feedback_handle = object()
        self.feedback[feedback_handle] = {'data': copy.deepcopy(local), 'nonce': request['nonce']}
        return response, capture_handle, feedback_handle

    def _discard_nonce(self,nonce):
        for handles in (self.captures,self.feedback):
            for handle,item in list(handles.items()):
                if item['nonce']==nonce:del handles[handle]

    def accept_finish(self,response):
        if self.session_finished:raise ValueError('trusted_session_finished')
        expected={'kind':'finish','run_id':self.run,'clock_epoch':self.clock,'source_sha256':self.source,'model_sha256':self.model,'publisher_gid':self.gid,'motion_authorized':False}
        if canonical(response)!=canonical(expected):raise ValueError('terminal_session_binding_mismatch')
        self.captures.clear();self.feedback.clear();self.session_finished=True

    def discard_pair(self, capture_receipt, feedback_receipt):
        """Fail-closed release when another gate rejects; never creates receipts."""
        self.captures.pop(capture_receipt, None)
        self.feedback.pop(feedback_receipt, None)

    def local_packet(self, handle):
        """Build a packet view only from registered live local feedback."""
        local = copy.deepcopy(self.feedback[handle]['data'])
        return {'local_feedback': local, 'arm_measured_position': local['q'].copy(),
                'arm_feedback_capture_monotonic_ns': (local['read_start_ns']+local['read_end_ns'])//2}

    def join(self, frame, packet, *, capture_receipt, feedback_receipt, now_ns):
        if capture_receipt not in self.captures or feedback_receipt not in self.feedback:
            raise ValueError('live_socket_receipts_required')
        capture = self.captures[capture_receipt]
        feedback = self.feedback[feedback_receipt]
        body, current = capture['body'], feedback['data']
        if feedback['nonce'] != capture['nonce']:
            raise ValueError('capture_feedback_session_mismatch')
        self._validate_feedback(current, now_ns)
        if not 0 <= now_ns-body['capture_monotonic_lower_bound_ns'] <= 500_000_000:
            raise ValueError('capture_expired_at_adapter')
        if current['physics_step'] < body['capture_physics_step']:
            raise ValueError('local_feedback_predates_image')
        if packet.get('local_feedback') != current:
            raise ValueError('local_feedback_overwritten_or_identity_mismatch')
        if (not np.array_equal(packet.get('arm_measured_position'), current['q']) or
                packet.get('arm_feedback_capture_monotonic_ns') !=
                (current['read_start_ns']+current['read_end_ns'])//2):
            raise ValueError('current_q_overwritten_by_untrusted_or_historical_value')
        def frame_int(name):
            value=np.asarray(frame[name])
            if value.shape!=() or value.dtype.kind not in 'iu':raise ValueError('invalid_integer_frame_metadata:'+name)
            return int(value)
        run = str(np.asarray(frame['run_id']).item())
        step = frame_int('frame_index')
        capture_ns = frame_int('capture_monotonic_ns')
        raw = body['native_camera_metadata']['arm_camera_left_optical']
        stamp = round(Fraction(raw['referenceTimeNumerator'],raw['referenceTimeDenominator'])*1_000_000_000)
        if (run != self.run or step != body['capture_physics_step'] or
                str(np.asarray(frame['clock_domain']).item()) != 'native_render_reference_time_v2' or
                frame_int('stamp_ns') != stamp or
                capture_ns != body['capture_monotonic_lower_bound_ns'] or
                packet.get('capture_run_id') != run or packet.get('stamp_ns') != stamp or
                packet.get('frame_capture_monotonic_ns') != capture_ns or
                packet.get('publisher_gid') != self.gid or
                not body['issued_ns'] <= packet.get('packet_received_monotonic_ns', -1) <= now_ns):
            raise ValueError('native_capture_packet_identity_mismatch')
        if self.camera_schema:self.camera_schema.verify_archive(frame)
        if any((self.camera_schema.digest(frame[k],k,'rgb') if self.camera_schema else digest_image(frame[k])) != value for k, value in body['pixel_sha256'].items()):
            raise ValueError('RGB_bytes_changed_after_RPC')
        if any((self.camera_schema.digest(frame[k],k,'depth') if self.camera_schema else digest_depth(frame[k])) != value for k, value in body['depth_sha256'].items()):
            raise ValueError('depth_bytes_changed_after_RPC')
        history = body['encoder_history']
        if history['joint_names'] != JOINTS:
            raise ValueError('historical_joint_names_mismatch')
        if packet.get('camera_transforms') != history['camera_fk']:
            raise ValueError('historical_camera_FK_overwritten')
        admitted_ns=max(now_ns,time.monotonic_ns())
        self._validate_feedback(current,admitted_ns)
        if admitted_ns-body['capture_monotonic_lower_bound_ns']>500_000_000:raise ValueError('capture_expired_during_adapter_validation')
        result = {'admission_monotonic_ns':admitted_ns,'historical_encoder': copy.deepcopy(history),
                  'current_local_feedback': copy.deepcopy(current),
                  'image_capture_association_verified': True,
                  'rgbd_capture_association_verified': True,
                  'motion_authorized': False}
        # Inputs can be used once even when later ID/pose checks reject the image.
        del self.captures[capture_receipt]
        del self.feedback[feedback_receipt]
        return result
