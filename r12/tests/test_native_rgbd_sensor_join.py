import copy
import hashlib
import os
from pathlib import Path
import socket
import sys
import time
import unittest
from unittest.mock import patch
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts/sensor_runtime'))
from arm_practice.native_rgbd.renderer_epoch_issuer import RendererEpochIssuer, canonical
from arm_practice.native_rgbd.trusted_capture_ipc import make_request, send_packet
from arm_practice.native_rgbd.separated_sensor_inputs import SeparatedSensorInputs, JOINTS
from arm_practice.native_rgbd.pick_sensor_adapter import PickSensorAdapter, clearance_from_report
from arm_practice.isaac_pixel_geometry import archive_geometry_fields, metadata


class BoundaryTests(unittest.TestCase):
    def setUp(self):
        pixel_env=patch.dict(os.environ,{"R12_ISAAC_PIXEL_CENTER_V1":"1"});pixel_env.start();self.addCleanup(pixel_env.stop)
        self.now = time.monotonic_ns()
        self.sock, self.server = socket.socketpair()
        self.addCleanup(self.sock.close)
        self.addCleanup(self.server.close)
        self.cameras = ['arm_camera_left_optical', 'arm_camera_right_optical']
        def fk(q):
            transform = np.eye(4)
            transform[0, 3] = q['arm_joint1']
            return {k: transform.copy() for k in self.cameras}
        issuer = RendererEpochIssuer(run_id='r', clock_epoch='epoch', cameras=self.cameras,
            joint_names=JOINTS, physics_hz=960, model_sha256='0'*64, source_sha256='1'*64,
            fk=fk, depth_camera_map={'arm_camera_depth': self.cameras[0]})
        for step, q, age in [(96, .1, 60_000_000), (144, .2, 10_000_000)]:
            start = self.now-age
            issuer.record_encoder_epoch(physics_step=step, sim_time_s=step/960,
                read_start_ns=start, read_end_ns=start+100, measured_encoder=[q]*5)
        self.pixels = {k: np.zeros((480, 640, 3), np.uint8) for k in self.cameras}
        self.depths = {'arm_camera_depth': np.ones((480, 640), np.float32)}
        rgb_meta = {k: {'referenceTimeNumerator': 100000000, 'referenceTimeDenominator': 1000000000} for k in self.cameras}
        depth_meta = {'arm_camera_depth': copy.deepcopy(rgb_meta[self.cameras[0]])}
        self.sealed = issuer.issue(native_camera_metadata=rgb_meta, pixels=self.pixels,
            native_depth_metadata=depth_meta, depths=self.depths,
            native_metadata_after={'rgb': rgb_meta, 'depth': depth_meta},
            current_physics_step=144, readback_return_ns=self.now, now_ns=self.now)
        verdict = issuer.verify(self.sealed, pixels=self.pixels, depths=self.depths, now_ns=self.now)
        self.request = make_request(self.sealed, self.pixels, self.depths, nonce='n', gid='a1'*16)
        self.feedback = {'read_start_ns': self.now-1_000_000, 'read_end_ns': self.now-999900,
            'sequence': 1, 'physics_step': 144, 'q': [.2]*5,
            'identity': {'run_id': 'r', 'clock_epoch': 'epoch', 'source_sha256': '1'*64,
                'model_sha256': '0'*64, 'producer_pid': os.getpid(), 'stream_id': 'stream',
                'kind': 'current_local_actuator_encoder', 'joint_names': JOINTS}}
        self.boundary = SeparatedSensorInputs(self.sock, producer_pid=os.getpid(), uid=os.getuid(),
            run_id='r', clock_epoch='epoch', source_sha256='1'*64, model_sha256='0'*64,
            publisher_gid='a1'*16, feedback_stream_id='stream')
        response = {'kind': 'verdict', 'nonce': 'n', 'publisher_gid': 'a1'*16,
            'request_sha256': hashlib.sha256(canonical(self.request)).hexdigest(),
            'verdict': verdict, 'local_feedback': self.feedback}
        send_packet(self.server, response)
        _, self.capture, self.current = self.boundary.receive_response(self.request,
            sealed=self.sealed, pixels=self.pixels, depths=self.depths)
        body = self.sealed['body']
        self.frame = dict(self.pixels, **self.depths, run_id=np.array('r'), frame_index=np.int64(96),
            stamp_ns=np.int64(100000000), clock_domain=np.array('native_render_reference_time_v2'),
            capture_monotonic_ns=np.int64(body['capture_monotonic_lower_bound_ns']), **archive_geometry_fields())
        self.packet = dict(capture_run_id='r', stamp_ns=100000000,
            frame_capture_monotonic_ns=body['capture_monotonic_lower_bound_ns'],
            packet_received_monotonic_ns=self.now, publisher_gid='a1'*16,
            camera_transforms=copy.deepcopy(body['encoder_history']['camera_fk']),
            **self.boundary.local_packet(self.current))
        self.packet['perception'] = dict(run_id='r', stamp_ns=100000000,
            capture_monotonic_ns=body['capture_monotonic_lower_bound_ns'], camera='arm_camera',
            camera_frame=self.cameras[0], pixel_geometry=metadata(), detections=[])

    def join(self, now=None):
        return self.boundary.join(self.frame, self.packet, capture_receipt=self.capture,
            feedback_receipt=self.current, now_ns=time.monotonic_ns() if now is None else now)

    def test_historical_FK_and_current_q_are_distinct(self):
        pair = self.join()
        self.assertEqual(pair['historical_encoder']['encoder_q'], [.1]*5)
        self.assertEqual(pair['historical_encoder']['camera_fk'][self.cameras[0]][0][3], .1)
        self.assertEqual(pair['current_local_feedback']['q'], [.2]*5)
        self.assertEqual(self.packet['arm_measured_position'], [.2]*5)
        self.assertFalse(pair['motion_authorized'])

    def test_historical_q_value_overwrite_rejected(self):
        self.packet['arm_measured_position'] = [.1]*5
        with self.assertRaisesRegex(ValueError, 'current_q_overwritten'):
            self.join()

    def test_historical_q_provenance_overwrite_rejected(self):
        self.packet['local_feedback']['identity']['kind'] = 'historical_capture_encoder'
        with self.assertRaisesRegex(ValueError, 'local_feedback_overwritten'):
            self.join()

    def test_wrong_identity_rejected(self):
        self.packet['local_feedback']['identity']['stream_id'] = 'other'
        with self.assertRaisesRegex(ValueError, 'identity_mismatch'):
            self.join()

    def test_current_q_stale_even_image_is_fresh(self):
        with self.assertRaisesRegex(ValueError, 'older_than_100ms'):
            self.join(now=self.feedback['read_start_ns']+100_000_001)

    def test_current_read_age_at_exact_boundary(self):
        self.assertTrue(self.join(now=self.feedback['read_start_ns']+100_000_000)['rgbd_capture_association_verified'])

    def test_receipt_replay_rejected(self):
        self.join()
        with self.assertRaisesRegex(ValueError, 'live_socket_receipts_required'):
            self.join()

    def test_packet_boolean_not_receipt(self):
        self.capture = object()
        self.packet['rgbd_capture_association_verified'] = True
        with self.assertRaisesRegex(ValueError, 'live_socket_receipts_required'):
            self.join()

    def test_current_FK_cannot_replace_historical_FK(self):
        self.packet['camera_transforms'][self.cameras[0]][0][3] = .2
        with self.assertRaisesRegex(ValueError, 'historical_camera_FK_overwritten'):
            self.join()

    def test_changed_depth_after_rpc_rejected(self):
        self.frame['arm_camera_depth'][0, 0] += .01
        with self.assertRaisesRegex(ValueError, 'depth_bytes_changed'):
            self.join()

    def test_no_identity_pose_clearance_means_no_observation(self):
        os.environ['R12_ISAAC_PIXEL_CENTER_V1'] = '1'
        adapter = PickSensorAdapter(ROOT, sensor_inputs=self.boundary)
        observation = adapter.observe(self.frame, self.packet, {}, now_monotonic_ns=time.monotonic_ns(),
            desired_object_rotation_in_tcp=np.eye(3), capture_receipt=self.capture, feedback_receipt=self.current)
        self.assertIsNone(observation)
        self.assertEqual(adapter.reason, 'identity_unconfirmed')
        self.assertTrue(adapter.input_audit['admitted'])
        self.assertIsNone(clearance_from_report({}, phase='approach', capture_ns=1,
            target_id=None, q_start=[.2]*5, q_target=[.2]*5, width_target=.075, model_sha256='0'*64))

class ReviewAdmissionTests(unittest.TestCase):
    setUp = BoundaryTests.setUp
    join = BoundaryTests.join
    def test_float_frame_index_is_not_identity(self):
        self.frame['frame_index']=np.float64(96.1)
        with self.assertRaisesRegex(ValueError,'invalid_integer_frame_metadata'):self.join()
    def test_bool_native_stamp_is_not_identity(self):
        self.frame['stamp_ns']=np.bool_(True)
        with self.assertRaisesRegex(ValueError,'invalid_integer_frame_metadata'):self.join()
    def test_local_age_rechecked_after_byte_validation(self):
        with patch('arm_practice.native_rgbd.separated_sensor_inputs.time.monotonic_ns',return_value=self.feedback['read_start_ns']+100_000_001):
            with self.assertRaisesRegex(ValueError,'older_than_100ms'):
                self.join(now=self.feedback['read_start_ns']+99_000_000)

class PendingQueueTests(unittest.TestCase):
    setUp = BoundaryTests.setUp
    join = BoundaryTests.join

    def receive(self, *, kind='verdict', matched=True, sequence=2, now=None):
        req=dict(self.request,kind='refresh_local_feedback') if kind=='local_feedback' else self.request
        current=copy.deepcopy(self.feedback);current['sequence']=sequence
        if now is not None:current.update(read_start_ns=now-1000,read_end_ns=now-900)
        body=self.sealed['body']
        reply={'kind':kind,'nonce':'n','publisher_gid':'a1'*16,'request_sha256':hashlib.sha256(canonical(req)).hexdigest(),'local_feedback':current}
        if kind=='verdict':reply['verdict']={'matched':matched,'reason':'unit_fixture','rgbd_capture_association_verified':True,'motion_authorized':False,'capture_physics_step':body['capture_physics_step'],'encoder_history':body['encoder_history']}
        send_packet(self.server,reply)
        return self.boundary.receive_response(req,sealed=self.sealed,pixels=self.pixels,depths=self.depths)

    def test_default_capture_32_exact_limit_and_negative_still_readable(self):
        self.assertEqual(self.boundary.max_pending_captures,32)
        for i in range(2,33):self.receive(sequence=i)
        self.assertEqual(len(self.boundary.captures),32)
        with self.assertRaisesRegex(ValueError,'pending_capture_limit'):self.receive(sequence=33)
        self.assertEqual(len(self.boundary.captures),32)
        self.assertEqual(len(self.boundary.feedback),32)
        reply,cap,feed=self.receive(matched=False,sequence=33)
        self.assertFalse(reply['verdict']['matched']);self.assertIsNone(cap);self.assertIsNone(feed)

    def test_full_capture_queue_allows_current_feedback_refresh(self):
        self.boundary.max_pending_captures=1
        _,cap,feed=self.receive(kind='local_feedback')
        self.assertIsNone(cap);self.assertIn(feed,self.boundary.feedback)
        self.assertEqual(len(self.boundary.captures),1)
        self.current=feed;self.packet.update(self.boundary.local_packet(feed));self.join()
        self.assertEqual(len(self.boundary.captures),0)

    def test_default_feedback_64_exact_limit_rejection_is_atomic(self):
        self.assertEqual(self.boundary.max_pending_feedback,64)
        for i in range(2,65):self.receive(kind='local_feedback',sequence=i)
        with self.assertRaisesRegex(ValueError,'pending_feedback_limit'):self.receive(sequence=65)
        self.assertEqual(len(self.boundary.captures),1);self.assertEqual(len(self.boundary.feedback),64)
        self.assertEqual(self.boundary.last_feedback_sequence,64)

    def test_expired_handles_pruned_and_input_recovers(self):
        body=self.sealed['body'];future=body['capture_monotonic_lower_bound_ns']+500_000_001
        with patch('arm_practice.native_rgbd.separated_sensor_inputs.time.monotonic_ns',return_value=future):
            _,cap,feed=self.receive(kind='local_feedback',now=future)
        self.assertEqual(len(self.boundary.captures),0);self.assertEqual(len(self.boundary.feedback),1)
        self.assertIsNone(cap);self.assertIn(feed,self.boundary.feedback)

    def test_limits_are_bounded_integers(self):
        base=dict(producer_pid=os.getpid(),uid=os.getuid(),run_id='r',clock_epoch='epoch',source_sha256='1'*64,model_sha256='0'*64,publisher_gid='a1'*16,feedback_stream_id='stream')
        for field,values in [('max_pending_captures',[0,33,True,1.0]),('max_pending_feedback',[0,65,True,1.0])]:
            for v in values:
                with self.subTest(field=field,value=v),self.assertRaisesRegex(ValueError,'invalid_pending_limits'):SeparatedSensorInputs(self.sock,**base,**{field:v})

    def test_invalid_local_feedback_cannot_leave_capture_handle(self):
        before=len(self.boundary.captures)
        self.feedback['identity']['stream_id']='bad'
        with self.assertRaisesRegex(ValueError,'local_feedback_identity_mismatch'):self.receive()
        self.assertEqual(len(self.boundary.captures),before)

    def test_native_fraction_keeps_nanosecond_beyond_float_precision(self):
        value=10**18+1
        # Trusted receipt fixture tests boundary arithmetic, not producer proof issuance.
        self.boundary.captures[self.capture]['body']['native_camera_metadata']['arm_camera_left_optical']={'referenceTimeNumerator':value,'referenceTimeDenominator':10**9}
        self.frame['stamp_ns']=np.int64(value);self.packet['stamp_ns']=value
        self.assertTrue(self.join()['rgbd_capture_association_verified'])

    def test_capture_scalar_array_rejected(self):
        self.frame['capture_monotonic_ns']=np.array([int(self.frame['capture_monotonic_ns'])])
        with self.assertRaisesRegex(ValueError,'invalid_integer_frame_metadata'):self.join()

    def test_image_age_rechecked_after_hash_validation(self):
        body=self.boundary.captures[self.capture]['body'];future=body['capture_monotonic_lower_bound_ns']+500_000_001
        self.boundary.feedback[self.current]['data']['read_start_ns']=future-1000
        self.boundary.feedback[self.current]['data']['read_end_ns']=future-900
        self.packet.update(self.boundary.local_packet(self.current))
        with patch('arm_practice.native_rgbd.separated_sensor_inputs.time.monotonic_ns',return_value=future):
            with self.assertRaisesRegex(ValueError,'capture_expired_during_adapter_validation'):self.join(now=future-1)

class AdapterHistoricalPoseTests(unittest.TestCase):
    """CPU receipt fixture; mocked FK is never native pose/clearance evidence."""
    setUp = BoundaryTests.setUp

    def observe_candidate(self, confirmed_stamp=100000000):
        self.packet['perception']['detections'] = [{
            'tag_ids': [3], 'class_id': 0, 'depth_consistent': True,
            'pose': {'translation_valid': True, 'rotation_valid': True,
                     'confidence': .9, 'center_camera_m': [0, 0, .2],
                     'rotation_matrix': np.eye(3).tolist()}}]
        confirmed = dict(evidence_role='wrist_confirmed_identity_with_pose_estimate',
            camera='arm_camera', camera_frame=self.cameras[0], run_id='r',
            stamp_ns=confirmed_stamp, wrist_consistent_frames=3, id=3)
        adapter = PickSensorAdapter(ROOT, sensor_inputs=self.boundary,
            pose_error_budget={'validated': True, 'scope': 'simulation_sensor_pose',
                               'report_sha256': '2'*64, 'translation_bound_m': .001})
        seen = []
        def fk(root, joints):
            seen.append(dict(joints))
            T=np.eye(4);T[0,3]=joints['arm_joint1']
            return {self.cameras[0]: T}
        with patch('arm_practice.native_rgbd.pick_sensor_adapter.transforms', side_effect=fk), \
             patch('arm_practice.native_rgbd.pick_sensor_adapter.forward_transform', return_value=np.eye(4)):
            obs=adapter.observe(self.frame,self.packet,confirmed,
                now_monotonic_ns=self.now,desired_object_rotation_in_tcp=np.eye(3),
                capture_receipt=self.capture,feedback_receipt=self.current)
        return obs,adapter,seen

    def test_admitted_observation_uses_historical_q_preserves_current_feedback(self):
        original=copy.deepcopy(self.packet)
        obs,adapter,seen=self.observe_candidate()
        self.assertIsNotNone(obs,adapter.reason)
        self.assertEqual(list(seen[0].values()),[.1]*5)
        np.testing.assert_allclose(obs.object_base_m,[.1,0,.2])
        self.assertEqual(adapter.input_audit['current_local_q'],[.2]*5)
        self.assertEqual(self.packet['arm_measured_position'],original['arm_measured_position'])
        self.assertFalse(adapter.input_audit['motion_authorized'])
        self.assertEqual(adapter.reason,'schema_joined_not_clearance_authorized')

    def test_cached_identity_cannot_authorize_current_native_image(self):
        obs,adapter,seen=self.observe_candidate(50000000)
        self.assertIsNone(obs)
        self.assertEqual(adapter.reason,'identity_unconfirmed')
        self.assertEqual(seen,[])


    def test_feedback_expiring_during_pose_work_rejected(self):
        from types import SimpleNamespace
        deadline=self.feedback['read_start_ns']+100_000_001
        with patch('arm_practice.native_rgbd.pick_sensor_adapter.time',
                   SimpleNamespace(monotonic_ns=lambda:deadline)):
            obs,adapter,_=self.observe_candidate()
        self.assertIsNone(obs)
        self.assertEqual(adapter.reason,'feedback_expired_during_observation')
        self.assertFalse(adapter.input_audit['observation_admitted'])
        self.assertEqual(self.boundary.captures,{})

    def test_image_expiring_during_pose_work_rejected(self):
        from types import SimpleNamespace
        deadline=int(self.frame['capture_monotonic_ns'])+500_000_001
        with patch('arm_practice.native_rgbd.pick_sensor_adapter.time',
                   SimpleNamespace(monotonic_ns=lambda:deadline)):
            obs,adapter,_=self.observe_candidate()
        self.assertIsNone(obs)
        self.assertEqual(adapter.reason,'image_expired_during_observation')
        self.assertFalse(adapter.input_audit['observation_admitted'])
        self.assertEqual(self.boundary.captures,{})
