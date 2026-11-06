"""CPU protocol boundaries; fixture data is never native sensor evidence."""
import copy
import hashlib
import json
import socket
import struct
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
from arm_practice.native_rgbd.renderer_epoch_issuer import canonical
from arm_practice.native_rgbd.trusted_capture_ipc import TrustedCaptureChannel,send_packet,receive_packet
from arm_practice.native_rgbd.mixed_camera_schema import MixedCameraSchema,WIRE_V1,WIRE_V2,build_config
import native_rgbd_fixture as boundary_fixtures

DENIAL={'reason':'refresh_capture_session_missing_or_expired','discard_capture':True,'motion_authorized':False}
def channel():
    c=TrustedCaptureChannel.__new__(TrustedCaptureChannel)
    c.recent_requests={};c.gid='a1'*16;c._current_feedback=Mock(return_value={'fixture':True})
    c.issuer=SimpleNamespace(run_id='r',epoch='epoch',source='1'*64,model='0'*64)
    c._register_request({'nonce':'n','kind':'verify'},1_000_000_000,1_000_000_000)
    return c

class ProducerRefreshTests(unittest.TestCase):
    def setUp(self):self.c=channel();self.req={'nonce':'n','kind':'refresh_local_feedback'}
    def test_499ms_success_then_501ms_explicit_denial(self):
        self.assertEqual(self.c._refresh_response(self.req,1_499_000_000)['kind'],'local_feedback')
        response=self.c._refresh_response(self.req,1_501_000_000)
        self.assertEqual(response['kind'],'local_feedback_denied');self.assertEqual(response['denial'],DENIAL)
        self.assertNotIn('local_feedback',response);self.assertEqual(self.c._current_feedback.call_count,1)
        self.assertEqual(self.c.recent_requests,{})
    def test_exact_500ms_remains_allowed(self):
        self.assertEqual(self.c._refresh_response(self.req,1_500_000_000)['kind'],'local_feedback')
    def test_500ms_plus_1ns_denied(self):
        self.assertEqual(self.c._refresh_response(self.req,1_500_000_001)['denial'],DENIAL)
        self.c._current_feedback.assert_not_called()
    def test_duplicate_expired_request_idempotently_denied(self):
        a=self.c._refresh_response(self.req,1_501_000_000);b=self.c._refresh_response(self.req,1_502_000_000)
        self.assertEqual(a,b);self.c._current_feedback.assert_not_called();self.assertEqual(self.c.recent_requests,{})
    def test_missing_capture_is_explicit_denial(self):
        self.c.recent_requests.clear();self.assertEqual(self.c._refresh_response(self.req,1_001_000_000)['denial'],DENIAL)
    def test_fresh_binding_error_is_not_disguised_as_expiry(self):
        with self.assertRaisesRegex(ValueError,'request_binding_mismatch'):
            self.c._refresh_response(dict(self.req,extra=1),1_499_000_000)
        self.c._current_feedback.assert_not_called();self.assertIn('n',self.c.recent_requests)
    def test_invalid_clock_rejected(self):
        for value in [True,1.0,-1]:
            with self.subTest(value=value),self.assertRaisesRegex(ValueError,'invalid_refresh_clock'):self.c._refresh_response(self.req,value)
    def test_invalid_request_rejected(self):
        for value in [{},{'kind':'verify','nonce':'n'},{'kind':'refresh_local_feedback','nonce':False}]:
            with self.subTest(value=value),self.assertRaisesRegex(ValueError,'invalid_refresh_request'):self.c._refresh_response(value,1_499_000_000)

class ConsumerRefreshTests(unittest.TestCase):
    setUp=boundary_fixtures.BoundaryFixture.setUp
    def refresh(self):return dict(self.request,kind='refresh_local_feedback')
    def denial(self):return dict(kind='local_feedback_denied',nonce='n',publisher_gid='a1'*16,request_sha256=hashlib.sha256(canonical(self.refresh())).hexdigest(),denial=copy.deepcopy(DENIAL))
    def receive(self,response,**kwargs):
        send_packet(self.server,response,**kwargs)
        return self.boundary.receive_response(self.refresh(),sealed=self.sealed,pixels=self.pixels,depths=self.depths)
    def test_expired_denial_discards_matching_receipts_without_sequence_advance(self):
        response,cap,feedback=self.receive(self.denial())
        self.assertIsNone(cap);self.assertIsNone(feedback);self.assertEqual(self.boundary.captures,{})
        self.assertEqual(self.boundary.feedback,{});self.assertEqual(self.boundary.last_feedback_sequence,1)
    def test_duplicate_expired_denial_cannot_create_receipt(self):
        self.receive(self.denial());self.receive(self.denial());self.assertEqual(self.boundary.captures,{})
        self.assertEqual(self.boundary.feedback,{})
    def test_wrong_nonce_rejected(self):
        with self.assertRaisesRegex(ValueError,'binding_mismatch'):self.receive(dict(self.denial(),nonce='wrong'))
        self.assertIn(self.capture,self.boundary.captures)
    def test_wrong_gid_rejected(self):
        with self.assertRaisesRegex(ValueError,'binding_mismatch'):self.receive(dict(self.denial(),publisher_gid='b2'*16))
    def test_wrong_request_digest_rejected(self):
        with self.assertRaisesRegex(ValueError,'binding_mismatch'):self.receive(dict(self.denial(),request_sha256='0'*64))
    def test_feedback_in_denial_rejected(self):
        with self.assertRaisesRegex(ValueError,'invalid_refresh_denial'):self.receive(dict(self.denial(),local_feedback=self.feedback))
    def test_denial_boolean_type_and_motion_rejected(self):
        for field,value in [('motion_authorized',True),('motion_authorized',0),('discard_capture',1)]:
            response=self.denial();response['denial'][field]=value
            with self.subTest(field=field,value=value),self.assertRaisesRegex(ValueError,'invalid_refresh_denial'):self.receive(response)
    def test_wrong_request_kind_cannot_receive_refresh_denial(self):
        response=self.denial();response['request_sha256']=hashlib.sha256(canonical(self.request)).hexdigest();send_packet(self.server,response)
        with self.assertRaisesRegex(ValueError,'invalid_refresh_denial'):self.boundary.receive_response(self.request,sealed=self.sealed,pixels=self.pixels,depths=self.depths)
    def test_finish_during_pending_refresh_discards_all_receipts(self):
        response,cap,feedback=self.receive(channel()._finish_packet())
        self.assertEqual(response['kind'],'finish');self.assertIsNone(cap);self.assertIsNone(feedback)
        self.assertEqual(self.boundary.captures,{});self.assertEqual(self.boundary.feedback,{})
        self.assertTrue(self.boundary.session_finished)
    def test_wrong_finish_session_rejected_without_discard(self):
        with self.assertRaisesRegex(ValueError,'terminal_session_binding_mismatch'):self.receive(dict(channel()._finish_packet(),run_id='other'))
        self.assertIn(self.capture,self.boundary.captures);self.assertFalse(self.boundary.session_finished)
    def test_duplicate_finish_rejected(self):
        self.receive(channel()._finish_packet())
        with self.assertRaisesRegex(ValueError,'trusted_session_finished'):self.boundary.accept_finish(channel()._finish_packet())
    def test_late_feedback_after_finish_rejected(self):
        self.receive(channel()._finish_packet())
        with self.assertRaisesRegex(ValueError,'trusted_session_finished'):self.receive(self.denial())
    def test_duplicate_feedback_response_sequence_rejected(self):
        response={'kind':'local_feedback','nonce':'n','publisher_gid':'a1'*16,'request_sha256':hashlib.sha256(canonical(self.refresh())).hexdigest(),'local_feedback':self.feedback}
        with self.assertRaisesRegex(ValueError,'local_feedback_sequence_replay'):self.receive(response)

class TerminalWireTests(unittest.TestCase):
    def setUp(self):
        self.a,self.b=socket.socketpair();self.addCleanup(self.a.close);self.addCleanup(self.b.close)
        self.schema=MixedCameraSchema(build_config(Path(__file__).resolve().parents[1]))
    def raw(self,data):
        body=canonical(data);self.a.sendall(struct.pack('!I',len(body))+body)
    def recv(self):return receive_packet(self.b,wire_version=WIRE_V2,camera_schema=self.schema,allow_control_finish=True)
    def test_v1_finish_accepted_while_waiting_v2_capture(self):
        send_packet(self.a,channel()._finish_packet());reply,blobs=self.recv();self.assertEqual(reply,channel()._finish_packet());self.assertEqual(blobs,[])
    def test_v1_verdict_not_allowed_as_v2_capture(self):
        send_packet(self.a,{'kind':'verdict'})
        with self.assertRaisesRegex(ValueError,'profile_mismatch'):self.recv()
    def test_v1_finish_still_requires_explicit_opt_in(self):
        send_packet(self.a,channel()._finish_packet())
        with self.assertRaisesRegex(ValueError,'profile_mismatch'):receive_packet(self.b,wire_version=WIRE_V2,camera_schema=self.schema)
    def test_v2_finish_rejected_before_payload(self):
        self.raw(dict(channel()._finish_packet(),transport_version=WIRE_V2,blob_lengths=[]))
        with self.assertRaisesRegex(ValueError,'invalid_terminal_control_profile'):self.recv()
    def test_finish_blobs_rejected_before_payload_read(self):
        self.raw(dict(channel()._finish_packet(),transport_version=WIRE_V1,blob_lengths=[1]))
        with self.assertRaisesRegex(ValueError,'invalid_terminal_control_profile'):self.recv()

if __name__=='__main__':unittest.main()
