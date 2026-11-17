import unittest
from arm_practice.native_rgbd.trusted_capture_ipc import TrustedCaptureChannel

class RecentQueueTests(unittest.TestCase):
    def setUp(self):
        self.c=TrustedCaptureChannel.__new__(TrustedCaptureChannel);self.c.recent_requests={}
    def test_default_32_exact_max_and_drop_new(self):
        for i in range(32):self.c._register_request({'nonce':str(i),'kind':'verify'},0,10)
        with self.assertRaisesRegex(ValueError,'producer_pending_request_limit'):self.c._register_request({'nonce':'33'},0,10)
        self.assertEqual(len(self.c.recent_requests),32)
    def test_expired_queue_frees_capacity(self):
        for i in range(32):self.c._register_request({'nonce':str(i)},0,10)
        self.c._register_request({'nonce':'new'},500_000_001,500_000_001)
        self.assertEqual(list(self.c.recent_requests),['new'])
    def test_exact_500ms_refresh_and_past_expiry(self):
        self.c._register_request({'nonce':'a','kind':'verify'},0,0)
        r={'nonce':'a','kind':'refresh_local_feedback'}
        self.c._validate_refresh_request(r,500_000_000)
        with self.assertRaisesRegex(ValueError,'expired'):self.c._validate_refresh_request(r,500_000_001)
    def test_unknown_old_nonce_not_refreshed(self):
        with self.assertRaisesRegex(ValueError,'missing_or_expired'):self.c._validate_refresh_request({'nonce':'unknown'},10)
    def test_old_request_must_match_all_fields(self):
        self.c._register_request({'nonce':'a','kind':'verify','source':'abc'},0,0)
        with self.assertRaisesRegex(ValueError,'binding_mismatch'):self.c._validate_refresh_request({'nonce':'a','kind':'refresh_local_feedback','source':'changed'},10)
    def test_duplicate_nonce_denied(self):
        self.c._register_request({'nonce':'a'},0,0)
        with self.assertRaisesRegex(ValueError,'nonce_reuse'):self.c._register_request({'nonce':'a'},1,1)
    def test_invalid_recent_clock(self):
        for capture,now in [(True,1),(1,0),(1,1.0),(-1,0)]:
            with self.subTest(capture=capture,now=now),self.assertRaisesRegex(ValueError,'invalid_recent_request_clock'):self.c._register_request({'nonce':'a'},capture,now)
