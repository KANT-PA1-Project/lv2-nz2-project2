import unittest
from arm_practice.perception_response_identity import check_perception_response


class ResponseIdentityTests(unittest.TestCase):
    def setUp(self):
        self.req = dict(run_id='run-a', clock_domain='synthetic_packet_20hz',
                        camera='arm_camera', frame_index=2, stamp_ns=1_100_000_000,
                        capture_monotonic_ns=100)
        self.res = dict(self.req, inference_started_monotonic_ns=120,
                        inference_completed_monotonic_ns=150)

    def check(self, **kw):
        return check_perception_response(self.req, self.res, now_ns=kw.get('now', 180),
                                         max_age_ns=kw.get('age', 80))

    def test_exact_age_boundary_is_valid_not_motion_authorization(self):
        r = self.check()
        self.assertTrue(r['matched'])
        self.assertEqual(r['inference_duration_ns'], 30)
        self.assertEqual(r['capture_to_completion_ns'], 50)
        self.assertFalse(r['execution_authorized'])

    def test_cached_result_expires_at_consumption(self):
        self.assertEqual(self.check(now=181)['reason'], 'stale_response')

    def test_wrong_identity(self):
        for key,value in [('run_id','run-b'),('camera','r07_camera1'),
                          ('capture_monotonic_ns',99)]:
            with self.subTest(key=key):
                res=dict(self.res);res[key]=value
                self.assertFalse(check_perception_response(self.req,res,now_ns=180,max_age_ns=80)['matched'])
        self.res.update(frame_index=3,stamp_ns=1_150_000_000)
        self.assertEqual(self.check()['reason'],'response_identity_mismatch')

    def test_missing_response_metadata(self):
        for key in self.res:
            with self.subTest(key=key):
                res=dict(self.res);del res[key]
                self.assertFalse(check_perception_response(self.req,res,now_ns=180,max_age_ns=80)['matched'])

    def test_clock_order(self):
        for start,done,now in [(99,150,180),(151,150,180),(120,181,180)]:
            self.res.update(inference_started_monotonic_ns=start,inference_completed_monotonic_ns=done)
            self.assertEqual(self.check(now=now)['reason'],'clock_order_mismatch')

    def test_boolean_and_float_times_rejected(self):
        for value in [True,120.0,'120',None]:
            self.res['inference_started_monotonic_ns']=value
            self.assertFalse(self.check()['matched'])
        self.assertFalse(self.check(age=True)['matched'])

    def test_invalid_stamp_and_camera(self):
        self.res['stamp_ns']+=1
        self.assertEqual(self.check()['reason'],'invalid_frame_stamp')
        self.res=dict(self.req,inference_started_monotonic_ns=120,inference_completed_monotonic_ns=150,camera='bad')
        self.assertEqual(self.check()['reason'],'invalid_camera')

    def test_no_arm_feedback_required(self):
        self.assertTrue(self.check()['matched'])

if __name__ == '__main__':
    unittest.main()
