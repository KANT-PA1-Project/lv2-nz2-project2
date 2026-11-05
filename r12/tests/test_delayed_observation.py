import unittest
from arm_practice.delayed_observation import bind_historical_response
from arm_practice.perception_response_identity import check_perception_response

class DelayedObservationTests(unittest.TestCase):
    def setUp(self):
        self.q=dict(run_id='a',clock_domain='synthetic_packet_20hz',camera='r07_camera2',frame_index=0,stamp_ns=1000000000,capture_monotonic_ns=100)
        self.r=dict(self.q,inference_started_monotonic_ns=120,inference_completed_monotonic_ns=180,tags=[dict(id=4,corners=[[1,2]])])
    def bind(self):
        return bind_historical_response(self.q,self.r,received_ns=200,max_age_ns=100)
    def test_historical_not_current_or_pose_or_motion(self):
        x=self.bind()
        for k in ['current_observation_eligible','pose_associated','execution_authorized']:self.assertFalse(x[k])
        self.assertEqual(x['request']['capture_monotonic_ns'],100)
    def test_snapshot_does_not_alias_mutable_inputs(self):
        x=self.bind();self.r['tags'][0]['corners'][0][0]=99;self.q['run_id']='b'
        self.assertEqual(x['response']['tags'][0]['corners'],[[1,2]]);self.assertEqual(x['request']['run_id'],'a')
    def test_new_packet_cannot_claim_old_observation(self):
        x=self.bind();new=dict(self.q,frame_index=1,stamp_ns=1050000000,capture_monotonic_ns=150)
        self.assertFalse(check_perception_response(new,x['response'],now_ns=200,max_age_ns=100)['matched'])
    def test_stale_receipt_rejected(self):
        with self.assertRaisesRegex(ValueError,'stale_response'):bind_historical_response(self.q,self.r,received_ns=201,max_age_ns=100)
    def test_old_receipt_verdict_cannot_refresh_age(self):
        x=self.bind();self.assertFalse(check_perception_response(x['request'],x['response'],now_ns=201,max_age_ns=100)['matched'])
    def test_wrong_run_or_camera_rejected(self):
        for k,v in [('run_id','b'),('camera','arm_camera')]:
            r=dict(self.r);r[k]=v
            with self.assertRaises(ValueError):bind_historical_response(self.q,r,received_ns=200,max_age_ns=100)
    def test_optional_pose_not_trusted(self):
        self.r['visual_pose']={'source':'fixture','position':[0,0,0]}
        self.assertFalse(self.bind()['pose_associated'])

if __name__=='__main__':unittest.main()
