import unittest,numpy as np
from arm_practice.capture_joint_epoch import validate_capture_epoch
class EpochTests(unittest.TestCase):
 def setUp(self):
  self.f={k:np.int64(v) for k,v in dict(capture_monotonic_ns=1000000,encoder_read_start_monotonic_ns=1000000,encoder_read_end_monotonic_ns=1100000,arm_feedback_capture_monotonic_ns=1050000,render_start_monotonic_ns=1150000,render_return_monotonic_ns=2000000,readback_return_monotonic_ns=2500000,physics_step_before=48,physics_step_after=48).items()};self.f.update(arm_encoder=np.zeros(5),arm_encoder_after_render=np.zeros(5))
 def check(self,**kwargs):return validate_capture_epoch(self.f,received_monotonic_ns=3000000,now_monotonic_ns=kwargs.get('now',3500000))
 def test_valid_not_motion_authorization(self):self.assertTrue(self.check()['matched']);self.assertFalse(self.check()['motion_authorized']);self.assertFalse(self.check()['image_capture_association_verified'])
 def test_missing_encoder_time(self):del self.f['encoder_read_start_monotonic_ns'];self.assertFalse(self.check()['matched'])
 def test_cannot_relabel_capture_at_render_return(self):self.f['capture_monotonic_ns']=np.int64(2000000);self.assertFalse(self.check()['matched'])
 def test_negative_physics_step(self):
  self.f['physics_step_before']=self.f['physics_step_after']=np.int64(-1);self.assertFalse(self.check()['matched'])
 def test_physics_advance(self):self.f['physics_step_after']=np.int64(49);self.assertFalse(self.check()['matched'])
 def test_changed_encoder(self):self.f['arm_encoder_after_render'][0]=.001;self.assertFalse(self.check()['matched'])
 def test_nan_encoder(self):self.f['arm_encoder'][0]=np.nan;self.assertFalse(self.check()['matched'])
 def test_stale_render_included(self):self.assertEqual(self.check(now=600000000)['reason'],'stale_complete_acquisition')
 def test_future_packet(self):self.assertFalse(self.check(now=2900000)['matched'])
 def test_missing_readback(self):del self.f['readback_return_monotonic_ns'];self.assertFalse(self.check()['matched'])
 def test_midpoint_not_arbitrary_timestamp(self):self.f['arm_feedback_capture_monotonic_ns']=np.int64(1000000);self.assertFalse(self.check()['matched'])
 def test_long_join_interval(self):self.f['render_start_monotonic_ns']=np.int64(2100000);self.f['render_return_monotonic_ns']=np.int64(2200000);self.assertFalse(self.check()['matched'])
if __name__=='__main__':unittest.main()
