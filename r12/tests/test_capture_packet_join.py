import unittest,copy,numpy as np
from arm_practice.capture_joint_epoch import joined_arm_feedback,require_frame_epoch
class PacketJoin(unittest.TestCase):
 def setUp(self):
  self.epoch=dict(capture_monotonic_ns=1000000,encoder_read_start_monotonic_ns=1000000,encoder_read_end_monotonic_ns=1100000,arm_feedback_capture_monotonic_ns=1050000,render_start_monotonic_ns=1150000,render_return_monotonic_ns=2000000,readback_return_monotonic_ns=2500000,physics_step_before=48,physics_step_after=48,arm_encoder=[0.]*5,arm_encoder_after_render=[0.]*5,run_id='r',stamp_ns=1000000000)
  self.packet=dict(arm_capture_epoch=copy.deepcopy(self.epoch),capture_run_id='r',stamp_ns=1000000000,frame_capture_monotonic_ns=1000000)
  self.feedback=dict(arm_feedback_valid=True,arm_measured_position=[0.]*5)
 def join(self,now=3500000):return joined_arm_feedback(self.packet,self.feedback,received_monotonic_ns=3000000,now_monotonic_ns=now)
 def test_match_propagates_real_midpoint(self):self.assertEqual(self.join()['arm_feedback_capture_monotonic_ns'],1050000);self.assertTrue(self.join()['arm_feedback_live_freshness_verified'])
 def test_missing_evidence_invalidates_feedback(self):del self.packet['arm_capture_epoch'];self.assertFalse(self.join()['arm_feedback_valid'])
 def test_wrong_joint_snapshot_rejected(self):self.feedback['arm_measured_position'][0]=.001;self.assertFalse(self.join()['arm_feedback_valid'])
 def test_wrong_capture_rejected(self):self.packet['frame_capture_monotonic_ns']=2;self.assertFalse(self.join()['arm_feedback_valid'])
 def test_wrong_run_rejected(self):self.packet['capture_run_id']='old';self.assertFalse(self.join()['arm_feedback_valid'])
 def test_wrong_stamp_rejected(self):self.packet['stamp_ns']+=1;self.assertFalse(self.join()['arm_feedback_valid'])
 def test_stale_after_ros_work_rejected(self):self.assertFalse(self.join(600000000)['arm_feedback_valid'])
 def test_image_fresh_but_actuator_stale_rejected(self):
  r=self.join(101000001);self.assertFalse(r['arm_feedback_live_freshness_verified']);self.assertEqual(r['arm_capture_join']['reason'],'actuator_feedback_older_than_100ms')
 def test_actuator_freshness_boundary(self):self.assertTrue(self.join(101000000)['arm_feedback_live_freshness_verified'])
 def test_consumer_rechecks_actuator_age(self):
  r=self.result();self.assertRaisesRegex(ValueError,'actuator_feedback_older_than_100ms',require_frame_epoch,self.epoch,r,now_monotonic_ns=101000001)
 def result(self):return dict(self.packet,**self.feedback,**self.join(),packet_received_monotonic_ns=3000000)
 def test_full_frame_join(self):self.assertTrue(require_frame_epoch(self.epoch,self.result(),now_monotonic_ns=3500000)['matched'])
 def test_frame_evidence_cannot_differ(self):f=copy.deepcopy(self.epoch);f['physics_step_before']=f['physics_step_after']=49;self.assertRaisesRegex(ValueError,'frame_epoch_mismatch',require_frame_epoch,f,self.result(),now_monotonic_ns=3500000)
 def test_unverified_packet_not_accepted(self):r=self.result();r['arm_feedback_live_freshness_verified']=False;self.assertRaisesRegex(ValueError,'encoder_epoch_not_verified',require_frame_epoch,self.epoch,r,now_monotonic_ns=3500000)
if __name__=='__main__':unittest.main()
