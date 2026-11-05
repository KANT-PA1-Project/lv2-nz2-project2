import unittest,numpy as np
from arm_practice.transport_posture import TransportPosture
class TransportTests(unittest.TestCase):
 def gate(self):return TransportPosture({'arm_target':[0,.33,.33,.33,0],'joint_tolerance_rad':.04,'joint_velocity_limit_rad_s':.15,'settle_seconds':.5})
 def test_fold_requires_stationary_base(self):
  t=self.gate();self.assertIsNone(t.step(0,np.zeros(5),np.zeros(5),False));self.assertFalse(t.ready)
 def test_slew_settle_then_drive_and_pose_loss(self):
  t=self.gate();q=np.zeros(5)
  for k in range(80):
   target=t.step(k*.05,q,np.zeros(5),True)
   self.assertLessEqual(np.max(np.abs(target-q)),.15*.05+1e-8);q=target
  self.assertTrue(t.ready)
  t.step(4.,q,np.zeros(5),False);self.assertTrue(t.ready)
  t.step(4.05,np.zeros(5),np.zeros(5),False);self.assertFalse(t.ready);self.assertEqual(t.state,'await_base_stop')
 def test_bad_feedback_revokes_drive(self):
  t=self.gate();t.ready=True;t.step(0,[float('nan')]*5,[0]*5,True);self.assertFalse(t.ready)

class MissionTransportClearanceTests(unittest.TestCase):
 def test_mission_cannot_fold_using_encoder_only(self):
  t=TransportPosture({'arm_target':[0,.33,.33,.33,0],'joint_tolerance_rad':.04,'joint_velocity_limit_rad_s':.15,'settle_seconds':.5},require_clearance=True)
  for k in range(80):
   self.assertIsNone(t.step(k*.05,np.zeros(5),np.zeros(5),True))
   self.assertFalse(t.ready)
  self.assertEqual(t.state,'await_swept_volume_clearance')
 def test_cached_ready_cannot_bypass_missing_clearance(self):
  t=TransportPosture({'arm_target':[0]*5,'joint_tolerance_rad':.04,'joint_velocity_limit_rad_s':.15,'settle_seconds':.5},require_clearance=True)
  t.ready=True;t.command=np.ones(5)
  self.assertIsNone(t.step(0,np.zeros(5),np.zeros(5),True))
  self.assertFalse(t.ready)
  np.testing.assert_array_equal(t.command,np.zeros(5))
