import unittest,numpy as np
from arm_practice.wheel_position_hold import WheelPositionHold
from arm_practice.latest_policy import dc_effort
class Tests(unittest.TestCase):
 def test_anchor_and_sign(self):
  h=WheelPositionHold();np.testing.assert_array_equal(h.step([5]*4,True),[0]*4);np.testing.assert_allclose(h.step([5.01]*4,True),[-.003]*4)
 def test_limit_and_disable(self):
  h=WheelPositionHold();h.step([0]*4,True);np.testing.assert_allclose(h.step([10]*4,True),[-.04]*4);np.testing.assert_array_equal(h.step([10]*4,False),[0]*4);np.testing.assert_array_equal(h.step([10]*4,True),[0]*4)
 def test_invalid(self):
  h=WheelPositionHold();np.testing.assert_array_equal(h.step([float('nan')]*4,True),[0]*4)
 def test_total_torque_limit(self):
  t=dc_effort(np.zeros(4),np.zeros(4),np.ones(4)*10,wheel_kd=.08,wheel_feedforward=-.04);self.assertTrue((abs(t)<=.108).all())
 def test_pause_preserves_anchor_without_torque(self):
  h=WheelPositionHold();h.step([0]*4,True);np.testing.assert_array_equal(h.step([.1]*4,False,retain_anchor=True),[0]*4);np.testing.assert_allclose(h.step([.1]*4,True),[-.03]*4)
unittest.main()
