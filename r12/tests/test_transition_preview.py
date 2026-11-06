import unittest
import numpy as np
from arm_practice.transition_preview import preview
from arm_practice.swerve import RADIUS

class PreviewTests(unittest.TestCase):
    def test_aligned_straight_and_input_immutable(self):
        angles=np.array([0.,0.,np.pi/2,-np.pi/2]);wheels=np.array([-1,-1,1,1])*.1/RADIUS;copy=angles.copy()
        result=preview(np.tile([0.,.1],(20,1)),0.,angles,wheels)
        self.assertEqual(result[0]['goal_yaw_rate_rad_s'],0.)
        self.assertLess(result[0]['score'],1e-20)
        np.testing.assert_array_equal(angles,copy)
    def test_every_reference_step_bounded(self):
        for c in preview(np.tile([.3,.1],(40,1)),0.,np.zeros(4),np.zeros(4)):
            st=np.array([[0]*4]+[x['steer_reference_rad'] for x in c['trace']]);wh=np.array([[0]*4]+[x['wheel_reference_rad_s'] for x in c['trace']]);yaw=np.array([0]+[x['yaw_rate_rad_s'] for x in c['trace']])
            self.assertLessEqual(np.max(abs(np.diff(st,axis=0))),7.330382858*.05+1e-9)
            self.assertLessEqual(np.max(abs(np.diff(wh,axis=0))),24.45*.05+1e-9)
            self.assertLessEqual(np.max(abs(np.diff(yaw))),.4*.05+1e-9)
    def test_zero_candidate_holds_stationary(self):
        z=next(x for x in preview(np.zeros((5,2)),0.,np.ones(4)*.2,np.zeros(4)) if x['goal_yaw_rate_rad_s']==0)
        np.testing.assert_allclose(z['trace'][-1]['steer_reference_rad'],.2)
        self.assertEqual(z['score'],0)
    def test_reject_invalid_speed(self):
        with self.assertRaises(ValueError):preview([[1.,0]],0.,np.zeros(4),np.zeros(4))
