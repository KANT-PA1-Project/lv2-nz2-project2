import unittest
import numpy as np
from arm_practice.steering_transition import brake_before_large_turn

class TransitionTest(unittest.TestCase):
    def test_rolling_reversal_brakes_only_affected_module(self):
        a,w,b=brake_before_large_turn([.1,.1,-1.2,-1.2],[0,0,-1.57,-1.57],[12,12,0,0],[12,12,12,-12])
        np.testing.assert_array_equal(b,[False,False,True,True])
        np.testing.assert_allclose(a,[.1,.1,-1.57,-1.57])
        np.testing.assert_allclose(w,[12,12,0,0])
    def test_low_speed_releases_steering(self):
        a,w,b=brake_before_large_turn([.4]*4,[0]*4,[0]*4,[1.9,-1.9,0,0])
        self.assertFalse(b.any());np.testing.assert_allclose(a,[.4]*4)
    def test_aligned_travel_unchanged(self):
        a,w,b=brake_before_large_turn([.1]*4,[0]*4,[10]*4,[12]*4)
        self.assertFalse(b.any());np.testing.assert_allclose(w,[10]*4)

class ProjectionTest(unittest.TestCase):
    def test_reversal_keeps_rolling_sign_before_heading_crosses(self):
        from arm_practice.steering_transition import project_current_heading
        w=project_current_heading([-.08,-.4,0],[0,0,-np.pi/2,-np.pi/2])
        self.assertGreater(w[2],0);self.assertLess(w[3],0)
    def test_zero_command_stops_all_wheels(self):
        from arm_practice.steering_transition import project_current_heading
        np.testing.assert_array_equal(project_current_heading([0,0,0],[.1,.2,.3,.4]),np.zeros(4))
    def test_projection_changes_continuously_across_heading(self):
        from arm_practice.steering_transition import project_current_heading
        a=project_current_heading([-.08,-.4,0],[0,0,-1e-7,-1e-7])
        b=project_current_heading([-.08,-.4,0],[0,0,1e-7,1e-7])
        self.assertLess(np.max(abs(a-b)),1e-5)

class CoordinationTest(unittest.TestCase):
    def test_limits_turn_when_drive_cannot_follow(self):
        from arm_practice.steering_transition import project_current_heading,coordinate_turn_with_drive
        cur=np.array([0.,0.,-.2,-.2]);target=cur+.36;command=[-.08,-.4,0]
        measured=project_current_heading(command,cur)
        a,w=coordinate_turn_with_drive(command,cur,target,measured)
        self.assertLess(a[2],target[2]);self.assertTrue(np.all(np.abs(w-measured)<=24.45*.05+1e-9))
    def test_unreachable_demand_holds_and_reports_mismatch(self):
        from arm_practice.steering_transition import coordinate_turn_with_drive
        cur=np.zeros(4);measured=np.ones(4)*10
        a,w,status=coordinate_turn_with_drive([0,0,0],cur,np.ones(4)*.3,measured,return_status=True)
        np.testing.assert_allclose(a,cur)
        self.assertTrue(all(status['held_for_drive']))
        np.testing.assert_allclose(w,measured-24.45*.05)
        self.assertGreater(max(abs(np.array(status['projection_residual_rad_s']))),0)
    def test_invalid_input_rejected(self):
        from arm_practice.steering_transition import coordinate_turn_with_drive
        for dt in [0,-1,float('nan')]:
            with self.assertRaises(ValueError):coordinate_turn_with_drive([0,0,0],np.zeros(4),np.zeros(4),np.zeros(4),dt=dt)
        with self.assertRaises(ValueError):coordinate_turn_with_drive([0,0,0],np.zeros(4),np.zeros(4),np.ones(4)*30)
