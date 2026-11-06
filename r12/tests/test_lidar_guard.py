import unittest
import numpy as np
from arm_practice.lidar_guard import constrain_velocity

class LidarGuardTests(unittest.TestCase):
    def scene(self, distance):
        # Two central rays bracket a frontal obstacle; all others are no returns.
        scan=np.full(90,6.);scan[44:46]=distance
        return {'front':scan},{'front':np.eye(4)}
    def test_close_obstacle_blocks_approach_but_permits_retreat(self):
        scans,tf=self.scene(.29)
        forward,_=constrain_velocity([.1,0,0],scans,tf)
        retreat,_=constrain_velocity([-.1,0,0],scans,tf)
        self.assertLess(abs(forward[0]),1e-4)
        np.testing.assert_allclose(retreat,[-.1,0,0])
    def test_approach_speed_respects_clearance(self):
        scans,tf=self.scene(.33)
        v,_=constrain_velocity([.2,0,0],scans,tf)
        self.assertLess(v[0],.051)
        self.assertGreater(v[0],0)
    def test_missing_and_nonfinite_scan_stop(self):
        scans,tf=self.scene(.5)
        np.testing.assert_equal(constrain_velocity([.1,0,0],{},tf)[0],0)
        scans['front'][0]=np.nan
        np.testing.assert_equal(constrain_velocity([.1,0,0],scans,tf)[0],0)
    def test_free_space_preserves_command(self):
        scans,tf=self.scene(6.)
        np.testing.assert_allclose(constrain_velocity([.1,-.1,.2],scans,tf)[0],[.1,-.1,.2])

class PolygonGuardTests(unittest.TestCase):
 def test_side_margin_reduced_but_forward_gripper_protected(self):
  from arm_practice.lidar_guard import constrain_velocity
  footprint=[[-.22,-.22],[.38,-.22],[.38,.22],[-.22,.22]]
  ranges=np.full(90,6.);ranges[44:46]=.27
  transform=np.eye(4);transform[:2,:2]=[[0,-1],[1,0]]
  circle,_=constrain_velocity([0,.1,0],{'s':ranges},{'s':transform})
  polygon,_=constrain_velocity([0,.1,0],{'s':ranges},{'s':transform},footprint=footprint)
  self.assertGreater(polygon[1],circle[1]+.02)
  ranges[44:46]=.35
  forward,_=constrain_velocity([.1,0,0],{'s':ranges},{'s':np.eye(4)},footprint=footprint)
  self.assertLessEqual(forward[0],1e-6)  # Retreat/tangential escape is allowed; forward approach is not.
