"""Regress the S2 premature-zero branch using the real ROS arrival expression."""
import ast
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import numpy as np
from arm_practice.goal_stop_hold import GoalStopHold, goal_arrival_ready

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts/sensor_runtime'))
from path_follower import follow


class CaptureRoutingTests(unittest.TestCase):
    def bridge_arrival(self, distance, speed, hold):
        tree = ast.parse((ROOT/'scripts/r12_sensor_ros.py').read_text())
        expression = next(n.test for n in ast.walk(tree) if isinstance(n,ast.If)
                          and isinstance(n.test,ast.Call) and isinstance(n.test.func,ast.Name)
                          and n.test.func.id == 'goal_arrival_ready')
        scope = dict(np=np,goal_arrival_ready=goal_arrival_ready,
                     xy=np.array([distance,0.]),goal=np.zeros(2),goal_stop=hold,
                     est=NS(twist=NS(twist=NS(linear=NS(x=speed,y=0.)))))
        return eval(compile(ast.Expression(expression),'<arrival branch>','eval'),scope)

    def test_inside_arrival_but_outside_capture_keeps_approach_branch(self):
        hold=GoalStopHold(capture_radius_m=.01)
        hold.update(0,0,.019,.01,True,True)
        self.assertFalse(self.bridge_arrival(.019,.01,hold))

    def test_captured_stop_can_dwell_through_small_rebound(self):
        hold=GoalStopHold(capture_radius_m=.01)
        hold.update(0,0,.009,.01,False,True)
        hold.update(50_000_000,0,.016,0.,True,True)
        self.assertTrue(self.bridge_arrival(.016,0.,hold))
        self.assertFalse(self.bridge_arrival(.02,0.,hold))

    def test_stale_or_timed_out_hold_cannot_start_dwell(self):
        hold=GoalStopHold(timeout_s=.1,capture_radius_m=.01)
        hold.update(0,0,.009,0.,True,True)
        hold.update(50_000_000,0,.009,0.,True,False)
        self.assertFalse(self.bridge_arrival(.009,0.,hold))
        hold.update(200_000_000,0,.009,0.,True,True)
        self.assertFalse(self.bridge_arrival(.009,0.,hold))

    def test_without_latch_legacy_bounds_are_unchanged(self):
        self.assertTrue(self.bridge_arrival(.019,.014,None))
        self.assertFalse(self.bridge_arrival(.019,.015,None))
        self.assertFalse(self.bridge_arrival(.02,.014,None))

    def test_bridge_passes_braking_endpoint_inside_capture(self):
        tree=ast.parse((ROOT/'scripts/r12_sensor_ros.py').read_text())
        call=next(n for n in ast.walk(tree) if isinstance(n,ast.Call)
                  and isinstance(n.func,ast.Name) and n.func.id=='follow')
        expression=next(k.value for k in call.keywords if k.arg=='braking_stop_distance_m')
        hold=GoalStopHold(capture_radius_m=.01)
        distance=eval(compile(ast.Expression(expression),'<braking endpoint>','eval'),dict(goal_stop=hold))
        self.assertEqual(distance,.005)
        with patch('path_follower.constrain_velocity',side_effect=lambda v,*a,**k:(v,None)):
            velocity,_,_=follow([[.5,.6],[.6,.6]],[.59,.6],np.pi,[.6,.6],
                                np.ones((100,100)),.02,.12,{}, {},braking_stop_distance_m=distance)
        self.assertGreater(np.linalg.norm(velocity[:2]),.001)
        self.assertLessEqual(np.linalg.norm(velocity[:2]),.00800001)

    def test_capture_approach_still_respects_lidar_veto(self):
        with patch('path_follower.constrain_velocity',return_value=(np.zeros(3),'invalid_sensor')):
            velocity,reason,_=follow([[.5,.6],[.6,.6]],[.59,.6],np.pi,[.6,.6],
                                    np.ones((100,100)),.02,.12,{}, {},braking_stop_distance_m=.005)
        np.testing.assert_array_equal(velocity,0.)
        self.assertEqual(reason,'invalid_sensor')


if __name__=='__main__':
    unittest.main()
