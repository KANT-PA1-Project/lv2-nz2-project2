import unittest
import numpy as np
from arm_practice.heading_feasibility import choose_heading,steering_margin,world_to_body
class HeadingTest(unittest.TestCase):
 def test_avoids_limit_crossing(self):
  v=[[.08,-.4],[-.08,-.4]];h,m=choose_heading(v)
  self.assertGreater(m,.5);self.assertGreater(m,steering_margin(v,0))
 def test_rotation_preserves_world_vector(self):
  h=.7;v=np.array([.08,-.4]);b=world_to_body(v,h);c,s=np.cos(h),np.sin(h)
  np.testing.assert_allclose(np.array([[c,-s],[s,c]])@b,v)
 def test_stop_preserves_heading(self):
  self.assertEqual(choose_heading([[0,0]],.4),(.4,None))

class TwistHeadingTest(unittest.TestCase):
 def test_zero_yaw_matches_translation_margin(self):
  from arm_practice.heading_feasibility import twist_steering_margin
  v=[[.2,.1],[.1,.3]]
  self.assertAlmostEqual(twist_steering_margin(v,.3,[0]),steering_margin(v,.3))
 def test_rotation_can_move_module_onto_branch(self):
  from arm_practice.heading_feasibility import twist_steering_margin
  v=[[.02,.2]]
  self.assertGreater(twist_steering_margin(v,0,[0]),.09)
  self.assertLess(twist_steering_margin(v,0,[.2]),1e-8)
 def test_proposal_improves_worst_scenario(self):
  from arm_practice.heading_feasibility import twist_steering_margin,choose_twist_heading
  v=[[.02,.2],[-.02,.2]];h,m=choose_twist_heading(v)
  self.assertGreater(m,twist_steering_margin(v,0,[-.2,0,.2]))
  self.assertAlmostEqual(m,twist_steering_margin(v,h,[-.2,0,.2]))
 def test_invalid_scenario_rejected(self):
  from arm_practice.heading_feasibility import twist_steering_margin
  with self.assertRaises(ValueError):twist_steering_margin([[1,0]],0,[float('nan')])
 def test_good_endpoint_can_cross_branch(self):
  from arm_practice.heading_feasibility import twist_steering_margin,heading_transition_margin
  v=[[.2,.2]]
  self.assertGreater(twist_steering_margin(v,0,[0]),.7)
  self.assertGreater(twist_steering_margin(v,np.pi/2,[0]),.7)
  self.assertLess(heading_transition_margin(v,0,np.pi/2,[0]),1e-8)
 def test_connected_proposal_preserves_transition_margin(self):
  from arm_practice.heading_feasibility import choose_transition_heading,heading_transition_margin,twist_steering_margin
  for current in [0,.4,1.1]:
   v=[[.1,.2],[.2,.1]];h,m=choose_transition_heading(v,current)
   floor=min(.06,twist_steering_margin(v,current,[-.2,0,.2]))
   self.assertGreaterEqual(heading_transition_margin(v,current,h),floor-1e-9)
 def test_connected_stop_holds_heading(self):
  from arm_practice.heading_feasibility import choose_transition_heading
  self.assertEqual(choose_transition_heading([[0,0]],.4),(.4,None))
 def test_batch_matches_scalar_scenarios(self):
  from arm_practice.heading_feasibility import _twist_margins_batch,twist_steering_margin
  rng=np.random.default_rng(106);v=rng.normal(size=(12,2));h=np.linspace(-3,3,31);rates=[-.2,0,.2]
  np.testing.assert_allclose(_twist_margins_batch(v,h,rates),[twist_steering_margin(v,x,rates) for x in h],atol=1e-12)
