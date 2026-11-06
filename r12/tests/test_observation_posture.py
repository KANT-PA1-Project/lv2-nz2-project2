import unittest,numpy as np
from arm_practice.observation_posture import ObservationPosture
class Tests(unittest.TestCase):
 def make(self):return ObservationPosture([.5,0,0,0,0],[-1]*5,[1]*5)
 def test_bounds(self):
  with self.assertRaises(ValueError):ObservationPosture([2]*5,[-1]*5,[1]*5)
 def test_wait(self):
  o=self.make();self.assertIsNone(o.step(0,[0]*5,[0]*5,[0]*3,False,True));self.assertEqual(o.state,'waiting')
 def test_rate_and_drift(self):
  o=self.make();o.step(0,[0]*5,[0]*5,[0]*3,True,True);o.step(.5,[0]*5,[0]*5,[0]*3,True,True);q=o.step(.55,[0]*5,[0]*5,[0]*3,True,True);self.assertAlmostEqual(q[0],.005);o.step(.6,q,[0]*5,[.02,0,0],True,True);self.assertEqual(o.state,'failed')
 def test_invalid_stops(self):
  o=self.make();o.step(0,[0]*5,[0]*5,[0]*3,True,True);o.step(.5,[0]*5,[0]*5,[0]*3,True,True);self.assertIsNone(o.step(.55,[0]*5,[0]*5,[0]*3,True,False));self.assertEqual(o.state,'paused');self.assertFalse(o.done)
 def test_completion_after_settle_and_exposure(self):
  o=self.make();q=np.zeros(5)
  for i in range(250):
   target=o.step(i*.05,q,[0]*5,[0]*3,True,True)
   if target is not None:q=target
  self.assertEqual(o.state,'complete');self.assertLess(abs(q[0]-.5),1e-9)
 def test_clock_gap(self):
  o=self.make();o.step(0,[0]*5,[0]*5,[0]*3,True,True);o.step(.5,[0]*5,[0]*5,[0]*3,True,True);o.step(1,[0]*5,[0]*5,[0]*3,True,True);self.assertEqual(o.reason,'clock_gap')
 def test_pause_recovery_no_command_jump(self):
  o=self.make();o.step(0,[0]*5,[0]*5,[0]*3,True,True);o.step(.5,[0]*5,[0]*5,[0]*3,True,True);o.step(.55,[0]*5,[0]*5,[0]*3,True,False)
  q=o.step(.6,[0]*5,[0]*5,[0]*3,True,True);np.testing.assert_array_equal(q,np.zeros(5));self.assertEqual(o.state,'moving')
 def test_persistent_sensor_loss(self):
  o=self.make();o.step(0,[0]*5,[0]*5,[0]*3,True,True);o.step(.5,[0]*5,[0]*5,[0]*3,True,True);o.step(.55,[0]*5,[0]*5,[0]*3,True,False);o.step(1.6,[0]*5,[0]*5,[0]*3,True,False);self.assertEqual(o.reason,'sensor_timeout')
 def test_waypoint_sequence(self):
  o=ObservationPosture([.2,.2,0,0,0],[-1]*5,[1]*5,[[.2,0,0,0,0],[.2,.2,0,0,0]]);q=np.zeros(5)
  for i in range(240):
   target=o.step(i*.05,q,[0]*5,[0]*3,True,True)
   if target is not None:q=target
   if o.waypoint_index==0:self.assertEqual(q[1],0)
  self.assertEqual(o.state,'complete');self.assertEqual(o.waypoint_index,1)
unittest.main()
