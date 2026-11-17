import unittest
import numpy as np
from arm_practice.latest_policy import allocate
class SteerRateTests(unittest.TestCase):
 def test_bound_and_unchanged_wheel_request(self):
  current=np.zeros(4)
  a,w=allocate([.1,.2,.1],current,current,np.zeros(4))
  b,z=allocate([.1,.2,.1],current,current,np.zeros(4),steer_rate=3.665191429)
  self.assertLessEqual(np.max(abs(b-current)),3.665191429*.05+1e-12)
  self.assertGreater(np.max(abs(a-current)),np.max(abs(b-current)))
  np.testing.assert_allclose(w,z)
 def test_reject_invalid_rate(self):
  for v in [0,-1,8,float('nan')]:
   with self.assertRaises(ValueError):allocate([.1,0,0],np.zeros(4),None,np.zeros(4),steer_rate=v)
