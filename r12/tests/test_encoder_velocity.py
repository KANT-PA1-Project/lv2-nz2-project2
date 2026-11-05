import unittest
import numpy as np
from arm_practice.encoder_velocity import wrapped_velocity
class EncoderTest(unittest.TestCase):
 def test_four_pi_wrap_both_directions(self):
  np.testing.assert_allclose(wrapped_velocity([.1,-.1],[4*np.pi-.5,-4*np.pi+.5],.05),[12,-12])
 def test_unwrapped_motion(self):
  np.testing.assert_allclose(wrapped_velocity([2.6,-2.6],[2.,-2.],.05),[12,-12])
 def test_invalid_dt(self):
  with self.assertRaises(ValueError):wrapped_velocity([0],[0],0)
