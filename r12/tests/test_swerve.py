import unittest,numpy as np,torch
from arm_practice.swerve import teacher,actuate,wheel_odometry,RADIUS
class Exact(torch.nn.Module):
 def forward(self,x):return torch.tensor(teacher(x.numpy()),dtype=torch.float32)
class SwerveTests(unittest.TestCase):
 def test_rotation(self):
  a=teacher(np.array([0,0,1.])).reshape(4,2)
  np.testing.assert_allclose(a,[[-.1,.1],[.1,.1],[-.1,-.1],[.1,-.1]])
 def test_zero_holds(self):
  q=np.array([.3,-.2,.4,-.1]);a,w=actuate(Exact(),[0,0,0],q);np.testing.assert_equal(a,q);np.testing.assert_equal(w,np.zeros(4))
 def test_limits(self):
  rng=np.random.default_rng(9)
  for _ in range(1000):
   q=rng.uniform(-np.pi/2,np.pi/2,4);a,w=actuate(Exact(),rng.uniform(-2,2,3),q)
   self.assertTrue(np.all(abs(a)<=np.pi/2));self.assertTrue(np.all(abs(a-q)<=.05000001));self.assertTrue(np.all(abs(w)<=10))
 def test_nan(self):
  with self.assertRaises(ValueError):actuate(Exact(),[np.nan,0,0],np.zeros(4))

class OdometryTests(unittest.TestCase):
 def test_all_encoders_forward(self):
  twist,residual=wheel_odometry([np.pi/2,np.pi/2,0,0],np.array([.12,.12,.12,-.12])/RADIUS)
  np.testing.assert_allclose(twist,[.12,0,0],atol=1e-12);self.assertLess(residual,1e-12)
 def test_missing_encoder_rejected(self):
  with self.assertRaises(ValueError):wheel_odometry([0]*4,[0]*3)
