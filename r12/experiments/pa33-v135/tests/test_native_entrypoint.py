import unittest
import numpy as np
from arm_practice.native_rgbd.mission_entrypoint import EncoderVelocityHistory
class VelocityTests(unittest.TestCase):
 def sample(self,h,step,q=0,t=None,start=1000):
  return h.sample(q=np.ones(5)*q,step=step,sim_time_s=step/960 if t is None else t,read_start_ns=start,read_end_ns=start+10)
 def test_first_invalid(self):
  h=EncoderVelocityHistory();self.sample(h,8);self.assertFalse(h.current(1100)['valid'])
 def test_distinct_actual120hz(self):
  h=EncoderVelocityHistory();self.sample(h,8);self.sample(h,16,.01);r=h.current(1100);self.assertTrue(r['valid']);np.testing.assert_allclose(r['velocity'],1.2)
 def test_repeated_step_cannot_refresh(self):
  h=EncoderVelocityHistory();self.sample(h,8)
  with self.assertRaisesRegex(ValueError,'repeated_control_encoder_epoch'):self.sample(h,8,start=10000)
  self.assertEqual(h.latest['read_start_ns'],1000)
 def test_100ms_boundary(self):
  h=EncoderVelocityHistory();self.sample(h,8);self.sample(h,16)
  self.assertTrue(h.current(100_001_000)['valid']);self.assertFalse(h.current(100_001_001)['valid'])
 def test_wrong_rate_invalid(self):
  h=EncoderVelocityHistory();self.sample(h,8);self.sample(h,24);self.assertFalse(h.current(1100)['valid'])
 def test_historical_q_no_alias(self):
  h=EncoderVelocityHistory();q=np.zeros(5);h.sample(q=q,step=8,sim_time_s=8/960,read_start_ns=1000,read_end_ns=1010);q[:]=1;self.assertEqual(h.latest['q'],[0.]*5)
 def test_future_wall_invalid(self):
  h=EncoderVelocityHistory();self.sample(h,8);self.sample(h,16);self.assertFalse(h.current(999)['valid'])
if __name__=='__main__':unittest.main()
