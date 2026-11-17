import unittest,numpy as np
from arm_practice.implicit_hold_support import stationary_support_hold
class Support(unittest.TestCase):
 def setUp(self):self.kw=dict(solver_type="PGS",encoder_window=np.tile([0,.322,.318,.327,0],(11,1)),applied_reference_window=np.tile([0,.33,.33,.33,0],(11,1)),sim_times=np.linspace(0,.5,11),read_capture_monotonic_ns=100,now_monotonic_ns=100,kp=[80.]*5,kd=[8.]*5,previous_explicit_effort=[0.]*5)
 def run_hold(self):return stationary_support_hold(**self.kw)
 def test_static_force_equivalence(self):
  a=self.run_hold();q=np.array([0,.321,.317,.326,0]);old=80*(self.kw['applied_reference_window'][-1]-q);new=80*(np.array(a['hold_measured_target'])-q)+a['feedforward_nm'];np.testing.assert_allclose(old,new,rtol=0,atol=1e-7)
 def test_total_triangle_bound(self):a=self.run_hold();np.testing.assert_allclose(np.abs(a['feedforward_nm'])+a['implicit_max_effort_nm'],[2.]*5)
 def test_changed_applied_reference_rejected(self):self.kw['applied_reference_window'][0,0]=.01;self.assertRaises(ValueError,self.run_hold)
 def test_moving_encoder_rejected(self):self.kw['encoder_window'][-1,0]+=.01;self.assertRaises(ValueError,self.run_hold)
 def test_stale_rejected(self):self.kw['now_monotonic_ns']=100000101;self.assertRaises(ValueError,self.run_hold)
 def test_missing_window_rejected(self):self.kw['encoder_window']=np.zeros((10,5));self.assertRaises(ValueError,self.run_hold)
 def test_wrong_mode_gains_rejected(self):self.kw['kd']=[2.]*5;self.assertRaises(ValueError,self.run_hold)
 def test_existing_feedforward_rejected(self):self.kw['previous_explicit_effort']=[.1]*5;self.assertRaises(ValueError,self.run_hold)
 def test_support_over_budget_rejected(self):self.kw['encoder_window'][:,2]=.315;self.assertRaises(ValueError,self.run_hold)
 def test_nonfinite_rejected(self):self.kw['encoder_window'][0,0]=np.nan;self.assertRaises(ValueError,self.run_hold)
 def test_larger_motor_limit_rejected(self):self.kw['total_limit_nm']=3.;self.assertRaises(ValueError,self.run_hold)
 def test_short_duration_rejected(self):self.kw['sim_times']=np.linspace(0,.4,11);self.assertRaises(ValueError,self.run_hold)
 def test_tgs_rejected(self):self.kw['solver_type']='TGS';self.assertRaises(ValueError,self.run_hold)
 def test_float32_roundtrip_never_exceeds_budget(self):
  rng=np.random.default_rng(48)
  for _ in range(2000):
   bias=rng.uniform(-.999,.999,5)
   self.kw['encoder_window']=np.tile(self.kw['applied_reference_window'][-1]-bias/80,(11,1))
   a=self.run_hold();f=np.asarray(a['feedforward_nm'],np.float32).astype(float);cap=np.asarray(a['implicit_max_effort_nm'],np.float32).astype(float)
   self.assertTrue(np.all(abs(f)+cap<=2.));self.assertFalse(a['runtime_authorized'])
if __name__=='__main__':unittest.main()

