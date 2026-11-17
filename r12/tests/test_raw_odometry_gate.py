import copy,unittest
from arm_practice.raw_odometry_gate import check_raw_odometry
class TestRawOdometryGate(unittest.TestCase):
 def setUp(self):
  self.r=dict(stamp_ns=100,received_monotonic_ns=1000,source_topic='/visual_slam/tracking/odometry',unmodified=True,frame_id='odom',child_frame_id='base_link',position=[0,0,0],quaternion_xyzw=[0,0,0,1],covariance=[0.]*36)
 def check(self,**kw):
  args=dict(expected_stamp_ns=100,last_consumed_stamp_ns=99,now_ns=1100,max_receipt_age_ns=100);args.update(kw);return check_raw_odometry(self.r,**args)
 def test_valid_boundary_not_authorization(self):
  before=copy.deepcopy(self.r);r=self.check();self.assertTrue(r['structurally_eligible']);self.assertFalse(r['execution_authorized']);self.assertFalse(r['capture_freshness_verified']);self.assertEqual(before,self.r)
 def test_late_future_receipt(self):
  for now in (1101,999):self.assertFalse(self.check(now_ns=now)['structurally_eligible'])
 def test_reuse_regression_mismatch(self):
  for kw in ({'last_consumed_stamp_ns':100},{'last_consumed_stamp_ns':101},{'expected_stamp_ns':101}):self.assertFalse(self.check(**kw)['structurally_eligible'])
 def test_frames_not_relabelled(self):
  for key in ('frame_id','child_frame_id'):
   old=self.r[key];self.r[key]='wrong';self.assertFalse(self.check()['structurally_eligible']);self.assertEqual(self.r[key],'wrong');self.r[key]=old
 def test_bad_covariance(self):
  for value in (-.1,float('nan'),float('inf')):
   self.r['covariance'][21]=value;self.assertFalse(self.check()['structurally_eligible'])
  self.r['covariance']=[0.]*36;self.r['covariance'][1]=.1;self.assertFalse(self.check()['structurally_eligible'])
  self.r['covariance'][6]=.1;self.assertFalse(self.check()['structurally_eligible'])
 def test_bad_pose(self):
  self.r['quaternion_xyzw']=[0,0,0,0];self.assertFalse(self.check()['structurally_eligible']);self.r['quaternion_xyzw']=[0,0,0,1];self.r['position'][0]=float('nan');self.assertFalse(self.check()['structurally_eligible'])
 def test_missing_and_bad_types(self):
  self.assertFalse(self.check(now_ns=True)['structurally_eligible']);del self.r['unmodified'];self.assertFalse(self.check()['structurally_eligible'])
