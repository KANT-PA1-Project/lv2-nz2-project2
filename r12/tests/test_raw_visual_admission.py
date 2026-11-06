import copy,unittest
from arm_practice.raw_visual_admission import RawVisualAdmission
class TestAdmission(unittest.TestCase):
 def setUp(self):
  self.a=RawVisualAdmission('r','e',max_receipt_age_ns=50)
  self.o=dict(run_id='r',clock_epoch='e',clock_domain='synthetic_packet_20hz',stamp_ns=100,received_monotonic_ns=1000,source_topic='/visual_slam/tracking/odometry',unmodified=True,frame_id='odom',child_frame_id='base_link',position=[0,0,0],quaternion_xyzw=[0,0,0,1],covariance=[0.]*36)
  self.s={k:v for k,v in self.o.items() if k in ('run_id','clock_epoch','clock_domain','stamp_ns','received_monotonic_ns')};self.s.update(source_topic='/visual_slam/status',vo_state=1)
 def push(self,k,r,t=1000,expected=100):return self.a.push(k,r,now_ns=t,expected_stamp_ns=expected)
 def test_pair_once_and_no_mutation(self):
  original=copy.deepcopy(self.o);self.push('odometry',self.o);r=self.push('status',self.s);self.assertTrue(r['structurally_eligible']);self.assertFalse(r['execution_authorized']);r['raw_odometry']['position'][0]=9;self.assertEqual(self.o,original);self.assertEqual(self.push('status',self.s)['state'],'retired')
 def test_duplicate_does_not_refresh(self):
  self.push('odometry',self.o);late=dict(self.o,received_monotonic_ns=1040);self.assertEqual(self.push('odometry',late,1040)['state'],'duplicate');self.assertEqual(self.push('status',dict(self.s,received_monotonic_ns=1051),1051)['state'],'retired')
 def test_late_pair_current_stamp_mismatch_terminal(self):
  self.push('odometry',self.o);r=self.push('status',self.s,expected=101);self.assertFalse(r['structurally_eligible']);self.assertEqual(r['reason'],'stamp_mismatch');self.assertEqual(self.push('status',self.s)['state'],'retired')
 def test_reset_rejects_old_session(self):
  self.push('odometry',self.o);self.a.reset('r','new')
  with self.assertRaises(ValueError):self.push('status',self.s)
  self.o['clock_epoch']=self.s['clock_epoch']='new';self.push('odometry',self.o);self.assertTrue(self.push('status',self.s)['structurally_eligible'])
 def test_conflict_latches_until_new_epoch(self):
  self.push('odometry',self.o);bad=copy.deepcopy(self.o);bad['position'][0]=1
  with self.assertRaises(ValueError):self.push('odometry',bad)
  with self.assertRaises(ValueError):self.push('status',self.s)
 def test_invalid_tracking_and_raw_frame(self):
  self.o['child_frame_id']='camera';self.push('odometry',self.o);self.assertFalse(self.push('status',self.s)['structurally_eligible'])
  self.a.reset('new','e');self.o.update(run_id='new',child_frame_id='base_link');self.s.update(run_id='new',vo_state=2);self.push('status',self.s);self.assertEqual(self.push('odometry',self.o)['reason'],'tracking_not_valid')
 def test_clock_regression_and_argument_validation(self):
  with self.assertRaises(ValueError):self.push('odometry',self.o,expected=True)
  self.push('odometry',self.o)
  with self.assertRaises(ValueError):self.push('status',self.s,t=999)
