import unittest,copy
from arm_practice.raw_visual_collector import RawVisualCollector
import test_raw_visual_evidence as raw_tests
import test_capture_pose_evidence as schema_tests
from arm_practice.capture_pose_evidence import check_capture_pose_evidence
class CollectorTests(unittest.TestCase):
 def setUp(self):
  source=raw_tests.RawVisualTests();source.setUp();self.o,self.s=source.snapshots();self.c=RawVisualCollector('r','b',capacity=2,timeout_ns=100)
 def push(self,k,r,t=200):return self.c.push(k,r,now_ns=t)
 def test_both_arrival_orders(self):
  for order in [('status','odometry'),('odometry','status')]:
   self.setUp();d=dict(odometry=self.o,status=self.s);self.assertEqual(self.push(order[0],d[order[0]])['state'],'pending');r=self.push(order[1],d[order[1]]);self.assertEqual(r['state'],'paired');self.assertFalse(r['pair']['pose_join_verified'])
 def test_duplicates_no_refresh_and_expired_no_resurrection(self):
  self.push('odometry',self.o);d=dict(self.o,received_monotonic_ns=280);self.assertEqual(self.push('odometry',d,280)['state'],'duplicate');self.c.expire(301);self.assertFalse(self.c.pending);self.assertEqual(self.push('status',dict(self.s,received_monotonic_ns=301),301)['state'],'retired')
 def test_conflict_latches_until_new_session(self):
  self.push('odometry',self.o);d=copy.deepcopy(self.o);d['position'][0]=99
  with self.assertRaisesRegex(ValueError,'conflicting'):self.push('odometry',d)
  with self.assertRaises(ValueError):self.push('status',self.s)
  with self.assertRaises(ValueError):self.c.reset('r','b')
  self.c.reset('new','b');self.assertFalse(self.c.faulted)
 def test_bounded_and_old_completed_rejected(self):
  for i in range(3):self.push('odometry',dict(self.o,stamp_ns=1000000000+i))
  self.assertEqual(len(self.c.pending),2);self.assertEqual(self.push('status',self.s)['state'],'retired')
 def test_wrong_session_and_clock_regression(self):
  with self.assertRaises(ValueError):self.push('status',dict(self.s,run_id='wrong'))
  with self.assertRaises(ValueError):self.c.expire(199)
  self.assertTrue(self.c.faulted)
 def test_timeout_boundary(self):
  self.push('odometry',self.o);self.assertEqual(self.push('status',self.s,300)['state'],'paired')
 def test_composed_schema_preserves_raw_frames(self):
  fixture=schema_tests.CapturePoseTests();fixture.setUp();b=fixture.b
  self.o.update(frame_id='odom',child_frame_id='base_link');self.push('odometry',self.o);pair=self.push('status',self.s)['pair'];b['raw_odometry']=pair['raw_odometry']
  for key in ['frame','extrinsic']:b[key].update(run_id='r',clock_epoch='b')
  result=check_capture_pose_evidence(b,expected_calibration_sha256=fixture.h);self.assertTrue(result['schema_consistent']);self.assertFalse(result['pose_join_verified'])
if __name__=='__main__':unittest.main()
