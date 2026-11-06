import unittest
from types import SimpleNamespace as N
from copy import deepcopy
from arm_practice.raw_visual_evidence import snapshot_raw_odometry,snapshot_raw_status,pair_raw_visual_evidence
class RawVisualTests(unittest.TestCase):
 def setUp(self):
  self.odom=N(header=N(stamp=N(sec=1,nanosec=0),frame_id='source_odom'),child_frame_id='source_base',pose=N(pose=N(position=N(x=1.,y=2.,z=3.),orientation=N(x=0.,y=0.,z=0.,w=1.)),covariance=[0.]*36))
  self.status=N(header=N(stamp=N(sec=1,nanosec=0)),vo_state=1);self.kw=dict(run_id='r',clock_epoch='b',received_ns=200)
 def snapshots(self):return snapshot_raw_odometry(self.odom,**self.kw),snapshot_raw_status(self.status,**self.kw)
 def test_later_ros_mutations_do_not_rewrite_original(self):
  o,s=self.snapshots();self.odom.header.frame_id='odom';self.odom.child_frame_id='base_link';self.odom.pose.covariance[0]=.0025;self.odom.pose.pose.position.x=99
  self.assertEqual(o['frame_id'],'source_odom');self.assertEqual(o['child_frame_id'],'source_base');self.assertEqual(o['covariance'][0],0);self.assertEqual(o['position'][0],1)
 def test_pair_preserves_source_and_is_not_authority(self):
  o,s=self.snapshots();p=pair_raw_visual_evidence(o,s);self.assertTrue(p['raw_odometry']['tracking_valid']);self.assertEqual(p['raw_odometry']['frame_id'],'source_odom')
  for k in ['source_authenticity_verified','pose_join_verified','execution_authorized']:self.assertFalse(p[k])
 def test_pair_output_not_alias(self):
  o,s=self.snapshots();p=pair_raw_visual_evidence(o,s);o['covariance'][0]=99;s['vo_state']=0;self.assertEqual(p['raw_odometry']['covariance'][0],0);self.assertEqual(p['raw_odometry']['raw_status']['vo_state'],1)
 def test_order_independent_receipts_same_stamp(self):
  o,s=self.snapshots();s['received_monotonic_ns']=500;self.assertTrue(pair_raw_visual_evidence(o,s)['raw_odometry']['tracking_valid'])
 def test_wrong_stamp_run_epoch_source_rejected(self):
  o,s=self.snapshots()
  for k,v in [('stamp_ns',1050000000),('run_id','x'),('clock_epoch','x'),('clock_domain','wall'),('source_topic','fixture')]:
   t=deepcopy(s);t[k]=v
   with self.assertRaises(ValueError):pair_raw_visual_evidence(o,t)
 def test_failed_tracking_kept_not_upgraded(self):
  o,s=self.snapshots()
  for v in [0,2,-1,True]:
   s['vo_state']=v;self.assertFalse(pair_raw_visual_evidence(o,s)['raw_odometry']['tracking_valid'])
 def test_bad_stamp_receipt_and_session(self):
  self.odom.header.stamp.nanosec=10**9
  with self.assertRaises(ValueError):snapshot_raw_odometry(self.odom,**self.kw)
  self.odom.header.stamp.nanosec=0
  for k,v in [('received_ns',True),('run_id',''),('clock_epoch',None)]:
   kw=dict(self.kw);kw[k]=v
   with self.assertRaises(ValueError):snapshot_raw_odometry(self.odom,**kw)
if __name__=='__main__':unittest.main()
