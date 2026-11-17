import unittest,copy
import test_raw_visual_evidence as fixtures
from arm_practice.raw_visual_callback import RawVisualCallback
class TestCallback(unittest.TestCase):
 def setUp(self):
  f=fixtures.RawVisualTests();f.setUp();self.o=f.odom;self.o.header.frame_id='odom';self.o.child_frame_id='base_link';self.s=f.status;self.gid=dict(implementation_identifier='rmw',data=list(range(16)));self.a=RawVisualCallback('g','r','e',expected_gids={k:self.gid for k in ('odometry','status')},max_receipt_age_ns=50)
 def receive(self,k,m,**kw):
  args=dict(callback_generation='g',received_ns=200,now_ns=200,expected_stamp_ns=1000000000);args.update(kw);return self.a.receive(k,m,{'publisher_gid':self.gid},**args)
 def test_snapshot_independent_and_pair(self):
  r=self.receive('odometry',self.o);self.o.pose.pose.position.x=99;r['received_snapshot']['position'][0]=88
  pair=self.receive('status',self.s);self.assertTrue(pair['structurally_eligible']);self.assertEqual(pair['raw_odometry']['position'][0],1.);self.assertFalse(pair['source_session_verified'])
 def test_old_callback_rejected_without_relabel(self):
  with self.assertRaises(ValueError):self.receive('odometry',self.o,callback_generation='old')
  self.assertFalse(self.a.faulted)
 def test_endpoint_change_latches(self):
  self.receive('odometry',self.o);self.gid['data'][0]=99
  with self.assertRaises(ValueError):self.receive('status',self.s)
  self.gid['data'][0]=0
  with self.assertRaises(ValueError):self.receive('status',self.s)
 def test_missing_gid_latches(self):
  with self.assertRaises(KeyError):self.a.receive('odometry',self.o,{},callback_generation='g',received_ns=200,now_ns=200,expected_stamp_ns=1000000000)
  self.assertTrue(self.a.faulted)
 def test_no_receipt_refresh(self):
  self.receive('odometry',self.o);self.receive('odometry',self.o,received_ns=240,now_ns=240)
  r=self.receive('status',self.s,received_ns=251,now_ns=251);self.assertEqual(r['state'],'retired');self.assertFalse(r['structurally_eligible'])
 def test_configuration_copied_and_recreated(self):
  self.gid['data'][0]=99;self.assertEqual(self.a.expected['odometry'][1][0],0)
  with self.assertRaises(ValueError):RawVisualCallback('g','r','e',expected_gids={})
