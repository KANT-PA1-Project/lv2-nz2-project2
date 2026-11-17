import unittest
import numpy as np
from arm_practice.capture_bundle_identity import inspect_capture_bundle

class CaptureBundleTest(unittest.TestCase):
 def setUp(self):
  self.d=dict(run_id=np.array('run-a'),clock_domain=np.array('synthetic_packet_20hz'),frame_index=np.int64(2),stamp_ns=np.int64(1100000000),capture_monotonic_ns=np.int64(123),rgb=np.zeros((2,480,640,3),np.uint8))
  self.d.update({f'r07_camera{c}_{s}_optical':np.zeros((480,640,3),np.uint8) for c in (2,3) for s in ('left','right')})
  self.p=dict(capture_run_id='run-a',stamp_ns=1100000000,frame_capture_monotonic_ns=123)
 def test_matching_not_authorized(self):
  r=inspect_capture_bundle(self.d,self.p);self.assertTrue(r['matched']);self.assertEqual(len(r['image_sha256']),6);self.assertFalse(r['pose_join_verified']);self.assertFalse(r['execution_authorized'])
 def test_same_stamp_previous_run(self):
  self.p['capture_run_id']='run-b';self.assertFalse(inspect_capture_bundle(self.d,self.p)['matched'])
 def test_overwritten_bundle(self):
  self.d['frame_index']=np.int64(3);self.d['stamp_ns']=np.int64(1150000000);self.assertFalse(inspect_capture_bundle(self.d,self.p)['matched'])
 def test_capture_mismatch(self):
  self.p['frame_capture_monotonic_ns']=124;self.assertFalse(inspect_capture_bundle(self.d,self.p)['matched'])
 def test_missing_or_wrong_image(self):
  del self.d['r07_camera3_right_optical'];self.assertFalse(inspect_capture_bundle(self.d,self.p)['matched'])
 def test_hash_changes(self):
  before=inspect_capture_bundle(self.d,self.p)['image_sha256'];self.d['rgb'][0,0,0,0]=1;after=inspect_capture_bundle(self.d,self.p)['image_sha256'];self.assertNotEqual(before['r07_camera1_left_optical'],after['r07_camera1_left_optical'])
 def test_invalid_scalar(self):
  self.d['frame_index']=np.array([2]);self.assertFalse(inspect_capture_bundle(self.d,self.p)['matched'])
