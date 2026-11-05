import unittest,copy
import numpy as np
from arm_practice.capture_pose_evidence import check_capture_pose_evidence
class CapturePoseTests(unittest.TestCase):
 def setUp(self):
  self.h='a'*64;session=dict(run_id='run',clock_epoch='boot1',stamp_ns=1000000000)
  self.b=dict(frame=dict(**session,clock_domain='synthetic_packet_20hz',frame_index=0,capture_monotonic_ns=100,camera='r07_camera2',resolution=[1920,1440]),raw_odometry=dict(**session,clock_domain='synthetic_packet_20hz',received_monotonic_ns=200,source_topic='/visual_slam/tracking/odometry',tracking_valid=True,unmodified=True,frame_id='odom',child_frame_id='base_link',position=[0,0,0],quaternion_xyzw=[0,0,0,1],covariance=np.eye(6).tolist()),calibration=dict(camera='r07_camera2',sha256=self.h,resolution=[1920,1440],pose_frame_id='odom',K=[[1200,0,960],[0,1200,720],[0,0,1]]),extrinsic=dict(**session,calibration_sha256=self.h,parent_frame_id='base_link',child_frame_id='r07_camera2_left_optical',T_parent_child=np.eye(4).tolist()))
 def check(self):return check_capture_pose_evidence(self.b,expected_calibration_sha256=self.h)
 def test_synthetic_valid_is_not_verified_pose_or_motion(self):
  r=self.check();self.assertTrue(r['schema_consistent'])
  for k in ['pose_join_verified','current_observation_eligible','execution_authorized']:self.assertFalse(r[k])
 def test_every_required_field_missing_rejected(self):
  original=copy.deepcopy(self.b)
  for group in original:
   for key in original[group]:
    with self.subTest(group=group,key=key):
     self.b=copy.deepcopy(original);del self.b[group][key];self.assertFalse(self.check()['schema_consistent'])
 def test_cross_run_epoch_stamp_and_camera_rejected(self):
  original=copy.deepcopy(self.b)
  for group,key,value in [('raw_odometry','run_id','other'),('extrinsic','clock_epoch','boot2'),('raw_odometry','stamp_ns',1050000000),('frame','camera','arm_camera'),('extrinsic','child_frame_id','r07_camera1_left_optical')]:
   self.b=copy.deepcopy(original);self.b[group][key]=value;self.assertFalse(self.check()['schema_consistent'])
 def test_wrong_calibration_and_resolution(self):
  self.b['calibration']['sha256']='b'*64;self.assertFalse(self.check()['schema_consistent']);self.b['calibration']['sha256']=self.h;self.b['frame']['resolution']=[1280,960];self.assertFalse(self.check()['schema_consistent'])
 def test_untrusted_pose_fields(self):
  original=copy.deepcopy(self.b)
  for key,v in [('unmodified',False),('tracking_valid',False),('source_topic','fixture'),('frame_id','map'),('quaternion_xyzw',[0,0,0,2]),('position',[float('nan'),0,0]),('received_monotonic_ns',99)]:
   self.b=copy.deepcopy(original);self.b['raw_odometry'][key]=v;self.assertFalse(self.check()['schema_consistent'])
 def test_invalid_covariance_and_transform(self):
  self.b['raw_odometry']['covariance'][0][0]=-1;self.assertFalse(self.check()['schema_consistent']);self.setUp();self.b['extrinsic']['T_parent_child'][0][0]=-1;self.assertFalse(self.check()['schema_consistent'])
 def test_flat_ros_covariance_supported_without_modification(self):
  self.b['raw_odometry']['covariance']=np.eye(6).flatten().tolist();before=copy.deepcopy(self.b);self.assertTrue(self.check()['schema_consistent']);self.assertEqual(self.b,before)
 def test_legacy_record_rejected(self):self.assertFalse(check_capture_pose_evidence({'visual_pose':{}},expected_calibration_sha256=self.h)['schema_consistent'])
if __name__=='__main__':unittest.main()
