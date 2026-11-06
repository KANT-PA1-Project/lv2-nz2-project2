import os,sys,unittest,json,dataclasses
from pathlib import Path
from unittest.mock import patch
import numpy as np
default_root=next((p/'work/manipulator-isaac61-lyrical-practice' for p in Path(__file__).resolve().parents if (p/'work/manipulator-isaac61-lyrical-practice').is_dir()),Path(__file__).resolve().parents[1]/'reference')
root=Path(os.getenv('R12_ADAPTER_REPO',default_root))
sys.path[:0]=[str(root/'src'),str(root/'scripts/sensor_runtime')]
from pick_sensor_adapter import PickSensorAdapter,clearance_from_report,fault_hold_action
from arm_practice.isaac_pixel_geometry import archive_geometry_fields,metadata
from camera_rig import transforms
class Adapter(unittest.TestCase):
 def setUp(self):
  self.patch=patch.dict(os.environ,{'R12_ISAAC_PIXEL_CENTER_V1':'1'});self.patch.start();self.addCleanup(self.patch.stop)
  self.frame=dict(run_id=np.array('r'),frame_index=np.int64(0),stamp_ns=np.int64(1_000_000_000),clock_domain=np.array('synthetic_packet_20hz'),capture_monotonic_ns=np.int64(100_000_000_000),**archive_geometry_fields())
  self.packet=dict(capture_run_id='r',stamp_ns=1_000_000_000,arm_feedback_stamp_ns=1_000_000_000,arm_feedback_valid=True,frame_capture_monotonic_ns=100_000_000_000,packet_received_monotonic_ns=100_010_000_000,arm_feedback_capture_monotonic_ns=100_000_000_000,arm_measured_position=[0]*5,camera_transforms={k:v.tolist() for k,v in transforms(root,{f'arm_joint{i}':0 for i in range(1,6)}).items()})
  pose=dict(translation_valid=True,rotation_valid=True,confidence=.9,center_camera_m=[0,0,.3],rotation_matrix=np.eye(3).tolist())
  self.packet['perception']=dict(run_id='r',stamp_ns=1_000_000_000,capture_monotonic_ns=100_000_000_000,camera='arm_camera',camera_frame='arm_camera_left_optical',pixel_geometry=metadata(),detections=[dict(tag_ids=[3],class_id=0,depth_consistent=True,pose=pose)])
  self.confirmed=dict(evidence_role='wrist_confirmed_identity_with_pose_estimate',camera='arm_camera',camera_frame='arm_camera_left_optical',stamp_ns=1_000_000_000,run_id='r',wrist_consistent_frames=3,id=3)
  capture=100_000_000_000
  epoch=dict(capture_monotonic_ns=capture,encoder_read_start_monotonic_ns=capture,encoder_read_end_monotonic_ns=capture+100000,arm_feedback_capture_monotonic_ns=capture+50000,render_start_monotonic_ns=capture+110000,render_return_monotonic_ns=capture+5000000,readback_return_monotonic_ns=capture+6000000,physics_step_before=480,physics_step_after=480,arm_encoder=[0.]*5,arm_encoder_after_render=[0.]*5)
  self.frame.update({k:np.asarray(value) for k,value in epoch.items()})
  epoch.update(run_id='r',stamp_ns=1_000_000_000)
  self.packet.update(arm_capture_epoch=epoch,arm_feedback_capture_monotonic_ns=capture+50000,arm_feedback_live_freshness_verified=True)
  self.a=PickSensorAdapter(root,pose_error_budget=dict(validated=True,scope='simulation_sensor_pose',report_sha256='0'*64,translation_bound_m=.001))
 def observe(self):return self.a.observe(self.frame,self.packet,self.confirmed,now_monotonic_ns=100_020_000_000,desired_object_rotation_in_tcp=np.eye(3))
 def test_host_epoch_alone_cannot_authorize_pick_observation(self):
  self.assertIsNone(self.observe());self.assertEqual(self.a.reason,'image_capture_association_unverified')
 def test_packet_boolean_cannot_certify_image_epoch(self):
  self.packet['image_capture_association_verified']=True
  self.assertIsNone(self.observe());self.assertEqual(self.a.reason,'image_capture_association_unverified')
 def test_missing_budget_rejected(self):self.a.budget=None;self.assertIsNone(self.observe());self.assertEqual(self.a.reason,'pose_error_budget_missing')
 def test_encoder_time_missing_rejected(self):del self.packet['arm_feedback_capture_monotonic_ns'];self.assertIsNone(self.observe())
 def test_encoder_time_mismatch_rejected(self):self.packet['arm_feedback_capture_monotonic_ns']-=2_000_000;self.assertIsNone(self.observe())
 def test_transform_truth_or_wrong_fk_rejected(self):self.packet['camera_transforms']['arm_camera_left_optical'][0][3]+=.02;self.assertIsNone(self.observe())
 def test_geometry_mismatch_rejected(self):self.packet['perception']['pixel_geometry']['intrinsics'][2]=320.;self.assertIsNone(self.observe())
 def test_rotation_invalid_rejected(self):self.packet['perception']['detections'][0]['pose']['rotation_valid']=False;self.assertIsNone(self.observe())
 def test_no_current_target_rejected(self):self.packet['perception']['detections']=[];self.assertIsNone(self.observe())
 def test_cached_confirmation_rejected_even_with_current_detection(self):
  self.confirmed['stamp_ns']-=50_000_000
  self.assertIsNone(self.observe());self.assertEqual(self.a.reason,'identity_not_from_current_wrist_frame')
 def test_confirmation_missing_stamp_or_wrong_optical_frame_rejected(self):
  for key,value in [('stamp_ns',None),('camera_frame','r07_camera1_left_optical')]:
   previous=self.confirmed[key];self.confirmed[key]=value
   self.assertIsNone(self.observe());self.assertEqual(self.a.reason,'identity_not_from_current_wrist_frame')
   self.confirmed[key]=previous
 def test_top_camera_not_accepted(self):self.packet['perception']['camera']='r07_camera1';self.assertIsNone(self.observe())
 def test_existing_surface_guard_not_clearance(self):
  report=dict(reason='observed_surface_and_nominal_self_sweep_clear',unknown_volume_clearance_verified=False)
  self.assertIsNone(clearance_from_report(report,phase='approach',capture_ns=1,target_id=3,q_start=[0]*5,q_target=[0]*5,width_target=.075,model_sha256='0'*64))
class HoldAdapter(unittest.TestCase):
 def command(self,mode='hold_measured_aperture'):
  return dict(hold=True,cancel_previous_arm_target=True,arm_target=[.9]*5,gripper_hold_mode=mode,gripper_width_m=.027)
 def test_previous_goal_never_reused(self):
  out=fault_hold_action(self.command(),local_q=[.1]*5,local_aperture_m=.05,local_capture_monotonic_ns=100,now_monotonic_ns=100)
  self.assertEqual(out['arm_target'],[.1]*5);self.assertTrue(out['cancel_pending_commands']);self.assertEqual(out['gripper_width_m'],.05)
 def test_held_object_keeps_existing_clamp(self):
  out=fault_hold_action(self.command('retain_existing_clamp'),local_q=[.1]*5,local_aperture_m=.030,local_capture_monotonic_ns=100,now_monotonic_ns=100)
  self.assertEqual(out['gripper_width_m'],.027);self.assertEqual(out['finger_effort_limit_N'],10.)
 def test_stale_local_no_old_motion(self):
  with self.assertRaisesRegex(ValueError,'no_target_emitted'):fault_hold_action(self.command(),local_q=[.1]*5,local_aperture_m=.05,local_capture_monotonic_ns=0,now_monotonic_ns=200_000_000)
if __name__=='__main__':unittest.main(verbosity=2)

class ActuatorFaultResponse(unittest.TestCase):
 def test_cancel_uses_new_joint_read_and_overrides_motion(self):
  from pick_sensor_adapter import apply_fault_hold_response
  response=dict(hold=True,cancel_previous_arm_target=True,gripper_hold_mode='hold_measured_aperture',arm_target=[.9]*5,command=[.6,0,0],wheel_action=[1]*4)
  ticks=iter([100_000_000,101_000_000]);calls=[]
  def read():calls.append(True);return np.array([.1,.2,.3,.4,.5,.02,-.02])
  result=apply_fault_hold_response(response,read_joint_positions=read,arm_indices=list(range(5)),grip_indices=[5,6],cad_zero_aperture_m=.02,monotonic_ns=lambda:next(ticks))
  self.assertEqual(len(calls),1);self.assertEqual(result['arm_target'],[.1,.2,.3,.4,.5]);self.assertEqual(result['command'],[0.]*3);self.assertEqual(result['wheel_action'],[0.]*4);self.assertAlmostEqual(result['gripper_width_m'],.06)
 def test_slow_read_emits_no_response(self):
  from pick_sensor_adapter import apply_fault_hold_response
  ticks=iter([0,100_000_001])
  with self.assertRaisesRegex(ValueError,'fresh_local_feedback'):
   apply_fault_hold_response(dict(hold=True,cancel_previous_arm_target=True,gripper_hold_mode='hold_measured_aperture'),read_joint_positions=lambda:np.zeros(7),arm_indices=list(range(5)),grip_indices=[5,6],cad_zero_aperture_m=.075,monotonic_ns=lambda:next(ticks))
 def test_regular_response_does_not_read_or_modify(self):
  from pick_sensor_adapter import apply_fault_hold_response
  response={'arm_target':[.1]*5}
  def forbidden():raise AssertionError('unexpected read')
  self.assertIs(apply_fault_hold_response(response,read_joint_positions=forbidden,arm_indices=[],grip_indices=[],cad_zero_aperture_m=.02,monotonic_ns=forbidden),response)
