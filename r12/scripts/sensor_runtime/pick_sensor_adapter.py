"""Strict candidate bridge from existing wrist perception packets to pick inputs.

No truth fallback or blanket collision waiver. A validated simulation pose-error
budget and current swept-volume certificate are external prerequisites. Without
these the adapter returns no observation/clearance and records the missing input.
"""
import json,math
from pathlib import Path
import numpy as np
from arm_practice.pick_place_cycle import Observation,Clearance
from arm_practice.kinematics import forward_transform
from arm_practice.isaac_pixel_geometry import require_archive_geometry,metadata
from arm_practice.frame_packet_identity import check_live_frame_provenance
from camera_rig import transforms
from arm_practice.capture_joint_epoch import require_frame_epoch

def rigid(T):
 T=np.asarray(T,float)
 if T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0,0,0,1],atol=1e-9) or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-6) or abs(np.linalg.det(T[:3,:3])-1)>1e-6:raise ValueError('invalid_rigid_transform')
 return T

class PickSensorAdapter:
 def __init__(self,root,*,pose_error_budget=None):self.root=Path(root);self.budget=pose_error_budget;self.reason='await_input'
 def observe(self,frame,packet,confirmed,*,now_monotonic_ns,desired_object_rotation_in_tcp):
  try:
   require_archive_geometry(frame)
   capture_verdict=require_frame_epoch(frame,packet,now_monotonic_ns=now_monotonic_ns)
   index=int(np.asarray(frame['frame_index']).item());run=str(np.asarray(frame['run_id']).item())
   verdict=check_live_frame_provenance(frame,packet,index,run,now_monotonic_ns,500_000_000)
   if not verdict['matched']:raise ValueError('frame:'+verdict['reason'])
   perception=packet['perception'];capture=int(np.asarray(frame['capture_monotonic_ns']).item());encoder_capture=packet['arm_feedback_capture_monotonic_ns']
   if type(encoder_capture) is not int or abs(capture-encoder_capture)>1_000_000:raise ValueError('encoder_capture_time_unjoined')
   if perception['run_id']!=run or perception['stamp_ns']!=packet['stamp_ns'] or perception['capture_monotonic_ns']!=capture or perception['camera']!='arm_camera' or perception['camera_frame']!='arm_camera_left_optical':raise ValueError('perception_identity_mismatch')
   if perception.get('pixel_geometry')!=metadata():raise ValueError('perception_geometry_mismatch')
   if (confirmed.get('evidence_role')!='wrist_confirmed_identity_with_pose_estimate' or confirmed.get('camera')!='arm_camera' or confirmed.get('run_id')!=run or confirmed.get('wrist_consistent_frames',0)<3 or confirmed.get('invalidated',False)):raise ValueError('identity_unconfirmed')
   if (confirmed.get('camera_frame')!='arm_camera_left_optical' or
       type(confirmed.get('stamp_ns')) is not int or confirmed['stamp_ns']!=perception['stamp_ns']):raise ValueError('identity_not_from_current_wrist_frame')
   target=confirmed['id'];candidates=[d for d in perception['detections'] if d.get('tag_ids')==[target] and d.get('class_id',99)<4]
   if len(candidates)!=1:raise ValueError('target_not_unique_in_current_wrist_frame')
   d=candidates[0];pose=d['pose']
   if d.get('depth_consistent') is not True or pose.get('translation_valid') is not True or pose.get('rotation_valid') is not True or float(pose['confidence'])<.25:raise ValueError('pose_depth_rotation_unverified')
   if not self.budget or self.budget.get('validated') is not True or self.budget.get('scope')!='simulation_sensor_pose' or not self.budget.get('report_sha256'):raise ValueError('pose_error_budget_missing')
   error=float(self.budget['translation_bound_m'])
   if not math.isfinite(error) or not 0<=error<=.005:raise ValueError('pose_error_budget_too_large')
   measured=np.asarray(packet['arm_measured_position'],float)
   if measured.shape!=(5,) or not np.isfinite(measured).all():raise ValueError('invalid_arm_encoder')
   joints={f'arm_joint{i+1}':v for i,v in enumerate(measured)};T=rigid(transforms(self.root,joints)['arm_camera_left_optical']);reported=rigid(packet['camera_transforms']['arm_camera_left_optical'])
   if not np.allclose(T,reported,atol=1e-8):raise ValueError('reported_camera_transform_differs_from_encoder_fk')
   tcp=rigid(forward_transform(self.root/'models/r12/r12_arm.urdf','arm_link5',joints));cfg=json.loads((self.root/'configs/r12_tcp_calibration.json').read_text());offset=np.eye(4);offset[:3,3]=cfg['translation_m'];tcp=tcp@offset
   center=np.asarray(pose['center_camera_m'],float);R=np.asarray(pose['rotation_matrix'],float);desired=np.asarray(desired_object_rotation_in_tcp,float)
   if center.shape!=(3,) or not np.isfinite(center).all() or center[2]<=0:raise ValueError('invalid_object_center')
   object_T=np.eye(4);object_T[:3,:3]=R;object_T[:3,3]=center;rigid(object_T);desired_T=np.eye(4);desired_T[:3,:3]=desired;rigid(desired_T)
   aligned=tcp[:3,:3].T@T[:3,:3]@R;angle=float(np.arccos(np.clip((np.trace(desired.T@aligned)-1)/2,-1,1)))
   # Host read timing cannot certify GPU pixels from the current physics epoch.
   if capture_verdict.get('image_capture_association_verified') is not True:raise ValueError('image_capture_association_unverified')
   obs=Observation(run,index,capture,encoder_capture,target,'arm_camera_left_optical','wrist_rgbd_encoder_fk',tuple((T@np.r_[center,1])[:3]),tuple(tcp[:3,3]),True,True,error,angle)
   self.reason='schema_joined_not_clearance_authorized';return obs
  except (KeyError,ValueError,TypeError,OverflowError,IndexError) as e:self.reason=str(e);return None

def clearance_from_report(report,*,phase,capture_ns,target_id,q_start,q_target,width_target,model_sha256):
 """Existing WristSurfaceGuard.surface_clear is intentionally insufficient."""
 try:
  if report.get('evidence_role')!='sensor_swept_volume_certificate' or report.get('unknown_volume_clearance_verified') is not True or report.get('sweep_phase')!=phase or report.get('capture_monotonic_ns')!=capture_ns or report.get('interval_matches_requested_command') is not True:return None
  if report.get('model_sha256')!=model_sha256 or type(model_sha256) is not str or len(model_sha256)!=64:return None
  if not np.array_equal(np.asarray(report['q_start']),np.asarray(q_start)) or not np.array_equal(np.asarray(report['q_target']),np.asarray(q_target)) or report.get('width_target_m')!=width_target:return None
  margin=float(report['external_cad_margin_m'])
  if not np.isfinite(margin) or margin<.03:return None
  contact=report.get('intended_contact')
  if contact is not None and (contact.get('target_id')!=target_id or contact.get('bodies')!=['left_pad','right_pad']):return None
  return Clearance(True,capture_ns,phase,True,margin,target_id if contact else None,('left_pad','right_pad') if contact else ())
 except (KeyError,TypeError,ValueError):return None


def fault_hold_action(command, *, local_q, local_aperture_m, local_capture_monotonic_ns, now_monotonic_ns):
    """Adapter must call with its own fresh joint read, never an old target.

    Missing local feedback raises before any position target is emitted. The
    actuator loop must cancel queued plans and enter its local measured-hold
    watchdog; merely omitting arm_target in the current ROS protocol is unsafe.
    """
    if command.get('hold') is not True or command.get('cancel_previous_arm_target') is not True:
        raise ValueError('not_a_fault_hold_command')
    q=np.asarray(local_q,float)
    if q.shape!=(5,) or not np.isfinite(q).all() or type(local_capture_monotonic_ns) is not int or type(now_monotonic_ns) is not int or not 0<=now_monotonic_ns-local_capture_monotonic_ns<=100_000_000 or not np.isfinite(local_aperture_m) or not .019<=local_aperture_m<=.076:
        raise ValueError('fresh_local_feedback_required_no_target_emitted')
    mode=command.get('gripper_hold_mode')
    if mode=='retain_existing_clamp':
        width=command.get('gripper_width_m')
        if type(width) not in (int,float) or not .020<=width<=.075:raise ValueError('invalid_previous_clamp')
    elif mode=='hold_measured_aperture':width=float(np.clip(local_aperture_m,.020,.075))
    else:raise ValueError('invalid_gripper_hold_mode')
    return dict(arm_mode='measured_hold',arm_target=q.tolist(),arm_velocity_target=[0.]*5,
                gripper_width_m=width,gripper_mode=mode,cancel_pending_commands=True,
                base_command=[0.]*3,arm_effort_limit_Nm=2.,finger_effort_limit_N=10.,
                source='fresh_local_encoder_hold',hardware_authorized=False)


def apply_fault_hold_response(response, *, read_joint_positions, arm_indices,
                              grip_indices, cad_zero_aperture_m, monotonic_ns):
    """Resolve explicit cancellation using actuator-local feedback after RPC.

    A failed read raises before another physics step; no cached target fallback.
    Ordinary responses are untouched. This does not enable a pick mission.
    """
    if response.get('cancel_previous_arm_target') is not True:
        return response
    captured = monotonic_ns()
    measured = np.asarray(read_joint_positions(), dtype=float)
    action = fault_hold_action(
        response, local_q=measured[arm_indices],
        local_aperture_m=cad_zero_aperture_m + measured[grip_indices[0]] - measured[grip_indices[1]],
        local_capture_monotonic_ns=captured, now_monotonic_ns=monotonic_ns())
    return {**response, 'arm_target': action['arm_target'],
            'gripper_width_m': action['gripper_width_m'], 'command': [0., 0., 0.],
            'wheel_action': [0.] * 4, 'wheel_scale': 1.,
            'stationary_wheel_hold_torque': [0.] * 4, 'stop_brake_valid': False,
            'applied_fault_hold': action}
