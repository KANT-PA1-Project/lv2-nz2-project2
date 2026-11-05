"""New explicit offline combination. Does not mutate learned output or admit poses."""
import numpy as np
from .association import associate
from .pick_sensor_adapter import rigid

def shadow(tags,depth,K,*,shape,decoded):
 if shape!='cuboid':raise ValueError('dual_plane_requires_square_prism_not_cylinder')
 objects=[o for o in decoded['detections']if o.get('class_id',99)<4]
 if any('cylinder'in o.get('class_name','')for o in objects):raise ValueError('sensor_shape_conflict')
 paired=associate(tags,depth,K);g=paired['pair']['geometry'];ident=paired['pair']['tag_id']
 geo=decoded['geometry']
 if geo['run_id']!=decoded['run_id']or geo['frame_index']!=decoded['frame']or geo['capture_monotonic_ns']!=decoded['capture_ns']:raise ValueError('FK_identity_unjoined')
 T=rigid(geo['T_base_camera']);tcp=rigid(geo['T_base_tcp']);center=np.asarray(g['center_camera_m'],float);R=np.asarray(g['rotation_matrix'],float)
 if center.shape!=(3,) or not np.isfinite(center).all() or center[2]<=0 or R.shape!=(3,3):raise ValueError('invalid_shadow_pose')
 O=np.eye(4);O[:3,:3]=R;O[:3,3]=center;rigid(O)
 return dict(run_id=decoded['run_id'],frame_index=decoded['frame'],capture_monotonic_ns=decoded['capture_ns'],camera_frame='arm_camera_left_optical',parent_frame='base_link',id=ident,tag_ids=[ident],pose_geometry=g,observed_T_base_object=(T@O).tolist(),T_base_tcp=geo['T_base_tcp'],symmetry='square-prism8; labeled face alignment unresolved',semantic_box_association_validated=False,validated_pose_confidence=None,pose_error_budget=None,control_admitted=False,motion_authorized=False)

def mapping_audit(result,decoded):
 # Available values are diagnostic mappings, not authenticated live envelopes.
 return dict(available={'frame.run_id':result['run_id'],'frame.frame_index':result['frame_index'],'frame.capture_monotonic_ns':result['capture_monotonic_ns'],'packet.camera_transforms.arm_camera_left_optical':decoded['geometry']['T_base_camera'],'pose.center_camera_m':result['pose_geometry']['center_camera_m'],'pose.rotation_matrix':result['pose_geometry']['rotation_matrix'],'detection.tag_ids':result['tag_ids'],'capture_FK.T_base_tcp':result['T_base_tcp']},missing=['live one-use capture_receipt and feedback_receipt','frame pixel_geometry envelope and sensor_inputs join','packet.stamp_ns/perception.stamp_ns binding','perception camera/run/pixel_geometry envelope','confirmed identity evidence_role/stamp and current3frame authority','unique semantic object box association','depth_consistent validated gate','pose translation_valid/rotation_valid admission','validated shadow pose confidence >=0.25','pose_error_budget translation_bound_m <=0.005','pose_error_budget rotation_bound_rad <=0.10 with validated report_sha256','desired_object_rotation_in_tcp and symmetry policy','current local feedback <=100ms at observation completion'],adapter_ready=False,offline_archive_not_live=True)
