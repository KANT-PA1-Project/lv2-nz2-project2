"""Offline visible RGB-D surface height; no fixture poses or runtime changes."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from audit_r12_usd_arm_path import CollisionModel
root=Path(__file__).resolve().parents[1];out=root.parent.parent/'outputs/r12';run=root/'reports/local/diagnostic_wrist_waypoints_v31'
assert (run/'completion.json').exists()
frame=np.load(run/'frame.npz');depth=frame['arm_camera_depth'];rgb=frame['arm_camera_left_optical'].astype(float)
obs=json.loads((run/'offline_wrist_multiscale.json').read_text());obj=next(d for d in obs['detections'] if d.get('tag_ids')==[11] and d.get('depth_consistent'))
row=json.loads((run/'ros_results.json').read_text())[-1];q=row['arm_measured_position'];rig=json.loads((root/'configs/sensors/camera_rig.json').read_text())['arm_camera']
C=np.eye(4);C[:3,:3]=Rotation.from_euler('xyz',rig['optical_rpy_rad']).as_matrix();C[:3,3]=rig['optical_center_xyz_m'];C[1,3]+=rig['stereo_baseline_m']/2;C=CollisionModel().transforms(q)['arm_link5']@C
v,u=np.indices(depth.shape);x0,y0,x1,y1=obj['bbox'];valid=np.isfinite(depth)&(depth>.03)&(depth<5);box=valid&(u>=x0)&(u<=x1)&(v>=y0)&(v<=y1)
red=box&(rgb[:,:,0]>40)&(rgb[:,:,0]>1.5*rgb[:,:,1])&(rgb[:,:,0]>1.5*rgb[:,:,2])
bound=json.loads((out/'v123_gripper_reach_interval.json').read_text())['level_base_pad_height_lower_m'];results={}
for name,mask in [('all_valid_detection_bbox',box),('red_surface_subset',red)]:
 z=depth[mask];xyz=np.stack([(u[mask]-320)*z/400,(v[mask]-240)*z/400,z],axis=1);base=xyz@C[:3,:3].T+C[:3,3];h=base[:,2];assert len(h)>0
 results[name]={'points':len(h),'height_min_max_m':[float(h.min()),float(h.max())],'height_percentiles_1_50_99_m':np.percentile(h,[1,50,99]).tolist(),'pad_lower_bound_minus_highest_point_m':float(bound-h.max())}
center=(C@np.r_[obj['pose']['center_camera_m'],1])[:3];prior=json.loads((out/'v31_observed_reach.json').read_text())['results'][0]['target_base_m'];assert np.allclose(center,prior,atol=1e-9)
report={'run':run.name,'observed_tag_id':11,'camera':'arm_camera_left_optical','arm_encoder_q_rad':q,'camera_intrinsics':[400,400,320,240],'surface_sets':results,'conditional_pad_height_lower_m':bound,'prior_center_transform_regression_passed':True,'scope':'Recorded visible RGB-D points only. Bounding box includes background and red subset is a color heuristic, not certified segmentation. Unseen object surfaces, depth/hand-eye/encoder uncertainty and synchronization not bounded. Last recorded encoder row follows previous reach audit; no new synchronized acquisition. Original model limits and level base; not full-object grasp infeasibility or hardware certification. No fixture/PhysX truth used.'}
(out/'v125_observed_surface_height.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
