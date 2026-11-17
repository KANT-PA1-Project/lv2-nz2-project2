"""Sampled arm/gripper USD boxes against recorded top-camera RGB-D surfaces."""
import json,sys
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from audit_r12_usd_arm_path import CollisionModel
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'src'))
from arm_practice.kinematics import forward_transform
out=root.parent.parent/'outputs/r12';run=root/'reports/local/diagnostic_arm_feedback_v129'
assert (run/'completion.json').exists()
data=np.load(run/'frame.npz');home=np.array([0,.33,.33,.33,0]);points=[]
for name in ['r07_camera1','r07_camera2','r07_camera3']:
 depth=data['depth'] if name=='r07_camera1' else data[name+'_depth'];rgb=data['rgb'][0] if name=='r07_camera1' else data[name+'_left_optical'];v,u=np.indices(depth.shape);valid=np.isfinite(depth)&(depth>.03)&(depth<5)&(rgb.max(axis=2)>8);z=depth[valid]
 xyz=np.stack([(u[valid]-320)*z/400,(v[valid]-240)*z/400,z],axis=1);T=forward_transform(root/'models/r12/r12_arm.urdf',name+'_left_optical',dict(zip([f'arm_joint{i}' for i in range(1,6)],home)));points.append(xyz@T[:3,:3].T+T[:3,3])
cloud=np.concatenate(points);_,idx=np.unique(np.floor(cloud/.005).astype(int),axis=0,return_index=True);cloud=cloud[idx];tree=cKDTree(cloud);model=CollisionModel()
e=json.loads((root/'reports/local/arm_roundtrip_v141_recorded/evaluation.json').read_text());segments=[]
for r in e['results']:segments.extend([r['home_samples'],r['samples']])
segments.append(e['final_home_samples']);hits=[];count=0;nearest=.2
for segment in segments:
 for sample in segment[::10]+segment[-1:]:
  q=np.array(sample['q']);trans=model.transforms(q);count+=1
  for name,body,local,_ in model.boxes:
   if body=='base_link':continue
   T=trans[body]@local;half=model.sizes[name]/2;ids=tree.query_ball_point(T[:3,3],np.linalg.norm(half)+.2)
   if not ids:continue
   distance=np.linalg.norm(np.maximum(np.abs((cloud[ids]-T[:3,3])@T[:3,:3])-half,0),axis=1);nearest=min(nearest,float(distance.min()))
   if (distance<.03).any():hits.append(dict(sample=count,collider=name,min_distance_m=float(distance.min()),points=int((distance<.03).sum())))
result=dict(complete=True,samples=count,cloud_points=len(cloud),minimum_distance_m_capped_at_0_2=nearest,near_surface_samples=hits,scope='Offline V141 measured encoder poses subsampled at 2Hz plus segment endpoints, against archived V129 top3 RGB-D surfaces in robot base frame; assumes same base/environment. Not fresh environment authorization, no self-point filtering, unseen volume or depth/calibration uncertainty coverage, not continuous sweep or full corrected envelope.')
(out/'v144_archived_surface_audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
