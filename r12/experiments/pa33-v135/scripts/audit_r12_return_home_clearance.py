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
cloud=np.concatenate(points);_,idx=np.unique(np.floor(cloud/.005).astype(int),axis=0,return_index=True);cloud=cloud[idx];tree=cKDTree(cloud);model=CollisionModel();rows=json.loads((run/'ros_results.json').read_text());start=np.array(rows[-1]['arm_measured_position']);candidate={'waypoints':[[0.,0.,0.,0.,0.]]};hits=[];self_hits=[];count=0;nearest=.2
for goal in map(np.array,candidate['waypoints']):
 for t in np.arange(0,np.max(np.abs(goal-start))/.1+.101,.1):
  q=start+np.clip(goal-start,-.1*t,.1*t);trans=model.transforms(q);count+=1
  if model.hits(q):self_hits.append({'sample':count,'pairs':model.hits(q)})
  for name,body,local,_ in model.boxes:
   if body=='base_link':continue
   T=trans[body]@local;half=model.sizes[name]/2;ids=tree.query_ball_point(T[:3,3],np.linalg.norm(half)+.2)
   if not ids:continue
   distance=np.linalg.norm(np.maximum(np.abs((cloud[ids]-T[:3,3])@T[:3,:3])-half,0),axis=1);nearest=min(nearest,float(distance.min()))
   if (distance<.03).any():hits.append({'sample':count,'collider':name,'min_distance_m':float(distance.min()),'points':int((distance<.03).sum())})
 start=goal
result={'run':run.name,'samples':count,'cloud_points':len(cloud),'minimum_distance_m_capped_at_0_2':nearest,'near_surface_samples':hits,'self_collision_samples':self_hits,'start_q':rows[-1]['arm_measured_position'],'goal_q':[0.,0.,0.,0.,0.],'scope':'Actual USD arm and open gripper cube boxes; top3 RGB-D, base coordinates, 5mm voxels, 30mm buffer. No self-point filtering. Unseen space, depth uncertainty, continuous sweep and PhysX margins unproven.'}
(out/'v130_return_home_clearance.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
