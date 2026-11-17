"""Sparse home-path visibility diagnostic; unknown samples cannot certify free space."""
import json,itertools,sys
from pathlib import Path
import numpy as np
from audit_r12_usd_arm_path import CollisionModel
from arm_practice.kinematics import forward_transform
root=Path(__file__).resolve().parents[1];run=root/'reports/local/diagnostic_base_feedback_v153';data=np.load(run/'frame.npz',allow_pickle=False);rows=json.loads((run/'ros_results.json').read_text());start=np.array(rows[-1]['arm_measured_position']);model=CollisionModel();cameras=[]
sys.path.insert(0,str(root/'scripts/sensor_runtime'))
from camera_rig import transforms
rig=transforms(root,dict(zip([f'arm_joint{i}' for i in range(1,6)],start)))
for name in ['r07_camera1','r07_camera2','r07_camera3','arm_camera']:
 cameras.append((rig[name+'_left_optical'],data['depth'] if name=='r07_camera1' else data[name+'_depth']))
start_trans=model.transforms(start);start_boxes=[(start_trans[body]@local,model.sizes[name]/2) for name,body,local,_ in model.boxes]
counts={'observed_free_external':0,'not_confirmed_free_external':0,'inside_start_model':0};wrist_added=0;unknown_reasons={'outside_all_fov':0,'no_valid_depth_in_fov':0,'behind_or_within_depth_margin':0};examples=[];poses=int(np.ceil(np.max(abs(start))/.01))+1
for i,t in enumerate(np.linspace(0,1,poses)):
 q=start*(1-t);trans=model.transforms(q)
 for name,body,local,_ in model.boxes:
  if body=='base_link':continue
  T=trans[body]@local;half=model.sizes[name]/2+.03
  probes=np.array(list(itertools.product([-1,1],repeat=3))+[(0,0,0)])*half;points=probes@T[:3,:3].T+T[:3,3];free=np.zeros(len(points),bool)
  free_top=None;any_fov=np.zeros(len(points),bool);any_depth=np.zeros(len(points),bool)
  for camera_index,(C,depth) in enumerate(cameras):
   pc=(points-C[:3,3])@C[:3,:3];z=pc[:,2];valid=z>.03;uv=np.zeros((len(points),2),int);uv[valid]=np.rint(pc[valid,:2]/z[valid,None]*400+[320,240]).astype(int)
   valid&=(uv[:,0]>=0)&(uv[:,0]<depth.shape[1])&(uv[:,1]>=0)&(uv[:,1]<depth.shape[0]);any_fov|=valid;ids=np.where(valid)[0];d=depth[uv[ids,1],uv[ids,0]];any_depth[ids]|=np.isfinite(d)&(d>.03);free[ids]|=np.isfinite(d)&(d>.03)&(z[ids]<d-.03)
   if camera_index==2:free_top=free.copy()
  inside=np.zeros(len(points),bool)
  for B,half_start in start_boxes:inside|=np.all(np.abs((points-B[:3,3])@B[:3,:3])<=half_start,axis=1)
  external=~inside;unknown=external&~free
  unknown_reasons['outside_all_fov']+=int((unknown&~any_fov).sum());unknown_reasons['no_valid_depth_in_fov']+=int((unknown&any_fov&~any_depth).sum());unknown_reasons['behind_or_within_depth_margin']+=int((unknown&any_depth).sum())
  wrist_added+=int((free&~free_top&external).sum())
  counts['observed_free_external']+=int((free&external).sum());counts['not_confirmed_free_external']+=int((~free&external).sum());counts['inside_start_model']+=int(inside.sum())
  if (~free&external).any() and len(examples)<12:examples.append(dict(pose=i,collider=name,unknown_probes=int((~free&external).sum())))
e=dict(complete=True,poses=poses,probe_counts=counts,unknown_reasons=unknown_reasons,wrist_added_external_probes=wrist_added,examples=examples,execution_authorized=False,scope='Top3 plus wrist depth ray visibility using runtime camera_rig transforms of 9 sparse points per expanded arm/gripper box, 3cm expansion plus depth margin. Starting-model interior reported separately, not treated as observed free or clearance authorization. Self masks/occlusion/invalid depth treated as not confirmed free. Not whole swept-volume or collision proof; archive replay only.')
(root.parent.parent/'outputs/r12/v157_visibility_reasons.json').write_text(json.dumps(e,indent=2)+'\n');print(json.dumps(e))
