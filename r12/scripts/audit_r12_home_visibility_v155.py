"""Sparse home-path visibility diagnostic; unknown samples cannot certify free space."""
import json,itertools
from pathlib import Path
import numpy as np
from audit_r12_usd_arm_path import CollisionModel
from arm_practice.kinematics import forward_transform
root=Path(__file__).resolve().parents[1];run=root/'reports/local/diagnostic_base_feedback_v153';data=np.load(run/'frame.npz',allow_pickle=False);rows=json.loads((run/'ros_results.json').read_text());start=np.array(rows[-1]['arm_measured_position']);model=CollisionModel();cameras=[]
for name in ['r07_camera1','r07_camera2','r07_camera3']:
 T=forward_transform(root/'models/r12/r12_arm.urdf',name+'_left_optical',dict(zip([f'arm_joint{i}' for i in range(1,6)],start)))
 cameras.append((T,data['depth'] if name=='r07_camera1' else data[name+'_depth']))
counts={'observed_free':0,'not_confirmed_free':0};examples=[];poses=int(np.ceil(np.max(abs(start))/.01))+1
for i,t in enumerate(np.linspace(0,1,poses)):
 q=start*(1-t);trans=model.transforms(q)
 for name,body,local,_ in model.boxes:
  if body=='base_link':continue
  T=trans[body]@local;half=model.sizes[name]/2+.03
  probes=np.array(list(itertools.product([-1,1],repeat=3))+[(0,0,0)])*half;points=probes@T[:3,:3].T+T[:3,3];free=np.zeros(len(points),bool)
  for C,depth in cameras:
   pc=(points-C[:3,3])@C[:3,:3];z=pc[:,2];valid=z>.03;uv=np.zeros((len(points),2),int);uv[valid]=np.rint(pc[valid,:2]/z[valid,None]*400+[320,240]).astype(int)
   valid&=(uv[:,0]>=0)&(uv[:,0]<depth.shape[1])&(uv[:,1]>=0)&(uv[:,1]<depth.shape[0]);ids=np.where(valid)[0];d=depth[uv[ids,1],uv[ids,0]];free[ids]|=np.isfinite(d)&(d>.03)&(z[ids]<d-.03)
  counts['observed_free']+=int(free.sum());counts['not_confirmed_free']+=int((~free).sum())
  if not free.all() and len(examples)<12:examples.append(dict(pose=i,collider=name,unknown_probes=int((~free).sum())))
e=dict(complete=True,poses=poses,probe_counts=counts,examples=examples,execution_authorized=False,scope='Top3 depth ray visibility of 9 sparse points per expanded arm/gripper box, 3cm expansion plus depth margin. Self masks/occlusion/invalid depth treated as not confirmed free. Not whole swept-volume or collision proof; archive replay only.')
(root.parent.parent/'outputs/r12/v155_home_visibility.json').write_text(json.dumps(e,indent=2)+'\n');print(json.dumps(e))
