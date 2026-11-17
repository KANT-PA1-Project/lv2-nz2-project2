"""Reject arm paths intersecting observed RGB-D surfaces; unknown space is unproven.
Requires a completed run so final RGB-D and final estimated pose are not racing.
Never uses simulator poses, fixture identities, or PhysX queries.
"""
import argparse,json,sys,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'src'))
from arm_practice.kinematics import forward_transform


def main():
 p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('candidates',type=Path);p.add_argument('output',type=Path);p.add_argument('--index',type=int,default=0);a=p.parse_args()
 if not (a.run/'completion.json').exists():raise RuntimeError('Completed recorded run required')
 row=json.loads((a.run/'ros_results.json').read_text())[-1]
 if not row.get('finished'):raise RuntimeError('Observation approach did not finish')
 pose=np.array(row['estimated_map_pose']);yaw=Rotation.from_euler('z',pose[2]).as_matrix();world=np.eye(4);world[:3,:3]=yaw;world[:2,3]=pose[:2]
 data=np.load(a.run/'frame.npz');urdf=root/'models/r12/r12_arm.urdf';robot=ET.parse(urdf).getroot();names=[f'arm_joint{i}' for i in range(1,6)];home=np.array([0,.33,.33,.33,0]);points=[]
 for name in ['r07_camera1','r07_camera2','r07_camera3']:
  depth=data['depth'] if name=='r07_camera1' else data[name+'_depth'];rgb=data['rgb'][0] if name=='r07_camera1' else data[name+'_left_optical'];v,u=np.indices(depth.shape);valid=np.isfinite(depth)&(depth>.03)&(depth<5)&(rgb.max(axis=2)>8)
  z=depth[valid];xyz=np.stack([(u[valid]-320)*z/400,(v[valid]-240)*z/400,z],axis=1)
  T=world@forward_transform(urdf,name+'_left_optical',dict(zip(names,home)));points.append(xyz@T[:3,:3].T+T[:3,3])
 cloud=np.concatenate(points);_,idx=np.unique(np.floor(cloud/.005).astype(int),axis=0,return_index=True);cloud=cloud[idx]
 candidate=json.loads(a.candidates.read_text())['candidates'][a.index];target=np.array(candidate['arm_q']);hits=[];nearest=float('inf')
 boxes=[]
 for i in range(1,6):
  name=f'arm_link{i}'
  for col in robot.find(f"link[@name='{name}']").findall('collision'):
   geom=col.find('geometry/box')
   if geom is None:raise RuntimeError('Unsupported collision geometry')
   origin=col.find('origin');T=np.eye(4)
   if origin is not None:
    T[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ');T[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
   boxes.append((name,T,np.fromstring(geom.get('size'),sep=' ')/2))
 for step,q in enumerate(np.linspace(home,target,41)):
  for name,offset,half in boxes:
   T=world@forward_transform(urdf,name,dict(zip(names,q)))@offset;local=(cloud-T[:3,3])@T[:3,:3];outside=np.maximum(np.abs(local)-half,0);distance=np.linalg.norm(outside,axis=1);nearest=min(nearest,float(distance.min()));near=distance<.015
   if near.any():hits.append(dict(sample=step,link=name,observed_points_within15mm=int(near.sum()),minimum_distance_m=float(distance.min())))
 result=dict(run=a.run.name,candidate_index=a.index,cloud_points=len(cloud),samples=41,minimum_observed_surface_distance_m=nearest,near_surface_samples=hits,scope='Observed surface rejection check using URDF collision boxes and15mm buffer. Includes possible visible self points; no unseen-space clearance proof, gripper geometry or continuous swept-volume guarantee. No execution authorization generated.')
 a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':main()
