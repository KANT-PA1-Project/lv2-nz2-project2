"""Offline object-relative observation poses; no fixture identities or execution."""
import argparse,json,sys
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'src'))
from arm_practice.kinematics import forward_transform
p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('views',type=Path);p.add_argument('output',type=Path);a=p.parse_args()
rows=json.loads((a.run/'sim_results.json').read_text());pose=np.array(rows[-1]['estimated_map_pose']);views=json.loads(a.views.read_text())['candidates'];rig=json.loads((root/'configs/sensors/camera_rig.json').read_text());vertices=np.array(json.loads((root/'configs/sensors/transport_footprint.json').read_text())['vertices']);grid=np.load(a.run/'costmap.npz');res=float(grid['resolution']);origin=grid['origin'];occupied=(grid['data']>=100)|(grid['data']<0)
def R(y):return np.array([[np.cos(y),-np.sin(y)],[np.sin(y),np.cos(y)]])
# Use existing padded CAD; do not add a second footprint-sized inflation.
hull=ConvexHull(vertices);gx,gy=np.meshgrid(np.arange(vertices[:,0].min(),vertices[:,0].max()+res/2,res/2),np.arange(vertices[:,1].min(),vertices[:,1].max()+res/2,res/2));samples=np.c_[gx.ravel(),gy.ravel()];samples=samples[(samples@hull.equations[:,:2].T+hull.equations[:,2]<=1e-9).all(1)];samples=np.vstack([samples,vertices])
row=next(r for r in reversed(rows) if r.get('diagnostic_lidar_scans'));ex=json.loads((a.run/'lidar_extrinsics.json').read_text());obstacles=[]
for n,scan in row['diagnostic_lidar_scans'].items():
 d=np.array(scan);ang=np.linspace(-np.pi/2,np.pi/2,90);T=np.array(ex[n]);pts=np.c_[d*np.cos(ang),d*np.sin(ang),np.zeros(90)]@T[:3,:3].T+T[:3,3];valid=np.isfinite(d)&(d>.02)&(d<5.95);obstacles.extend(pts[valid,:2]@R(row['estimated_map_pose'][2]).T+row['estimated_map_pose'][:2])
obstacles=np.array(obstacles);local=np.eye(4);local[:3,:3]=Rotation.from_euler('xyz',rig['arm_camera']['optical_rpy_rad']).as_matrix();local[:3,3]=rig['arm_camera']['optical_center_xyz_m'];local[1,3]+=.025
accepted=[];counts={};tested=0
for index,obj in enumerate(views):
 target=np.array(obj['surface_base_m']);world=target[:2]@R(pose[2]).T+pose[:2]
 for yaw in [np.pi]: # Current follower supports fixed yaw only.
  for q1 in np.linspace(-np.pi/2,np.pi/2,25):
   q=[float(q1),-.35,-.35,-.35,-.35];C=forward_transform(root/'models/r12/r12_arm.urdf','arm_link5',dict(zip([f'arm_joint{i}' for i in range(1,6)],q)))@local
   for distance in [.18,.20,.25,.30]:
    tested+=1;axis=(C@np.array([0,0,distance,1]))[:3];xy=world-axis[:2]@R(yaw).T
    rel=C[:3,:3].T@(np.r_[axis[:2],target[2]]-C[:3,3]);uv=np.array([400*rel[0]/rel[2]+320,400*rel[1]/rel[2]+240]) if rel[2]>0 else np.array([-1,-1])
    reason=None
    if not (40<=uv[0]<=600 and 40<=uv[1]<=440):reason='outside_wrist_fov'
    cells=np.floor((samples@R(yaw).T+xy-origin)/res).astype(int)
    if reason is None and ((cells<0).any() or (cells[:,0]>=occupied.shape[1]).any() or (cells[:,1]>=occupied.shape[0]).any()):reason='map_bounds'
    if reason is None and occupied[cells[:,1],cells[:,0]].any():reason='padded_footprint_map_collision'
    body=(obstacles-xy)@R(yaw);d=np.linalg.norm(body,axis=1);normal=body/np.maximum(d[:,None],1e-9);gap=float(np.min(d-np.max(normal@vertices.T,axis=1)))
    if reason is None and gap<.015:reason='observed_lidar_clearance'
    counts[reason or 'candidate']=counts.get(reason or 'candidate',0)+1
    if reason is None:accepted.append(dict(object_candidate=index,base_xy=xy.tolist(),yaw=yaw,arm_q=q,wrist_pixel=uv.tolist(),nominal_tag_px=float(8/rel[2]),min_lidar_support_gap_m=gap,displacement_m=float(np.linalg.norm(xy-pose[:2]))))
accepted.sort(key=lambda r:r['displacement_m']);result=dict(tested=tested,rejection_counts=counts,candidates=accepted[:20],scope='Offline single-snapshot RGB-D hypotheses plus observed lidar and existing Nav2 costmap. Fixed yaw only. No path reachability, arm collision/occlusion, identity or grasp validation. Not executable until these checks pass.')
a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
