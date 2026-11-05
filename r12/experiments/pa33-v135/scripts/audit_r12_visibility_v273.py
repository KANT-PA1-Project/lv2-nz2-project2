"""Pure diagnostic projection; no collision, occlusion, or controller commands."""
import sys,json,math
from pathlib import Path
import numpy as np
repo=Path(__file__).resolve().parents[1];sys.path.insert(0,str(repo/'scripts/sensor_runtime'))
from camera_rig import transforms
out=repo.parents[1]/'outputs/r12'
rig=transforms(repo,{f'arm_joint{i}':0 for i in range(1,6)})
# Representative authored cuboid tag surfaces,2cm square, body center height100.1mm.
faces=[]
for name,n,u,v,offset in [('top',[0,0,1],[1,0,0],[0,1,0],.03003),('px',[1,0,0],[0,1,0],[0,0,1],.01503),('nx',[-1,0,0],[0,1,0],[0,0,1],.01503),('py',[0,1,0],[1,0,0],[0,0,1],.01503),('ny',[0,-1,0],[1,0,0],[0,0,1],.01503)]:
 n,u,v=map(np.array,(n,u,v));center=np.array([0,0,.1001])+n*offset
 corners=np.array([center+.01*(a*u+b*v) for a,b in [(-1,-1),(1,-1),(1,1),(-1,1)]])
 faces.append((name,n,center,corners))
rows=[]
for name,T in rig.items():
 if 'left' not in name:continue
 # Camera XY may move independently with base translation; extrinsic orientation/height fixed.
 bearing=np.arctan2(T[1,2],T[0,2]);radii=np.linspace(.05,3,591);angles=bearing+np.deg2rad(np.linspace(-80,80,161))
 rr,aa=np.meshgrid(radii,angles,indexing='ij');origins=np.column_stack([-rr.ravel()*np.cos(aa.ravel()),-rr.ravel()*np.sin(aa.ravel()),np.full(rr.size,T[2,3]+.008)])
 record=dict(camera=name,origin_height_m=float(T[2,3]+.008),downward_tilt_deg=float(np.rad2deg(np.arcsin(-T[2,2]))),faces=[])
 for face,n,center,corners in faces:
  pc=(corners[None,:,:]-origins[:,None,:])@T[:3,:3]
  with np.errstate(divide='ignore',invalid='ignore'):uv=pc[:,:,:2]/pc[:,:,2,None]*400+np.array([320,240])
  front=((origins-center)@n)>0
  valid=front&np.all(pc[:,:,2]>.01,axis=1)&np.all((uv[:,:,0]>=0)&(uv[:,:,0]<640)&(uv[:,:,1]>=0)&(uv[:,:,1]<480),axis=1)
  edges=np.linalg.norm(uv-np.roll(uv,1,axis=1),axis=2).min(1)
  ids=np.flatnonzero(valid)
  if not len(ids):record['faces'].append(dict(face=face,fully_visible_samples=0));continue
  best=ids[np.argmax(edges[ids])]
  record['faces'].append(dict(face=face,fully_visible_samples=len(ids),max_min_edge_px=float(edges[best]),best_camera_origin=origins[best].tolist(),best_base_translation=(origins[best]-T[:3,3]).tolist(),best_bbox=[*uv[best].min(0).tolist(),*uv[best].max(0).tolist()],min_visible_horizontal_distance_m=float(np.min(rr.ravel()[ids])),samples_ge16px=int(np.sum(edges[ids]>=16))))
 rows.append(record)
report=dict(cameras=rows,grid=dict(radial_m=[.05,3],radial_samples=591,bearing_offsets_deg=[-80,80],bearing_samples=161),tag_size_m=.02,fx=400,fy=400,frame=[640,480],scope='Sampled fixed-yaw extrinsic geometry envelope for representative planar cuboid tags at100.1mm center height. Ideal frontal/unoccluded surfaces;no robot/collision checks. 16px is descriptive only,not validated decode threshold.',control_observations_used=False,runtime_changed=False)
(out/'v273_visibility_envelope.json').write_text(json.dumps(report,indent=2)+'\n')
for r in rows:print(r['camera'],'height',r['origin_height_m'],'tilt',r['downward_tilt_deg'],[(f['face'],round(f.get('max_min_edge_px',0),2),f.get('samples_ge16px')) for f in r['faces']])
