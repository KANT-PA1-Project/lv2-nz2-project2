"""Bound supplied URDF collision boxes over steering and transport arm poses."""
import json,sys,itertools,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial import ConvexHull
root=Path(__file__).resolve().parents[1]
urdf=ET.parse(root/'models/r12/r12_mobile_gripper.urdf').getroot()
joints=urdf.findall('joint');bychild={j.find('child').get('link'):j for j in joints}
links=urdf.findall('link');allpoints=[];per_link={}
def origin(el):
 t=np.eye(4)
 if el is not None:t[:3,3]=np.fromstring(el.get('xyz','0 0 0'),sep=' ');t[:3,:3]=Rotation.from_euler('xyz',np.fromstring(el.get('rpy','0 0 0'),sep=' ')).as_matrix()
 return t
cfg=json.loads((root/'configs/r12_transport.json').read_text());goal=np.array(cfg['arm_target']);arm_poses=[(goal+np.array(signs)*cfg['joint_tolerance_rad']).tolist() for signs in itertools.product([-1,1],repeat=5)]
for steering,wheel,arm in itertools.product(np.linspace(-np.pi/2,np.pi/2,19),[0,np.pi/4,np.pi/2],arm_poses):
 q={j.get('name'):0. for j in joints}
 for name in q:
  if name.endswith('_steer_joint'):q[name]=steering
  if name.endswith('_wheel_joint'):q[name]=wheel
 for i,x in enumerate(arm):q[f'arm_joint{i+1}']=x
 q['gripper_left_joint']=.02768613;q['gripper_right_joint']=-.02768613
 cache={}
 def fk(name):
  if name not in cache:
   j=bychild.get(name)
   if j is None:cache[name]=np.eye(4)
   else:
    t=origin(j.find('origin'));motion=np.eye(4)
    if j.get('type')!='fixed':
     axis=np.fromstring(j.find('axis').get('xyz'),sep=' ');axis/=np.linalg.norm(axis)
     if j.get('type')=='prismatic':motion[:3,3]=axis*q[j.get('name')]
     else:motion[:3,:3]=Rotation.from_rotvec(axis*q[j.get('name')]).as_matrix()
    cache[name]=fk(j.find('parent').get('link'))@t@motion
  return cache[name]
 for link in links:
  name=link.get('name')
  for col in link.findall('collision'):
   box=col.find('geometry/box')
   cylinder=col.find('geometry/cylinder')
   if box is not None:size=np.fromstring(box.get('size'),sep=' ')
   elif cylinder is not None:size=np.array([2*float(cylinder.get('radius'))]*2+[float(cylinder.get('length'))])
   else:raise RuntimeError('Unsupported collision geometry: '+name)
   corners=np.array(list(itertools.product([-1,1],repeat=3)))*size/2
   t=fk(name)@origin(col.find('origin'));points=corners@t[:3,:3].T+t[:3,3]
   allpoints.append(points);per_link.setdefault(name,[]).append(points)
points=np.concatenate(allpoints);report={'source':'URDF collision boxes; not measured hardware envelope','sampled_steer_degrees':list(np.linspace(-90,90,19)),'arm_poses_rad':arm_poses,'gripper_width_m':.075,'bounds_m':[points.min(0).tolist(),points.max(0).tolist()],'max_xy_radius_m':float(np.linalg.norm(points[:,:2],axis=1).max()),'per_link_max_xy_radius_m':{name:float(np.linalg.norm(np.concatenate(v)[:,:2],axis=1).max()) for name,v in per_link.items()},'limitations':['Sampled steering/arm poses, not continuous swept-volume proof.','URDF collision boxes may overbound CAD visual geometry.','Arbitrary arm extension requires a separate dynamic collision envelope.']}
hull=points[ConvexHull(points[:,:2]).vertices,:2]
padded=(hull[:,None,:]+np.array(list(itertools.product([-.03,.03],repeat=2)))[None,:,:]).reshape(-1,2)
padded=padded[ConvexHull(padded).vertices]
report['transport_footprint_xy_m']=hull.tolist();report['padded_footprint_xy_m']=padded.tolist();report['padding_m']=.03
(root/'configs/sensors/transport_footprint.json').write_text(json.dumps({'vertices':padded.tolist(),'padding_m':.03,'source':'Sampled CAD collision geometry at transport arm poses and all sampled steering angles; includes open gripper.','restriction':'Base translation permitted only with transport arm held; other arm poses need regenerated envelope.'},indent=2)+'\n')
out=root.parents[1]/'outputs/r12/footprint_audit.json';out.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
