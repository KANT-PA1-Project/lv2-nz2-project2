"""Compare bounded transport candidates using supplied CAD triangles (FCL)."""
from pathlib import Path
import json,sys,itertools,xml.etree.ElementTree as ET
import numpy as np,trimesh,fcl
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'src'))
from arm_practice.kinematics import forward_transform
path=root/'models/r12/r12_mobile_gripper.urdf';robot=ET.parse(path).getroot()
names=['base_link']+[f'arm_link{i}' for i in range(1,6)]+['gripper_left','gripper_right'];objects={};meshes={}
for name in names:
 link=robot.find(f"link[@name='{name}']")
 meshes[name]=trimesh.util.concatenate([trimesh.load_mesh(path.parent/v.find('geometry/mesh').get('filename'),process=False) for v in link.findall('visual')])
 m=meshes[name];model=fcl.BVHModel();model.beginModel(len(m.vertices),len(m.faces));model.addSubModel(m.vertices,m.faces);model.endModel();objects[name]=fcl.CollisionObject(model)
adjacent={frozenset([j.find('parent').get('link'),j.find('child').get('link')]) for j in robot.findall('joint')}
# Fingers belong to the same symmetric mechanism, but their collision remains checked.
def evaluate(q):
 positions={j.get('name'):0. for j in robot.findall('joint')};positions.update({f'arm_joint{i+1}':float(x) for i,x in enumerate(q)});positions.update(gripper_left_joint=.02768613,gripper_right_joint=-.02768613)
 points=[]
 for name,o in objects.items():
  t=forward_transform(path,name,positions);o.setTransform(fcl.Transform(t[:3,:3],t[:3,3]))
  if name!='base_link':points.append(meshes[name].vertices@t[:3,:3].T+t[:3,3])
 hits=[]
 for a,b in itertools.combinations(names,2):
  if frozenset([a,b]) in adjacent:continue
  if fcl.collide(objects[a],objects[b],fcl.CollisionRequest(num_max_contacts=1),fcl.CollisionResult()):hits.append([a,b])
 points=np.concatenate(points);return hits,float(np.linalg.norm(points[:,:2],axis=1).max()),[points.min(0).tolist(),points.max(0).tolist()]
rows=[]
for q in [[0,x,x,x,0] for x in [.20,.25,.30,.33]]:
 hits,radius,bounds=evaluate(q);sweep=[]
 for u in np.linspace(0,1,21):
  pairs,_,_=evaluate(np.array(q)*u)
  if pairs:sweep.append({'fraction':float(u),'pairs':pairs})
 rows.append({'q':q,'arm_xy_radius_m':radius,'arm_bounds_m':bounds,'endpoint_hits':hits,'transition_hits':sweep,'sampled_path_clear':not hits and not sweep});print(rows[-1],flush=True)
valid=[r for r in rows if r['sampled_path_clear']];best=min(valid,key=lambda r:r['arm_xy_radius_m']) if valid else None
report={'candidates':rows,'selected':best,'method':'FCL nonadjacent CAD triangle intersections,21 sampled straight joint transitions from zero; open gripper. Not continuous collision proof or dynamic contact validation.'}
(root/'reports/local/transport_geometry.json').write_text(json.dumps(report,indent=2)+'\n')
if best:(root/'configs/r12_transport.json').write_text(json.dumps({'arm_target':best['q'],'joint_tolerance_rad':.04,'joint_velocity_limit_rad_s':.15,'settle_seconds':.5,'geometry_validated':True,'physics_validated':False,'base_motion_requires_transport':True,'folding_requires_stationary_base':True,'unfolding_requires_stationary_base':True,'scope':'Empty gripper only; carrying-object retention needs separate validation.','selection':'Smallest arm XY radius among four sampled collision-free candidates, not global optimum.'},indent=2)+'\n')
