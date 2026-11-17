"""Composed USD enabled cube colliders and joint frames; sampled self-collision.
No fixture poses are used. Not a PhysX contact or continuous collision certificate.
"""
import json
from pathlib import Path
import numpy as np,fcl
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom,UsdPhysics
root=Path(__file__).resolve().parents[1]

class CollisionModel:
 def __init__(self):
  stage=Usd.Stage.Open(str(root/'models/r12/r12_mobile_gripper.usda'));self.joints=[];self.skip=set();self.boxes=[];self.sizes={}
  self.bodies={'base_link',*[f'arm_link{i}' for i in range(1,6)],'gripper_left','gripper_right'}
  for p in stage.Traverse():
   if p.IsA(UsdPhysics.Joint):
    j=UsdPhysics.Joint(p);b0=j.GetBody0Rel().GetTargets();b1=j.GetBody1Rel().GetTargets()
    if not b0 or not b1 or b0[0].name not in self.bodies or b1[0].name not in self.bodies:continue
    def local(n):
     T=np.eye(4);T[:3,3]=p.GetAttribute(f'physics:localPos{n}').Get();q=p.GetAttribute(f'physics:localRot{n}').Get();T[:3,:3]=Rotation.from_quat([*q.GetImaginary(),q.GetReal()]).as_matrix();return T
    self.joints.append((p.GetName(),b0[0].name,b1[0].name,local(0),local(1),p.GetAttribute('physics:axis').Get(),p.IsA(UsdPhysics.PrismaticJoint)))
    if not j.GetCollisionEnabledAttr().Get():self.skip.add(frozenset([b0[0].name,b1[0].name]))
  cache=UsdGeom.XformCache()
  for p in stage.Traverse():
   if not p.HasAPI(UsdPhysics.CollisionAPI) or not UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get():continue
   body=p.GetParent()
   while body and body.GetName() not in self.bodies:body=body.GetParent()
   if not body:continue
   if not p.IsA(UsdGeom.Cube):raise ValueError(f'Unsupported active collider {p.GetPath()}')
   M=np.array(cache.ComputeRelativeTransform(p,body)[0]).T;scale=np.linalg.norm(M[:3,:3],axis=0);M[:3,:3]/=scale;size=UsdGeom.Cube(p).GetSizeAttr().Get()*scale
   self.sizes[str(p.GetPath())]=size
   self.boxes.append((str(p.GetPath()),body.GetName(),M,fcl.CollisionObject(fcl.Box(*size))))
 def transforms(self,q):
  values={f'arm_joint{i+1}':v for i,v in enumerate(q)};values.update(gripper_left_joint=.02768613,gripper_right_joint=-.02768613)
  result={'base_link':np.eye(4)};pending=list(self.joints)
  while pending:
   progress=False
   for j in pending[:]:
    name,b0,b1,L0,L1,axis,prism=j
    if b0 not in result:continue
    M=np.eye(4);v=values.get(name,0);ax=np.eye(3)['XYZ'.index(axis)]
    if prism:M[:3,3]=ax*v
    else:M[:3,:3]=Rotation.from_rotvec(ax*v).as_matrix()
    result[b1]=result[b0]@L0@M@np.linalg.inv(L1);pending.remove(j);progress=True
   if not progress:raise ValueError('Disconnected joint chain')
  return result
 def hits(self,q):
  T=self.transforms(q)
  for name,body,local,obj in self.boxes:
   M=T[body]@local;obj.setTransform(fcl.Transform(M[:3,:3],M[:3,3]))
  hits=[]
  for i,(n,b,_,o) in enumerate(self.boxes):
   for n2,b2,_,o2 in self.boxes[i+1:]:
    if b==b2 or frozenset([b,b2]) in self.skip:continue
    if fcl.collide(o,o2,fcl.CollisionRequest(),fcl.CollisionResult()):hits.append([n,n2])
  return hits

if __name__=='__main__':
 model=CollisionModel();home=np.array([0,.33,.33,.33,0]);goal=np.array([-np.pi/2,-.35,-.35,-.35,-.35]);waypoint=home.copy();waypoint[0]=goal[0]
 rows=json.loads((root/'reports/local/diagnostic_wrist_hold_v30/sim_results.json').read_text());stalled=rows[-1]['arm_measured_position']
 paths={'simultaneous':np.linspace(home,goal,41),'rotate_folded_then_lower':np.concatenate([np.linspace(home,waypoint,41),np.linspace(waypoint,goal,41)[1:]])}
 result={'active_cube_count':len(model.boxes),'stalled_hits':model.hits(stalled),'home_hits':model.hits(home),'goal_hits':model.hits(goal),'paths':{},'scope':'Composed enabled USD cube colliders and joint transforms incl open gripper. Adjacent collision-disabled pairs excluded. Sampled self-collision only; environment and PhysX margins not covered.'}
 for name,path in paths.items():
  hits=[{'sample':i,'pairs':h} for i,q in enumerate(path) if (h:=model.hits(q))];result['paths'][name]={'samples':len(path),'collision_samples':hits}
 out=root.parent.parent/'outputs/r12/v30_usd_collision_audit.json';out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
