from pathlib import Path
import sys, json, xml.etree.ElementTree as ET
import numpy as np
import trimesh
from pxr import Usd, UsdGeom, UsdPhysics
root=Path('/home/pa33/git/p2_perception/Claude outputs/tower_signal')
sys.path.insert(0,str(root/'sim_assembly'))
from common import load_config,base_transform,homogeneous,rpy_matrix,_floats,ARM_LINKS,CHASSIS_LINKS
from plan_poses import tower_target
cfg,model=load_config(root/'sim_assembly/config_auto_mast_relocate_30cm.json')
source=Path(cfg['assembly_dir'])/'assembly.urdf'
robot=ET.parse(source).getroot(); meshes=[]
for link in robot.findall('link'):
 name=link.get('name')
 if name not in ARM_LINKS+CHASSIS_LINKS:continue
 for collision in link.findall('collision'):
  origin=collision.find('origin');tf=homogeneous(rpy_matrix(_floats(origin.get('rpy') if origin is not None else None)),_floats(origin.get('xyz') if origin is not None else None))
  mesh=collision.find('geometry/mesh')
  if mesh is not None:
   obj=trimesh.load(source.parent/mesh.get('filename'),force='mesh',process=False);bounds=np.array(obj.bounds)
  elif collision.find('geometry/box') is not None:
   half=np.array(_floats(collision.find('geometry/box').get('size')))/2;bounds=np.array([-half,half])
  elif collision.find('geometry/cylinder') is not None:
   cyl=collision.find('geometry/cylinder');half=np.array([float(cyl.get('radius'))]*2+[float(cyl.get('length'))/2]);bounds=np.array([-half,half])
  else:raise RuntimeError('Unknown collision: '+name)
  corners=np.array([[x,y,z,1] for x in bounds[:,0] for y in bounds[:,1] for z in bounds[:,2]])
  if mesh is not None and mesh.get('scale'):corners[:,:3]*=np.array(_floats(mesh.get('scale')))
  meshes.append((name,corners,tf))
stage=Usd.Stage.Open('/home/pa33/git/lv2-nz2-project2/r12/models/navigation/mission_scene.usda')
for prim in stage.Traverse():
 if any('/StartPanel/'+s in str(prim.GetPath()) for s in ['Board','Barcode','LampR','LampY','LampG']):prim.SetActive(False)
cache=UsdGeom.BBoxCache(0.,[UsdGeom.Tokens.default_,UsdGeom.Tokens.render,UsdGeom.Tokens.proxy,UsdGeom.Tokens.guide]);obs=[]
for prim in stage.Traverse():
 if not prim.HasAPI(UsdPhysics.CollisionAPI) or UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:continue
 if prim.IsA(UsdGeom.Mesh) and UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get() in (None,'none'):
  mesh=UsdGeom.Mesh(prim);pts=np.array(mesh.GetPointsAttr().Get());cnt=np.array(mesh.GetFaceVertexCountsAttr().Get());ind=np.array(mesh.GetFaceVertexIndicesAttr().Get());matrix=np.array(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0.));verts=(pts@matrix[:3,:3]+matrix[3,:3])[ind];starts=np.r_[0,np.cumsum(cnt)[:-1]];lo=np.minimum.reduceat(verts,starts);hi=np.maximum.reduceat(verts,starts)
 else:
  box=cache.ComputeWorldBound(prim).ComputeAlignedRange();lo=np.array(box.GetMin())[None];hi=np.array(box.GetMax())[None]
 obs.append((str(prim.GetPath()),lo,hi))
sys.path.insert(0,str(root/'scripts'))
from live_mission_optimized import load_color_rule
r=load_color_rule();tower=r.TOWER;look=r.TOWER_LOOK;tx,ty=cfg['tower_scene']['xy_m'];bz=cfg['tower_scene']['base_z_m'];rad=tower['lens_diameter_m']/2
lo=min(x[0] for x in tower['tiers'].values());hi=max(x[1] for x in tower['tiers'].values())
for name,sz,z in [('Body',[2*rad,2*rad,hi-lo+look['cap_h_m']],(lo+hi+look['cap_h_m'])/2),('Pole',[2*look['pole_radius_m']]*2+[lo-r._BASE_H],(lo+r._BASE_H)/2),('Base0',list(look['base_size_m'])+[r._BASE_H],r._BASE_H/2),('Base1',list(look['base_size_m'][::-1])+[r._BASE_H],r._BASE_H/2)]:
 center=np.array([tx,ty,bz+z]);half=np.array(sz)/2;obs.append(('Tower/'+name,(center-half)[None],(center+half)[None]))
start=np.array(cfg['barcode_base_xyz_yaw']);goal=np.array(json.loads(sys.argv[1]) if len(sys.argv)>1 else cfg['signal_fallback']['base_xyz_yaw']);q=model.arm_q(cfg['barcode_arm_q_rad']);q.update(jaw_left_joint=cfg['jaw_open_m'],jaw_right_joint=cfg['jaw_open_m'])
hits=[];checks=0
for f in np.linspace(0,1,101):
 b=start+(goal-start)*f
 for name,corners,local in meshes:
  tf=model.link_transform(name,q,base_transform(b))@local;points=(tf@corners.T).T[:,:3];low=points.min(0);high=points.max(0)
  for oname,ol,oh in obs:
   checks+=1
   near=np.all(high+.03>=ol,axis=1)&np.all(oh+.03>=low,axis=1)
   if np.any(near):hits.append(dict(f=float(f),link=name,obstacle=oname))
report=dict(samples=101,checks=checks,method='Conservative mesh AABB with static mesh face bounds; 3cm margin; not swept-volume proof',start=start.tolist(),goal=goal.tolist(),hits=hits)
Path('/tmp/codex_mast_relocation_preflight.json').write_text(json.dumps(report,indent=2))
print(json.dumps(dict(checks=checks,hits=len(hits),examples=hits[:10])))
