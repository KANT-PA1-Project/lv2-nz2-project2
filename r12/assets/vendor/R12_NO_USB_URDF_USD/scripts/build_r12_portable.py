"""Portable current-CAD visuals; inherited experimental kinematics, conservative collisions."""
from pathlib import Path
import json, math, collections, hashlib, shutil, zipfile
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation as Rot
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Vt

ROOT=Path('D:/tuetlebot_upgrade')
OUT=ROOT/'design/exports/R12_NO_USB_URDF_USD'
SRC=ROOT/'releases/ubuntu_cuvslam_repo/assets/m38_arm_description'
C=np.array([[0,1,0],[-1,0,0],[0,0,1.]])
COLORS={'metal':(.58,.61,.65),'polymer':(.17,.39,.52),'electronics':(.12,.36,.22),'rubber':(.055,.06,.07)}
def s(a):return ' '.join(f'{float(v):.10g}' for v in a)
def trans(e):
 t=np.eye(4)
 if e is not None:
  t[:3,3]=np.fromstring(e.get('xyz','0 0 0'),sep=' ')
  t[:3,:3]=Rot.from_euler('xyz',np.fromstring(e.get('rpy','0 0 0'),sep=' ')).as_matrix()
 return t
def fk(r):
 ts={'base_link':np.eye(4)};todo=list(r.findall('joint'))
 while todo:
  n=len(todo)
  for j in todo[:]:
   p=j.find('parent').get('link');c=j.find('child').get('link')
   if p in ts:ts[c]=ts[p]@trans(j.find('origin'));todo.remove(j)
  assert len(todo)<n,'Disconnected tree'
 return ts
def quat(R):
 q=Rot.from_matrix(R).as_quat();return Gf.Quatf(float(q[3]),Gf.Vec3f(*q[:3]))
def pose(x,t):
 x.AddTranslateOp().Set(Gf.Vec3d(*t[:3,3]));x.AddOrientOp().Set(quat(t[:3,:3]))
def mesh_usd(st,path,m,color):
 u=UsdGeom.Mesh.Define(st,path)
 u.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(m.vertices,dtype=np.float32)))
 u.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(m.faces),3,dtype=np.int32)))
 u.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.asarray(m.faces,dtype=np.int32).ravel()))
 u.CreateSubdivisionSchemeAttr('none');u.CreateDisplayColorAttr([Gf.Vec3f(*color)])
 return u
def main():
 rows=json.loads((OUT/'body_link_map.json').read_text(encoding='utf-8'))
 assert int((OUT/'cursor.txt').read_text())==len(rows),'CAD export incomplete'
 for folder in ['urdf','usd','meshes','checks','scripts']:(OUT/folder).mkdir(exist_ok=True)
 r=ET.parse(SRC/'urdf/m38_r11.urdf').getroot();r.set('name','r12_no_usb_experimental')
 physical={x['link'] for x in rows}
 keep=physical|{'sim_left_optical','sim_right_optical'}
 keep|={l.get('name') for l in r.findall('link') if l.get('name').startswith(('r07_camera','lidar_'))}
 for l in list(r.findall('link')):
  if l.get('name') not in keep:r.remove(l);continue
  for v in list(l.findall('visual')):l.remove(v)
  if l.get('name') not in physical:
   for tag in ['collision','inertial']:
    for e in list(l.findall(tag)):l.remove(e)
 for j in list(r.findall('joint')):
  if j.find('child').get('link') not in keep:r.remove(j)
 placements=json.loads((ROOT/'design/exports/MAST450_SENSOR_REVIEW/R12/ASSEMBLY_PLACEMENTS.json').read_text())['placements']
 live={x['component']:x for x in json.loads((OUT/'checks/live_sensor_transforms.json').read_text())}
 corrections=[]
 for p in placements:
  if not p['name'].endswith('_sensor'):continue
  old=np.array(p['bounds_mm']);actual=np.array(live[p['name']]['bbox_cm'])*10
  delta=actual-old;assert np.allclose(delta[0],delta[1],atol=.001),'Sensor orientation or shape changed'
  p['optical_origin_cad_mm']=(np.array(p['optical_origin_cad_mm'])+delta.mean(0)).tolist();p['bounds_mm']=actual.tolist()
  corrections.append({'sensor':p['name'],'translation_from_old_record_mm':delta.mean(0).tolist()})
 (OUT/'checks/current_sensor_placements.json').write_text(json.dumps({'placements':placements,'corrections':corrections},indent=2))
 names={'front_right':'lidar_front_right','front_left':'lidar_low_front_left','rear_left':'lidar_rear_left','rear_right':'lidar_low_rear_right'}
 for p in placements:
  if not p['name'].endswith('_sensor'):continue
  n=names[p['name'].removeprefix('R12_').removesuffix('_sensor')]
  j=next(j for j in r.findall('joint') if j.find('child').get('link')==n)
  e=j.find('origin');e.set('xyz',s(C@p['optical_origin_cad_mm']/1000));e.set('rpy',s([0,0,math.radians(p['yaw_cad_deg']-90)]))
 ts=fk(r)
 grouped=collections.defaultdict(list);boxes={};evidence=[]
 for k,x in enumerate(rows):
  m=trimesh.load_mesh(OUT/'raw'/f'{k:05d}.stl',process=False)
  assert len(m.faces)>0 and np.isfinite(m.vertices).all()
  world=(C@m.vertices.T).T/1000;t=ts[x['link']]
  m.vertices=(world-t[:3,3])@t[:3,:3]
  n=(x['component']+' '+x['body']).upper()
  kind='metal' if any(v in n for v in ['ISO','BOLT','NUT','BEARING','SHAFT','PCB_SPACER']) else 'polymer'
  if any(v in n for v in ['STOCK_TB3_WHEEL','TPU','STRAP','RUBBER']):kind='rubber'
  if any(v in n for v in ['OFFICIAL','JETSON','OPENCR_REV','DC12','D435','SENSOR','XM430']):kind='electronics'
  grouped[(x['link'],kind)].append(m)
  if x['link']=='base_link':
   b=m.bounds;key=x['path'] or x['component']
   if key not in boxes:boxes[key]=b.copy()
   else:boxes[key]=np.array([np.minimum(boxes[key][0],b[0]),np.maximum(boxes[key][1],b[1])])
  evidence.append({**x,'cad_mesh':f'raw/{k:05d}.stl','world_bounds_m':[world.min(0).tolist(),world.max(0).tolist()]})
 meshes={};mesh_stats=[]
 for (link,kind),items in grouped.items():
  m=trimesh.util.concatenate(items);filename=f'{link}_{kind}.stl';m.export(OUT/'meshes'/filename);meshes[filename]=m
  l=r.find(f"link[@name='{link}']");v=ET.SubElement(l,'visual',name=kind)
  ET.SubElement(ET.SubElement(v,'geometry'),'mesh',filename='../meshes/'+filename)
  mat=ET.SubElement(v,'material',name=kind);ET.SubElement(mat,'color',rgba=s([*COLORS[kind],1]))
  mesh_stats.append({'file':filename,'bodies':len(items),'faces':len(m.faces),'bounds_m':m.bounds.tolist()})
 base=r.find("link[@name='base_link']")
 for c in list(base.findall('collision')):base.remove(c)
 for k,(path,b) in enumerate(boxes.items()):
  ext=b[1]-b[0]
  if ext.max()<.005:continue
  c=ET.SubElement(base,'collision',name=f'cad_occurrence_box_{k:03d}')
  ET.SubElement(c,'origin',xyz=s(b.mean(0)));ET.SubElement(ET.SubElement(c,'geometry'),'box',size=s(np.maximum(ext,.0005)))
 ET.indent(r);ET.ElementTree(r).write(OUT/'urdf/r12_no_usb.urdf',encoding='utf-8',xml_declaration=True)
 st=Usd.Stage.CreateNew(str(OUT/'usd/r12_no_usb.usdc'));root=UsdGeom.Xform.Define(st,'/Robot');st.SetDefaultPrim(root.GetPrim());UsdGeom.SetStageMetersPerUnit(st,1);UsdGeom.SetStageUpAxis(st,'Z')
 UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim());paths={}
 for l in r.findall('link'):
  n=l.get('name')
  if n not in physical:continue
  path='/Robot/'+n;paths[n]=path;x=UsdGeom.Xform.Define(st,path);pose(x,ts[n]);UsdPhysics.RigidBodyAPI.Apply(x.GetPrim())
  inertia=l.find('inertial');mass=UsdPhysics.MassAPI.Apply(x.GetPrim());mass.CreateMassAttr(float(inertia.find('mass').get('value')))
  mass.CreateCenterOfMassAttr(Gf.Vec3f(*trans(inertia.find('origin'))[:3,3]))
  a=inertia.find('inertia').attrib;I=np.array([[float(a['ixx']),float(a['ixy']),float(a['ixz'])],[float(a['ixy']),float(a['iyy']),float(a['iyz'])],[float(a['ixz']),float(a['iyz']),float(a['izz'])]])
  eig,V=np.linalg.eigh(I);assert eig.min()>0
  if np.linalg.det(V)<0:V[:,0]*=-1
  mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*eig));mass.CreatePrincipalAxesAttr(quat(V))
  for v in l.findall('visual'):
   f=Path(v.find('geometry/mesh').get('filename')).name;kind=v.get('name');mesh_usd(st,path+'/visual_'+kind,meshes[f],COLORS[kind])
  for i,c in enumerate(l.findall('collision')):
   cp=path+f'/collision_{i:03d}';g=c.find('geometry');b=g.find('box');cy=g.find('cylinder')
   if b is not None:
    u=UsdGeom.Cube.Define(st,cp);u.CreateSizeAttr(1);pose(u,trans(c.find('origin')));u.AddScaleOp().Set(Gf.Vec3f(*np.fromstring(b.get('size'),sep=' ')))
   elif cy is not None:
    u=UsdGeom.Cylinder.Define(st,cp);u.CreateRadiusAttr(float(cy.get('radius')));u.CreateHeightAttr(float(cy.get('length')));u.CreateAxisAttr('Z');pose(u,trans(c.find('origin')))
   else:raise ValueError('Unexpected dynamic collision mesh')
   UsdGeom.Imageable(u).CreatePurposeAttr('guide');UsdPhysics.CollisionAPI.Apply(u.GetPrim())
 # Fixed optical frames inherit their nearest physical parent rather than becoming massless rigid bodies.
 pending=[j for j in r.findall('joint') if j.find('child').get('link') not in physical]
 while pending:
  n=len(pending)
  for j in pending[:]:
   p=j.find('parent').get('link');c=j.find('child').get('link')
   if p not in paths:continue
   paths[c]=paths[p]+'/'+c;x=UsdGeom.Xform.Define(st,paths[c]);pose(x,trans(j.find('origin')));pending.remove(j)
  assert len(pending)<n
 for j in r.findall('joint'):
  child=j.find('child').get('link')
  if child not in physical:continue
  parent=j.find('parent').get('link');t=trans(j.find('origin'));a=np.fromstring(j.find('axis').get('xyz'),sep=' ')
  Q=trimesh.geometry.align_vectors([1,0,0],a)[:3,:3]
  u=UsdPhysics.RevoluteJoint.Define(st,'/Robot/Joints/'+j.get('name'));u.CreateBody0Rel().SetTargets([paths[parent]]);u.CreateBody1Rel().SetTargets([paths[child]])
  u.CreateAxisAttr('X');u.CreateLocalPos0Attr(Gf.Vec3f(*t[:3,3]));u.CreateLocalRot0Attr(quat(t[:3,:3]@Q));u.CreateLocalRot1Attr(quat(Q))
  lim=j.find('limit')
  if j.get('type')=='revolute':u.CreateLowerLimitAttr(math.degrees(float(lim.get('lower'))));u.CreateUpperLimitAttr(math.degrees(float(lim.get('upper'))))
  drive=UsdPhysics.DriveAPI.Apply(u.GetPrim(),'angular');drive.CreateTypeAttr('force');drive.CreateMaxForceAttr(float(lim.get('effort')));drive.CreateStiffnessAttr(0 if j.get('type')=='continuous' else 40);drive.CreateDampingAttr(4);drive.CreateTargetPositionAttr(0);drive.CreateTargetVelocityAttr(0)
 # Preserve the previous simulation's deliberate self-collision exclusion; this is NOT a clearance test.
 for n in physical:UsdPhysics.FilteredPairsAPI.Apply(st.GetPrimAtPath(paths[n])).CreateFilteredPairsRel().SetTargets([paths[k] for k in physical if k!=n])
 for n,path in paths.items():
  if not n.endswith('_optical'):continue
  u=UsdGeom.Camera.Define(st,path+'/Camera');u.AddOrientOp().Set(quat(np.diag([1,-1,-1])))
  u.CreateFocalLengthAttr(20);u.CreateHorizontalApertureAttr(40*math.tan(math.radians(43.5)));u.CreateVerticalApertureAttr(40*math.tan(math.radians(29)));u.CreateClippingRangeAttr(Gf.Vec2f(.2,12))
 st.GetRootLayer().Save()
 entry=Usd.Stage.CreateNew(str(OUT/'usd/r12_no_usb.usda'));p=entry.DefinePrim('/Robot','Xform');p.GetReferences().AddReference('./r12_no_usb.usdc');entry.SetDefaultPrim(p);UsdGeom.SetStageMetersPerUnit(entry,1);UsdGeom.SetStageUpAxis(entry,'Z');entry.GetRootLayer().Save()
 (OUT/'checks/body_geometry_map.json').write_text(json.dumps(evidence,ensure_ascii=False,indent=2),encoding='utf-8')
 (OUT/'checks/mesh_statistics.json').write_text(json.dumps(mesh_stats,indent=2))
 (OUT/'checks/usd_link_paths.json').write_text(json.dumps(paths,indent=2))
 shutil.copy2(__file__,OUT/'scripts/build_r12_portable.py')
 print(json.dumps({'bodies':len(rows),'visual_meshes':len(meshes),'physical_links':len(physical),'links':len(ts),'faces':sum(len(m.faces) for m in meshes.values())}))
if __name__=='__main__':main()
