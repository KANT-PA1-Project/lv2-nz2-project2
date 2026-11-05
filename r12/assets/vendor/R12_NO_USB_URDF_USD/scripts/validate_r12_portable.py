from pathlib import Path
import json, hashlib, xml.etree.ElementTree as ET, math, zipfile, shutil
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from pxr import Usd, UsdGeom, UsdPhysics, UsdUtils, Gf
from build_r12_portable import OUT,ROOT,SRC,C,fk,trans

def main():
 r=ET.parse(OUT/'urdf/r12_no_usb.urdf').getroot();ts=fk(r)
 links=[x.get('name') for x in r.findall('link')];joints=r.findall('joint')
 assert len(links)==len(set(links));assert len(joints)==len(links)-1
 assert len(set(j.get('name') for j in joints))==len(joints)
 children=[j.find('child').get('link') for j in joints];assert len(set(children))==len(children)
 assert set(links)-set(children)=={'base_link'}
 st=Usd.Stage.Open(str(OUT/'usd/r12_no_usb.usda'));assert st and st.GetDefaultPrim()
 assert UsdGeom.GetStageMetersPerUnit(st)==1 and UsdGeom.GetStageUpAxis(st)=='Z'
 paths=json.loads((OUT/'checks/usd_link_paths.json').read_text());cache=UsdGeom.XformCache()
 errors={}
 for name,path in paths.items():
  u=np.array(cache.GetLocalToWorldTransform(st.GetPrimAtPath(path))).T
  errors[name]=float(np.abs(u-ts[name]).max());assert errors[name]<2e-6,(name,errors[name])
 asset_checks=[]
 for el in r.findall('.//mesh'):
  p=(OUT/'urdf'/el.get('filename')).resolve();assert p.is_relative_to(OUT.resolve()) and p.is_file()
  m=trimesh.load_mesh(p,process=False);assert len(m.faces)>0 and np.isfinite(m.vertices).all()
  assert m.extents.max()<3,p
  asset_checks.append(p.name)
 _,assets,unresolved=UsdUtils.ComputeAllDependencies(str(OUT/'usd/r12_no_usb.usda'));assert not unresolved,unresolved
 inertias=[]
 for l in r.findall('link'):
  i=l.find('inertial')
  if i is None:continue
  a=i.find('inertia').attrib;I=np.array([[float(a['ixx']),float(a['ixy']),float(a['ixz'])],[float(a['ixy']),float(a['iyy']),float(a['iyz'])],[float(a['ixz']),float(a['iyz']),float(a['izz'])]])
  ev=np.linalg.eigvalsh(I);assert ev.min()>0 and ev.max()<=sum(ev)-ev.max()+1e-9
  inertias.append(float(i.find('mass').get('value')))
 movable=[]
 for j in joints:
  if j.get('type')=='fixed':continue
  p=j.find('parent').get('link');c=j.find('child').get('link');uj=UsdPhysics.RevoluteJoint(st.GetPrimAtPath('/Robot/Joints/'+j.get('name')))
  assert uj
  p0=np.array(uj.GetLocalPos0Attr().Get());p1=np.array(uj.GetLocalPos1Attr().Get())
  assert np.linalg.norm(ts[p][:3,:3]@p0+ts[p][:3,3]-ts[c][:3,:3]@p1-ts[c][:3,3])<1e-6
  def qmat(q):return Rotation.from_quat([*q.GetImaginary(),q.GetReal()]).as_matrix()
  axis0=ts[p][:3,:3]@qmat(uj.GetLocalRot0Attr().Get())@np.array([1,0,0])
  axis1=ts[c][:3,:3]@qmat(uj.GetLocalRot1Attr().Get())@np.array([1,0,0]);assert np.linalg.norm(axis0-axis1)<1e-6
  if j.get('type')=='revolute':
   lim=j.find('limit');assert abs(uj.GetLowerLimitAttr().Get()-math.degrees(float(lim.get('lower'))))<1e-4
   assert abs(uj.GetUpperLimitAttr().Get()-math.degrees(float(lim.get('upper'))))<1e-4
  movable.append(j.get('name'))
 assert len(movable)==13
 assert not any('harness' in x or 'usb_branch' in x or 'usb_connector' in x for x in links)
 rows=json.loads((OUT/'checks/body_geometry_map.json').read_text(encoding='utf-8'))
 assert len(rows)==4443
 for token in ['INBOARD_LATERAL_REINFORCED_V10','FRAME3_FIXED_SOCKET_V6','FRAME4_MAST_SEAT_V6','OPENCR_REAR_HANGER_REINFORCED_R04']:
  assert any(token in row['path'] for row in rows),token
 placements=json.loads((OUT/'checks/current_sensor_placements.json').read_text())['placements'];sensor_errors=[]
 for p in placements:
  if not p['name'].endswith('_sensor'):continue
  found=[x for x in rows if x['component']==p['name']];assert len(found)==1
  b=np.array(p['bounds_mm']);corners=trimesh.bounds.corners(b);world=corners@C.T/1000
  err=float(np.abs(np.array(found[0]['world_bounds_m'])-np.array([world.min(0),world.max(0)])).max());assert err<.0002,(p['name'],err)
  sensor_errors.append({'sensor':p['name'],'bbox_error_m':err})
 source=ROOT/'design/exports/MAST450_SENSOR_REVIEW/R12/CAD136_R12_V10_OPENCR_R04_NO_USB_CABLES.f3d'
 report={'status':'STRUCTURAL_VALIDATION_PASS_NOT_PHYSICS_QUALIFIED','source_f3d':str(source),'source_sha256':hashlib.file_digest(source.open('rb'),'sha256').hexdigest(),'visible_CAD_bodies':len(rows),'links':len(links),'movable_joints':movable,'visual_meshes':len(set(asset_checks)),'max_urdf_usd_zero_pose_error':max(errors.values()),'lidar_geometry_checks':sensor_errors,'nominal_mass_scenario_kg':sum(inertias),'unresolved_USD_dependencies':list(unresolved),'collision_geometry':'current static occurrence AABBs + inherited articulated proxies; conservative, not assembly-clearance geometry','not_verified':['Isaac Sim runtime / PhysX cooking','ROS topic streaming / calibrated camera intrinsics','real masses and inertias','self-collision and cable collision','physical 360 degree detection or 30 mm obstacle detection','gripper opening and internal gear contacts']}
 (OUT/'checks/VALIDATION.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
 print(json.dumps(report,ensure_ascii=True,indent=2))
if __name__=='__main__':main()

