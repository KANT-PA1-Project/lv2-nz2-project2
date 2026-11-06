"""Setup evaluator only. No sensor/control inputs or guaranteed visibility."""
from pathlib import Path
import sys,json,itertools,hashlib
import numpy as np
from pxr import Usd,UsdGeom,Gf
root=Path(__file__).resolve().parent;sys.path.insert(0,str(root/'src'))
from arm_practice.native_rgbd.puck_fixture import build
s=Usd.Stage.CreateInMemory();s.DefinePrim('/World/Mission').GetReferences().AddReference(str(root/'models/navigation/mission_scene.usda'))
p=UsdGeom.Xform.Define(s,'/World/Spawn');p.AddTranslateOp().Set(Gf.Vec3d(.734,.950,.008));p.AddRotateZOp().Set(180.)
# Same installed URDF FK path used at scene startup.
sys.path.insert(0,str(root/'scripts/sensor_runtime'))
from camera_rig import transforms
Tbc=transforms(root,{f'arm_joint{i}':0. for i in range(1,6)})['arm_camera_left_optical']
fixture=build(s,Tbc);Twb=np.array(UsdGeom.Xformable(p).ComputeLocalToWorldTransform(0)).T;Tcw=np.linalg.inv(Twb@Tbc)
cache=UsdGeom.BBoxCache(0,[UsdGeom.Tokens.default_,UsdGeom.Tokens.render]);objects=[]
for name in ['Puck0']:
 prim=s.GetPrimAtPath('/World/Mission/Pucks/'+name)
 assert prim.IsDefined() and prim.IsActive()
 world=np.array(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)).T
 bounds=cache.ComputeWorldBound(prim).ComputeAlignedRange();lo=np.array(bounds.GetMin());hi=np.array(bounds.GetMax());corners=np.array(list(itertools.product(*zip(lo,hi))))
 camera=(np.c_[corners,np.ones(8)]@Tcw.T)[:,:3];assert np.all(camera[:,2]>.12)
 pixels=camera[:,:2]/camera[:,2,None]*1200+[959.5,719.5];rect=np.r_[pixels.min(axis=0),pixels.max(axis=0)]
 assert 40<rect[0]<rect[2]<1880 and 40<rect[1]<rect[3]<1400
 objects.append(dict(path=str(prim.GetPath()),projected_world_AABB_bbox=rect.tolist(),camera_depth_range_m=[float(camera[:,2].min()),float(camera[:,2].max())],composed_children=[dict(path=str(q.GetPath()),type=q.GetTypeName())for q in Usd.PrimRange(prim)],world_matrix=world.tolist()))
gap=None
print(json.dumps(dict(scope='setup/evaluator only; frustum bounds do not prove rendered visibility',fixture=fixture,objects=objects,projected_bbox_gap_px=gap,fixture_source_sha256=hashlib.sha256((root/'src/arm_practice/native_rgbd/puck_fixture.py').read_bytes()).hexdigest(),motion_authorized=False),indent=2))
