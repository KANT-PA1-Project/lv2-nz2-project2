#!/usr/bin/env python3
from pathlib import Path
import json,numpy as np,cv2
from pxr import Usd,UsdGeom,UsdPhysics,Gf
from scipy.ndimage import distance_transform_edt
root=Path(__file__).resolve().parents[1];dst=root/'models/navigation';dst.mkdir(exist_ok=True)
s=Usd.Stage.Open(str(root/'assets/vendor/maze/maze.usda'))
b=UsdGeom.BBoxCache(0,['default','render','guide']);corner=s.GetPrimAtPath('/maze/wall_edge/collision/geo_3');box=b.ComputeWorldBound(corner).ComputeAlignedBox();c=(np.array(box.GetMin())+np.array(box.GetMax()))/2;offset=-c;offset[2]=0
res=.02;grid=np.zeros((330,235),np.uint8)
for p in s.Traverse():
 if not (p.IsA(UsdGeom.Mesh) and p.HasAPI(UsdPhysics.CollisionAPI)):continue
 mesh=UsdGeom.Mesh(p);points=np.array(mesh.GetPointsAttr().Get());m=np.array(UsdGeom.Xformable(p).ComputeLocalToWorldTransform(0));points=(np.c_[points,np.ones(len(points))]@m)[:,:3]+offset
 counts=mesh.GetFaceVertexCountsAttr().Get();indices=mesh.GetFaceVertexIndicesAttr().Get();i=0
 for n in counts:
  face=points[np.array(indices[i:i+n])];i+=n
  if face[:,2].max()<.015:continue
  # Faces are convex collision polygons; rasterize their XY projection.
  cv2.fillConvexPoly(grid,np.rint(face[:,:2]/res).astype(np.int32),1)
starts=np.array([[1.468,.950],[3.670,3.152],[2.936,5.354],[.734,5.354],[2.202,5.354],[3.670,3.886]])
goals=np.array([[3.670,.950],[2.202,3.700],[3.670,5.354]])
boxes=[{'name':f'S{i+1}','center':[float(x),float(y),.035],'size':[.205,.205,.07]} for i,(x,y) in enumerate(starts)]
boxes +=[{'name':n,'center':[float(x),float(y),.035],'size':[.205,.205,.07]} for n,(x,y) in zip(['L1','L3'],goals[[0,2]])]
# User override: one small box, centered at the mission's L2 placement grid.
boxes.append({'name':'L2','center':[2.202,3.700,.035],'size':[.205,.205,.07]})
for box in boxes:
 center=np.array(box['center'][:2]);size=np.array(box['size'][:2]);lo=np.floor((center-size/2)/res).astype(int);hi=np.ceil((center+size/2)/res).astype(int);cv2.rectangle(grid,tuple(lo),tuple(hi),1,-1)
grid[0,:]=1;grid[-1,:]=1;grid[:,0]=1;grid[:,-1]=1
clear=distance_transform_edt(1-grid)*res
# Chassis conservative disk. Home arm is above 390 mm walls; carried load not modeled.
radius=.30
np.savez(dst/'maze_grid.npz',occupied=grid,clearance=clear,resolution=res,radius=radius)
config={'map_offset_m':offset.tolist(),'origin_method':'Translation only: lower-left collision post geo_3 centre mapped to (0,0). Source geometry unchanged. PDF/scan dimensions differ; provisional registration.','start':[.734,.950],'starts':starts.tolist(),'destinations':goals.tolist(),'boxes':boxes,'footprint_radius_m':radius,'speed_limit_m_s':.20,'navigation_only':True,'box_source':'/home/seongmin/ros2_ws/src/turtlebot-swerve-cuvslam/perception/scripts/generate_dataset.py','box_source_status':'YOLO brown cardboard proxy 205x205x70 mm; physical Bambu dimensions unverified','start_region_bounds_assumed_m':[.434,.650,1.034,1.250],'start_region_note':'600 mm square inferred from drawing, exact bounds not specified in PDF'}
(dst/'mission_navigation.json').write_text(json.dumps(config,indent=2))
# Inspectable scene using the actual provided maze, with missing mission tables added.
(dst/'mission_scene.usda').unlink(missing_ok=True)
stage=Usd.Stage.CreateNew(str(dst/'mission_scene.usda'))
UsdGeom.SetStageMetersPerUnit(stage,1);UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
world=UsdGeom.Xform.Define(stage,'/World');stage.SetDefaultPrim(world.GetPrim())
maze=UsdGeom.Xform.Define(stage,'/World/Maze');UsdGeom.Xform.Define(stage,'/World/Maze/Geometry').GetPrim().GetReferences().AddReference('../../assets/vendor/maze/maze.usda');maze.AddTranslateOp().Set(Gf.Vec3d(*offset))
for box in boxes:
 cube=UsdGeom.Cube.Define(stage,'/World/Tables/'+box['name']);cube.CreateSizeAttr(1);cube.AddTranslateOp().Set(Gf.Vec3d(*box['center']));cube.AddScaleOp().Set(Gf.Vec3f(*box['size']));UsdPhysics.CollisionAPI.Apply(cube.GetPrim());cube.CreateDisplayColorAttr([Gf.Vec3f(.48,.30,.15)])
from mission_scene_objects import add_mission_objects
add_mission_objects(stage, root, boxes)
stage.GetRootLayer().Save()
print(json.dumps(config,indent=2))
