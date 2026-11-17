"""Static isolated surface diagnostic. No robot, physics stepping, or video."""
import sys,json,math,hashlib
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[1];out=root.parents[1]/'outputs/r12/mission_holdout_negative_v288';out.mkdir(exist_ok=False)
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','anti_aliasing':0})
import omni.usd,omni.replicator.core as rep
from pxr import UsdGeom,UsdShade,UsdLux,Sdf,Gf
import cv2
stage=omni.usd.get_context().get_stage();UsdLux.DomeLight.Define(stage,'/World/Light').CreateIntensityAttr(1500.)
cam=UsdGeom.Camera.Define(stage,'/World/Camera');op=cam.AddTranslateOp();cam.CreateHorizontalApertureAttr(20.955);cam.CreateVerticalApertureAttr(15.71625);cam.CreateFocalLengthAttr(13.096875);cam.CreateClippingRangeAttr(Gf.Vec2f(.01,30))
rp=rep.create.render_product(str(cam.GetPath()),(1920,1440));ann=rep.AnnotatorRegistry.get_annotator('rgb');ann.attach(rp);depth_ann=rep.AnnotatorRegistry.get_annotator('distance_to_image_plane');depth_ann.attach(rp)


sys.path[:0]=[str(root/'src'),str(root/'scripts/sensor_runtime')]
from camera_rig import transforms
source=root/'models/navigation/mission_scene.usda';before=hashlib.sha256(source.read_bytes()).hexdigest()
mission=stage.DefinePrim('/World/Mission','Xform');mission.GetReferences().AddReference(str(source))
hidden=[]
for prim in stage.Traverse():
 if str(prim.GetPath()).startswith('/World/Mission/Pucks/') and prim.GetName()=='Surface':
  hidden.append(str(prim.GetPath()));UsdGeom.Imageable(prim).MakeInvisible()
assert hidden,'No tag surfaces found'
ground=UsdGeom.Cube.Define(stage,'/World/Ground');ground.CreateSizeAttr(1);ground.AddTranslateOp().Set(Gf.Vec3d(2.2,3.2,-.025));ground.AddScaleOp().Set(Gf.Vec3f(12,12,.05))
rig=transforms(root,{f'arm_joint{i}':0 for i in range(1,6)})
cam.ClearXformOpOrder();camera_op=cam.AddTransformOp();rows=[]
# Fixture location is diagnostic placement/scoring only, never sent to worker/control.
target=next(p for p in json.loads((root/'models/navigation/scene_fixture.json').read_text())['pucks'] if p['id']==4)
origin=np.array([3.027844833581866,4.897731112552753]);delta=np.array(target['center_m'][:2])-origin;direction=delta/np.linalg.norm(delta)
poses=[(*(origin+step*direction).tolist(),0.) for step in [.25,.35]]
try:
 for pose_index,(x,y,yaw) in enumerate(poses):
  angle=math.radians(yaw);base=np.array([[math.cos(angle),-math.sin(angle),0,x],[math.sin(angle),math.cos(angle),0,y],[0,0,1,.008],[0,0,0,1.]])
  for name in ['r07_camera2']:
   T=base@rig[name+'_left_optical']@np.diag([1.,-1.,-1.,1.]);camera_op.Set(Gf.Matrix4d(*T.T.flatten().tolist()))
   projections=[]
   # Diagnostic projection only, not an occlusion test or perception input.
   optical=np.linalg.inv(base@rig[name+'_left_optical'])
   for path in hidden:
    mesh=UsdGeom.Mesh.Get(stage,path);world=UsdGeom.Xformable(mesh).ComputeLocalToWorldTransform(0)
    points=np.array([list(world.Transform(Gf.Vec3d(*p))) for p in mesh.GetPointsAttr().Get()]);pc=(optical@np.column_stack([points,np.ones(len(points))]).T).T[:,:3]
    if not np.all(pc[:,2]>.01):continue
    uv=pc[:,:2]/pc[:,2,None]*1200+np.array([960,720]);lo=uv.min(0);hi=uv.max(0)
    if hi[0]>=0 and lo[0]<1920 and hi[1]>=0 and lo[1]<1440:projections.append(dict(path=path,bbox=[*lo.tolist(),*hi.tolist()],width_px=float(hi[0]-lo[0]),height_px=float(hi[1]-lo[1]),occlusion_checked=False,center_distance_m=float(np.linalg.norm(pc.mean(0)))))
   for intensity in [900.,1200.,2400.]:
    UsdLux.DomeLight.Get(stage,'/World/Light').GetIntensityAttr().Set(intensity)
    for repeat in range(3):
     for _ in range(12):app.update()
     rep.orchestrator.step(rt_subframes=4);rgb=np.asarray(ann.get_data())[:,:,:3].copy();index=len(rows)
     cv2.imwrite(str(out/f'frame_{index:02d}.png'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR));rows.append(dict(index=index,pose_index=pose_index,base_pose=[x,y,yaw],translation_from_reference_m=[.25,.35][pose_index],camera=name,light_intensity=intensity,expected_tags=None,repeat=repeat,diagnostic_projections=projections))
     (out/'manifest.json').write_text(json.dumps(dict(source=str(source),source_sha256=before,resolution=[1920,1440],intrinsics=[1200,1200,960,720],hidden_tag_surfaces=hidden,scope='Mission asset static tag-hidden renders;configured3upper+1wrist transforms at zero arm joints;robot body omitted;no physics/control/video;camera placements diagnostic not navigation feasibility',rows=rows),indent=2)+'\n');print(rows[-1],flush=True)
 assert hashlib.sha256(source.read_bytes()).hexdigest()==before
finally:app.close()
