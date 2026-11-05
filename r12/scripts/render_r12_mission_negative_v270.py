"""Static isolated surface diagnostic. No robot, physics stepping, or video."""
import sys,json,math,hashlib
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[1];out=root.parents[1]/'outputs/r12/mission_negative_v270';out.mkdir(exist_ok=False)
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','anti_aliasing':0})
import omni.usd,omni.replicator.core as rep
from pxr import UsdGeom,UsdShade,UsdLux,Sdf,Gf
import cv2
stage=omni.usd.get_context().get_stage();UsdLux.DomeLight.Define(stage,'/World/Light').CreateIntensityAttr(1500.)
cam=UsdGeom.Camera.Define(stage,'/World/Camera');op=cam.AddTranslateOp();cam.CreateHorizontalApertureAttr(20.955);cam.CreateVerticalApertureAttr(15.71625);cam.CreateFocalLengthAttr(13.096875);cam.CreateClippingRangeAttr(Gf.Vec2f(.01,30))
rp=rep.create.render_product(str(cam.GetPath()),(640,480));ann=rep.AnnotatorRegistry.get_annotator('rgb');ann.attach(rp);depth_ann=rep.AnnotatorRegistry.get_annotator('distance_to_image_plane');depth_ann.attach(rp)


sys.path[:0]=[str(root/'src'),str(root/'scripts/sensor_runtime')]
from camera_rig import transforms
source=root/'models/navigation/mission_scene.usda';before=hashlib.sha256(source.read_bytes()).hexdigest()
mission=stage.DefinePrim('/World/Mission','Xform');mission.GetReferences().AddReference(str(source))
hidden=[]
for prim in stage.Traverse():
 if str(prim.GetPath()).startswith('/World/Mission/Pucks/') and prim.GetName()=='Surface':
  UsdGeom.Imageable(prim).MakeInvisible();hidden.append(str(prim.GetPath()))
assert hidden,'No tag surfaces hidden'
ground=UsdGeom.Cube.Define(stage,'/World/Ground');ground.CreateSizeAttr(1);ground.AddTranslateOp().Set(Gf.Vec3d(2.2,3.2,-.025));ground.AddScaleOp().Set(Gf.Vec3f(12,12,.05))
rig=transforms(root,{f'arm_joint{i}':0 for i in range(1,6)})
cam.ClearXformOpOrder();camera_op=cam.AddTransformOp();rows=[]
poses=[(.734,.950,180.),(3.027844833581866,4.897731112552753,0.),(3.027844833581866,4.897731112552753,90.)]
try:
 for pose_index,(x,y,yaw) in enumerate(poses):
  angle=math.radians(yaw);base=np.array([[math.cos(angle),-math.sin(angle),0,x],[math.sin(angle),math.cos(angle),0,y],[0,0,1,.008],[0,0,0,1.]])
  for name in ['r07_camera1','r07_camera2','r07_camera3','arm_camera']:
   T=base@rig[name+'_left_optical']@np.diag([1.,-1.,-1.,1.]);camera_op.Set(Gf.Matrix4d(*T.T.flatten().tolist()))
   for intensity in [600.,1800.]:
    UsdLux.DomeLight.Get(stage,'/World/Light').GetIntensityAttr().Set(intensity)
    for _ in range(12):app.update()
    rep.orchestrator.step(rt_subframes=4);rgb=np.asarray(ann.get_data())[:,:,:3].copy();index=len(rows)
    cv2.imwrite(str(out/f'frame_{index:02d}.png'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR));rows.append(dict(index=index,pose_index=pose_index,base_pose=[x,y,yaw],camera=name,light_intensity=intensity,expected_tags=[]))
    (out/'manifest.json').write_text(json.dumps(dict(source=str(source),source_sha256=before,hidden_tag_surfaces=hidden,scope='Mission asset static negative renders;configured3upper+1wrist transforms at zero arm joints;robot body omitted;no physics/control/video;camera placements diagnostic not navigation feasibility',rows=rows),indent=2)+'\n');print(rows[-1],flush=True)
 assert hashlib.sha256(source.read_bytes()).hexdigest()==before
finally:app.close()
