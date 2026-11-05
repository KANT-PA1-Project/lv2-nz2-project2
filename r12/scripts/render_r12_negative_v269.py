"""Static isolated surface diagnostic. No robot, physics stepping, or video."""
import sys,json,math,hashlib
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[1];out=root.parents[1]/'outputs/r12/negative_clutter_v269';out.mkdir(exist_ok=False)
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','anti_aliasing':0})
import omni.usd,omni.replicator.core as rep
from pxr import UsdGeom,UsdShade,UsdLux,Sdf,Gf
import cv2
stage=omni.usd.get_context().get_stage();UsdLux.DomeLight.Define(stage,'/World/Light').CreateIntensityAttr(1500.)
cam=UsdGeom.Camera.Define(stage,'/World/Camera');op=cam.AddTranslateOp();cam.CreateHorizontalApertureAttr(20.955);cam.CreateVerticalApertureAttr(15.71625);cam.CreateFocalLengthAttr(13.096875);cam.CreateClippingRangeAttr(Gf.Vec2f(.01,30))
rp=rep.create.render_product(str(cam.GetPath()),(640,480));ann=rep.AnnotatorRegistry.get_annotator('rgb');ann.attach(rp);depth_ann=rep.AnnotatorRegistry.get_annotator('distance_to_image_plane');depth_ann.attach(rp)

light=UsdLux.DomeLight.Get(stage,'/World/Light')
op.Set(Gf.Vec3d(0,0,.75))
rng=np.random.default_rng(2692026);rows=[]
def material(name,color,roughness):
 m=UsdShade.Material.Define(stage,'/World/Materials/'+name);sh=UsdShade.Shader.Define(stage,str(m.GetPath())+'/Shader');sh.CreateIdAttr('UsdPreviewSurface');sh.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color));sh.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(roughness);m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),'surface');return m
try:
 for layout in range(4):
  # Fixed geometry per layout across lighting and roughness comparisons.
  specs=[(float(rng.uniform(-.43,.43)),float(rng.uniform(-.30,.30)),float(rng.uniform(-.08,.12)),float(rng.uniform(.015,.07)),float(rng.uniform(.03,.13))) for _ in range(32)]
  for rough in [.15,.8]:
   stage.RemovePrim('/World/Objects');stage.RemovePrim('/World/Materials')
   palette=[(.8,.02,.02),(.02,.65,.02),(.03,.03,.03),(.8,.8,.8),(.35,.3,.22)]
   mats=[material('M'+str(i),c,rough) for i,c in enumerate(palette)]
   back=UsdGeom.Cube.Define(stage,'/World/Objects/Backdrop');back.CreateSizeAttr(1);back.AddTranslateOp().Set(Gf.Vec3d(0,0,-.16));back.AddScaleOp().Set(Gf.Vec3f(1.5,1.1,.02));UsdShade.MaterialBindingAPI.Apply(back.GetPrim()).Bind(mats[3])
   for i,(x,y,z,w,h) in enumerate(specs):
    path='/World/Objects/O'+str(i)
    if i%3==0:
     body=UsdGeom.Cylinder.Define(stage,path);body.CreateRadiusAttr(w/2);body.CreateHeightAttr(h);body.CreateAxisAttr('Y')
    else:
     body=UsdGeom.Cube.Define(stage,path);body.CreateSizeAttr(1)
    body.AddTranslateOp().Set(Gf.Vec3d(x,y,z));body.AddRotateYOp().Set(float((i*17+layout*23)%90))
    if i%3:body.AddScaleOp().Set(Gf.Vec3f(w,h,.025))
    UsdShade.MaterialBindingAPI.Apply(body.GetPrim()).Bind(mats[i%len(mats)])
   for intensity in [300.,1500.,4500.]:
    light.GetIntensityAttr().Set(intensity)
    for _ in range(12):app.update()
    rep.orchestrator.step(rt_subframes=4);rgb=np.asarray(ann.get_data())[:,:,:3].copy()
    index=len(rows);cv2.imwrite(str(out/f'frame_{index:02d}.png'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
    rows.append(dict(index=index,layout=layout,roughness=rough,light_intensity=intensity,expected_tags=[],rgb_sha256=hashlib.sha256(rgb.tobytes()).hexdigest()))
    (out/'manifest.json').write_text(json.dumps(dict(seed=2692026,scope='Abstract clutter negative render,opaque cubes/cylinders;no texture tags;not actual mission scene or robot cameras',rows=rows),indent=2)+'\n');print(rows[-1],flush=True)
finally:app.close()
