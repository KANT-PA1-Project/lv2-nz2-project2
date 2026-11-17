"""Static isolated surface diagnostic. No robot, physics stepping, or video."""
import sys,json,math,hashlib
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[1];out=root.parents[1]/'outputs/r12/tag_surface_v255';out.mkdir(exist_ok=False)
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','anti_aliasing':0})
import omni.usd,omni.replicator.core as rep
from pxr import UsdGeom,UsdShade,UsdLux,Sdf,Gf
import cv2
stage=omni.usd.get_context().get_stage();UsdLux.DomeLight.Define(stage,'/World/Light').CreateIntensityAttr(1500.)
cam=UsdGeom.Camera.Define(stage,'/World/Camera');op=cam.AddTranslateOp();cam.CreateHorizontalApertureAttr(20.955);cam.CreateVerticalApertureAttr(15.71625);cam.CreateFocalLengthAttr(13.096875);cam.CreateClippingRangeAttr(Gf.Vec2f(.01,30))
rp=rep.create.render_product(str(cam.GetPath()),(640,480));ann=rep.AnnotatorRegistry.get_annotator('rgb');ann.attach(rp)
mat=UsdShade.Material.Define(stage,'/World/Mat');sh=UsdShade.Shader.Define(stage,'/World/Mat/Shader');sh.CreateIdAttr('UsdPreviewSurface');sh.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.55);mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),'surface')
uv=UsdShade.Shader.Define(stage,'/World/Mat/UV');uv.CreateIdAttr('UsdPrimvarReader_float2');uv.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st')
tex=UsdShade.Shader.Define(stage,'/World/Mat/Texture');tex.CreateIdAttr('UsdUVTexture');tex.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(str(root/'models/navigation/textures/tag36h11-0.png'));tex.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(uv.ConnectableAPI(),'result');sh.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(),'rgb')
mesh=UsdGeom.Mesh.Define(stage,'/World/Target');mesh.CreatePointsAttr([(-.01,-.01,0),(.01,-.01,0),(.01,.01,0),(-.01,.01,0)]);mesh.CreateFaceVertexCountsAttr([4]);mesh.CreateFaceVertexIndicesAttr([0,1,2,3]);mesh.CreateDoubleSidedAttr(True);rotation=mesh.AddRotateYOp();st=UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,UsdGeom.Tokens.faceVarying);UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11));rows=[]
try:
 for tag in [0]:
  tex.GetInput('file').Set(str(root/f'models/navigation/textures/tag36h11-{tag}.png'))
  for curved in [True]:
   points=[];coords=[];strips=24 if curved else 1
   # Exact local quad construction from mission_scene_objects.quad, including
   # separate vertices per strip. No fixture poses or IDs enter control.
   for j in range(strips):
    for u,v in [(j/strips,0),((j+1)/strips,0),((j+1)/strips,1),(j/strips,1)]:
     x,y=(2*u-1)*.01,(2*v-1)*.01
     points.append((.01503*math.sin(x/.01503) if curved else x,y,.01503*(math.cos(x/.01503)-1) if curved else 0));coords.append((u,v))
   mesh.CreatePointsAttr(points);mesh.CreateFaceVertexCountsAttr([4]*strips);mesh.CreateFaceVertexIndicesAttr(list(range(4*strips)));st.Set(coords)
   for subdiv in ['catmullClark']:
    mesh.CreateSubdivisionSchemeAttr(subdiv)
    for distance in [.2]:
     op.Set(Gf.Vec3d(0,0,distance))
     for angle in [0.]:
      rotation.Set(angle)
      previous=None
      for repeat in range(20):
       for _ in range(12):app.update()
       rep.orchestrator.step(rt_subframes=4);rgb=np.asarray(ann.get_data())[:,:,:3].copy();gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY);_,ids,_=det.detectMarkers(gray)
       roi=gray[200:280,280:360];fixed_decodes=[]
       for _ in range(5):
        _,again,_=det.detectMarkers(gray);fixed_decodes.append([] if again is None else again.flatten().tolist())
       pixel_change=None if previous is None else float(np.mean(np.abs(roi.astype(float)-previous.astype(float))))
       previous=roi.copy()
       row=dict(roi_sha256=hashlib.sha256(roi.tobytes()).hexdigest(),roi_mean_abs_change=pixel_change,fixed_pixel_decodes=fixed_decodes,tag=tag,curved=curved,subdivision=subdiv,distance_m=distance,angle_deg=angle,repeat=repeat,decoded=[] if ids is None else ids.flatten().tolist());rows.append(row);cv2.imwrite(str(out/f'frame_{len(rows):02d}.png'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR));print(row,flush=True)
       (out/'results.json').write_text(json.dumps(dict(rows=rows,scope='Fixed tag0 curved surface20cm angle0;20 consecutive frames;5 identical-pixel decodes each; no robot/body'),indent=2)+'\n')
finally:app.close()
