"""Static isolated surface diagnostic. No robot, physics stepping, or video."""
import sys,json,math,hashlib
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[1];out=root.parents[1]/'outputs/r12/joint_observation_v264';out.mkdir(exist_ok=False)
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','anti_aliasing':0})
import omni.usd,omni.replicator.core as rep
from pxr import UsdGeom,UsdShade,UsdLux,Sdf,Gf
import cv2
stage=omni.usd.get_context().get_stage();UsdLux.DomeLight.Define(stage,'/World/Light').CreateIntensityAttr(1500.)
cam=UsdGeom.Camera.Define(stage,'/World/Camera');op=cam.AddTranslateOp();cam.CreateHorizontalApertureAttr(20.955);cam.CreateVerticalApertureAttr(15.71625);cam.CreateFocalLengthAttr(13.096875);cam.CreateClippingRangeAttr(Gf.Vec2f(.01,30))
rp=rep.create.render_product(str(cam.GetPath()),(640,480));ann=rep.AnnotatorRegistry.get_annotator('rgb');ann.attach(rp);depth_ann=rep.AnnotatorRegistry.get_annotator('distance_to_image_plane');depth_ann.attach(rp)
mat=UsdShade.Material.Define(stage,'/World/Mat');sh=UsdShade.Shader.Define(stage,'/World/Mat/Shader');sh.CreateIdAttr('UsdPreviewSurface');sh.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.55);mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),'surface')
uv=UsdShade.Shader.Define(stage,'/World/Mat/UV');uv.CreateIdAttr('UsdPrimvarReader_float2');uv.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st')
tex=UsdShade.Shader.Define(stage,'/World/Mat/Texture');tex.CreateIdAttr('UsdUVTexture');tex.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(str(root/'models/navigation/textures/tag36h11-0.png'));tex.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(uv.ConnectableAPI(),'result');sh.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(),'rgb')
group=UsdGeom.Xform.Define(stage,'/World/Assembly');group_rotation=group.AddRotateYOp()
mesh=UsdGeom.Mesh.Define(stage,'/World/Assembly/Target');mesh.CreatePointsAttr([(-.01,-.01,0),(.01,-.01,0),(.01,.01,0),(-.01,.01,0)]);mesh.CreateFaceVertexCountsAttr([4]);mesh.CreateFaceVertexIndicesAttr([0,1,2,3]);mesh.CreateDoubleSidedAttr(True);rotation=mesh.AddRotateYOp();st=UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,UsdGeom.Tokens.faceVarying);UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11));rows=[]
try:
 for tag in [0,1]:
  tex.GetInput('file').Set(str(root/f'models/navigation/textures/tag36h11-{tag}.png'))
  for curved in [False,True]:
   stage.RemovePrim('/World/Assembly/Body')
   if curved:
    body=UsdGeom.Cylinder.Define(stage,'/World/Assembly/Body');body.CreateRadiusAttr(.015);body.CreateHeightAttr(.06);body.CreateAxisAttr('Y')
   else:
    body=UsdGeom.Cube.Define(stage,'/World/Assembly/Body');body.CreateSizeAttr(1);body.AddScaleOp().Set(Gf.Vec3f(.03,.06,.03))
   body.AddTranslateOp().Set(Gf.Vec3d(0,0,-.01503)) if curved else None
   if not curved:
    # Translation must precede scale to avoid scaling the surface offset.
    body.ClearXformOpOrder();body.AddTranslateOp().Set(Gf.Vec3d(0,0,-.01503));body.AddScaleOp().Set(Gf.Vec3f(.03,.06,.03))
   bm=UsdShade.Material.Define(stage,'/World/BodyMat');bs=UsdShade.Shader.Define(stage,'/World/BodyMat/Shader');bs.CreateIdAttr('UsdPreviewSurface');bs.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set((.8,.02,.02) if tag==0 else (.02,.65,.02));bs.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.55);bm.CreateSurfaceOutput().ConnectToSource(bs.ConnectableAPI(),'surface');UsdShade.MaterialBindingAPI.Apply(body.GetPrim()).Bind(bm)
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
     for angle in [-30.,0.,30.]:
      group_rotation.Set(angle);rotation.Set(0.)
      previous=None
      for repeat in range(2):
       for _ in range(12):app.update()
       rep.orchestrator.step(rt_subframes=4);rgb=np.asarray(ann.get_data())[:,:,:3].copy();gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY);_,ids,_=det.detectMarkers(gray)
       np.savez_compressed(out/f'frame_{len(rows)+1:02d}.npz',rgb=rgb[None],depth=np.asarray(depth_ann.get_data()).copy())
       roi=gray[200:280,280:360];fixed_decodes=[]
       for _ in range(5):
        _,again,_=det.detectMarkers(gray);fixed_decodes.append([] if again is None else again.flatten().tolist())
       pixel_change=None if previous is None else float(np.mean(np.abs(roi.astype(float)-previous.astype(float))))
       previous=roi.copy()
       row=dict(roi_sha256=hashlib.sha256(roi.tobytes()).hexdigest(),roi_mean_abs_change=pixel_change,fixed_pixel_decodes=fixed_decodes,tag=tag,curved=curved,subdivision=subdiv,distance_m=distance,angle_deg=angle,repeat=repeat,decoded=[] if ids is None else ids.flatten().tolist());rows.append(row);cv2.imwrite(str(out/f'frame_{len(rows):02d}.png'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR));print(row,flush=True)
       (out/'results.json').write_text(json.dumps(dict(rows=rows,scope='24 static synthetic views:two colors/tags,two bodies,three rigid assembly angles,two repetitions at20cm;one front tag;no robot or video'),indent=2)+'\n')
finally:app.close()
