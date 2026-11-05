"""Static isolated surface diagnostic. No robot, physics stepping, or video."""
import sys,json
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[1];out=root.parents[1]/'outputs/r12/tag_surface_v252b';out.mkdir(exist_ok=False)
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
mesh=UsdGeom.Mesh.Define(stage,'/World/Target');mesh.CreatePointsAttr([(-.01,-.01,0),(.01,-.01,0),(.01,.01,0),(-.01,.01,0)]);mesh.CreateFaceVertexCountsAttr([4]);mesh.CreateFaceVertexIndicesAttr([0,1,2,3]);mesh.CreateDoubleSidedAttr(True);st=UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,UsdGeom.Tokens.faceVarying);UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11));rows=[]
try:
 for mapping,coords in [('mission',[(0,0),(1,0),(1,1),(0,1)]),('flipped_v',[(0,1),(1,1),(1,0),(0,0)])]:
  st.Set(coords)
  for subdiv in ['catmullClark','none']:
   mesh.CreateSubdivisionSchemeAttr(subdiv)
   for distance in [.2,.5]:
    op.Set(Gf.Vec3d(0,0,distance))
    for _ in range(12):app.update()
    rep.orchestrator.step(rt_subframes=4);rgb=np.asarray(ann.get_data())[:,:,:3].copy();gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY);_,ids,_=det.detectMarkers(gray)
    row=dict(mapping=mapping,subdivision=subdiv,distance_m=distance,decoded=[] if ids is None else ids.flatten().tolist());rows.append(row);cv2.imwrite(str(out/f'frame_{len(rows):02d}.png'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR));print(row,flush=True)
    (out/'results.json').write_text(json.dumps(dict(rows=rows,scope='Static single flat tag0; no robot; surface factors only, not full mission validation'),indent=2)+'\n')
finally:app.close()
