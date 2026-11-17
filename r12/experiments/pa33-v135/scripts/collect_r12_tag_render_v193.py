"""Static camera render pilot. No robot, physics stepping, control, or video writer."""
import sys,json,time,hashlib
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(root/'src'),str(root/'scripts/sensor_runtime')]
out=root.parent.parent/'outputs/r12/tag_render_v193';out.mkdir(exist_ok=False)
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','anti_aliasing':0})
import omni.usd,omni.replicator.core as rep
from pxr import UsdGeom,UsdShade,UsdLux,Sdf,Gf
from camera_rig import transforms
import cv2
stage=omni.usd.get_context().get_stage();rig=transforms(root,{f'arm_joint{i}':0 for i in range(1,6)})
light=UsdLux.DomeLight.Define(stage,'/World/Light');light.CreateIntensityAttr(1500.)
cam=UsdGeom.Camera.Define(stage,'/World/AcquisitionCamera');op=cam.AddTransformOp();cam.CreateHorizontalApertureAttr(20.955);cam.CreateVerticalApertureAttr(15.71625);cam.CreateFocalLengthAttr(13.096875);cam.CreateClippingRangeAttr(Gf.Vec2f(.01,30))
rp=rep.create.render_product(str(cam.GetPath()),(640,480));ann=rep.AnnotatorRegistry.get_annotator('rgb');ann.attach(rp)
mat=UsdShade.Material.Define(stage,'/World/Material');sh=UsdShade.Shader.Define(stage,'/World/Material/Shader');sh.CreateIdAttr('UsdPreviewSurface');sh.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.7);mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),'surface')
uv=UsdShade.Shader.Define(stage,'/World/Material/UV');uv.CreateIdAttr('UsdPrimvarReader_float2');uv.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st')
tex=UsdShade.Shader.Define(stage,'/World/Material/Texture');tex.CreateIdAttr('UsdUVTexture');tex.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(uv.ConnectableAPI(),'result');asset=tex.CreateInput('file',Sdf.ValueTypeNames.Asset);sh.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(),'rgb')
mesh=UsdGeom.Mesh.Define(stage,'/World/Target');mesh.CreateFaceVertexCountsAttr([4]);mesh.CreateFaceVertexIndicesAttr([0,1,2,3]);mesh.CreateDoubleSidedAttr(True);mesh.CreateSubdivisionSchemeAttr('none');UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,UsdGeom.Tokens.faceVarying).Set([(0,1),(1,1),(1,0),(0,0)]);UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11));rows=[]
try:
 for group,tag in enumerate([0,8,10]):
  texture=root/f'models/navigation/textures/tag36h11-{tag}.png';asset.Set(str(texture))
  for name in ['r07_camera1','r07_camera2','r07_camera3','arm_camera']:
   T=rig[name+'_left_optical'];op.Set(Gf.Matrix4d(*(T@np.diag([1.,-1.,-1.,1.])).T.flatten().tolist()))
   for distance in [.15,.30,.60]:
    pts=np.array([[-.01,-.01,distance,1],[.01,-.01,distance,1],[.01,.01,distance,1],[-.01,.01,distance,1]])
    mesh.CreatePointsAttr([(float(p[0]),float(p[1]),float(p[2])) for p in (T@pts.T).T])
    for _ in range(12):app.update()
    rep.orchestrator.step(rt_subframes=4)
    stamp=time.monotonic_ns();rgb=np.asarray(ann.get_data())[:,:,:3].copy();assert rgb.shape==(480,640,3)
    # Fixed central crop: projected 20mm target plus 25% padding, without runtime control use.
    half=int(np.ceil(400*.01/distance*1.5));gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY);crop=gray[240-half:240+half,320-half:320+half];scale=min(1.,60/max(crop.shape));small=cv2.resize(crop,(round(crop.shape[1]*scale),round(crop.shape[0]*scale)),interpolation=cv2.INTER_AREA);im=np.full((64,64),255,np.uint8);h,w=small.shape;im[(64-h)//2:(64-h)//2+h,(64-w)//2:(64-w)//2+w]=small
    _,ids,_=det.detectMarkers(im);decoded=[] if ids is None else ids.flatten().tolist();idx=len(rows);cv2.imwrite(str(out/f'crop_{idx:03d}.png'),im)
    rows.append(dict(index=idx,camera=name,capture_monotonic_ns=stamp,tag_id=tag,distance_m=distance,split=['train','validation','test'][group],decoded_ids=decoded,readable=tag in decoded,texture_sha256=hashlib.sha256(texture.read_bytes()).hexdigest()))
    (out/'progress.json').write_text(json.dumps(dict(samples=len(rows),positive=sum(r['readable'] for r in rows),planned=36)))
    print('sample',idx,name,distance,decoded,flush=True)
 (out/'manifest.json').write_text(json.dumps(dict(rows=rows,scope='Controlled static renderer pilot, not independent mission-camera validation. Identical views across cameras are grouped by tag identity. No robot or motion; no video. Known fixture geometry used only to place/crop training targets, never as control observation.',opencv=cv2.__version__,complete=True),indent=2))
finally:app.close()
