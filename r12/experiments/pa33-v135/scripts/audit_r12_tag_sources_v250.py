"""Texture-only sanity and ideal downsampling control, not scene validation."""
import json,sys
from pathlib import Path
import numpy as np,cv2
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'src'))
from arm_practice.tag_detection import detect_tags
out=root.parents[1]/'outputs/r12';det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11));cv2.setNumThreads(1)
controls=[];small=[]
for tag in range(12):
 path=root/f'models/navigation/textures/tag36h11-{tag}.png';image=cv2.imread(str(path),0)
 _,ids,_=det.detectMarkers(image);controls.append(dict(expected=tag,decoded=[] if ids is None else ids.flatten().tolist(),shape=list(image.shape)))
 for size in (8,10,12,16,20,24,32):
  reduced=cv2.resize(image,(size,size),interpolation=cv2.INTER_AREA)
  # Constant outer background avoids clipping detector boundaries.
  padded=cv2.copyMakeBorder(reduced,8,8,8,8,cv2.BORDER_CONSTANT,value=255)
  found=detect_tags(padded,det);small.append(dict(tag=tag,size=size,decoded=[x['id'] for x in found]))
summary=[dict(size=size,correct=sum(x['decoded']==[x['tag']] for x in small if x['size']==size),total=12) for size in (8,10,12,16,20,24,32)]
r=dict(source_controls=controls,ideal_downsampling=summary,rows=small,authored_tag_width_m=.02,configured_fx_px=400,ideal_frontal_width_px_at_depth_m={str(z):8/z for z in (.5,1,2,3)},mapping_review='Builder provides faceVarying st and explicit UsdPrimvarReader_float2 connection. Does not prove archived render mapping/orientation.',limitations=['Ideal frontal resampling only; no perspective/lighting/curvature/occlusion','Source textures at present may differ from historical capture; no historical hash available','IDs only used to evaluate source controls, not injected into perception'])
(out/'v250_tag_source_audit.json').write_text(json.dumps(r,indent=2)+'\n');print('source_correct',sum(x['decoded']==[x['expected']] for x in controls));print(summary)
