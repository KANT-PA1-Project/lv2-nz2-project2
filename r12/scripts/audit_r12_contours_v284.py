"""Full-frame decoding followed by offline geometric association."""
import json
from pathlib import Path
import cv2,numpy as np
from arm_practice.observation_readiness import iou
out=Path(__file__).resolve().parents[3]/'outputs/r12';src=out/'mission_holdout_v282';meta=json.loads((src/'manifest.json').read_text())['rows'];cv2.setNumThreads(1)
lut=np.round((np.arange(256)/255.)**2*255).astype(np.uint8);det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11));rows=[]
for r in meta:
 if r['light_intensity']!=2400 and r['index']!=4:continue
 gray=cv2.LUT(cv2.cvtColor(cv2.imread(str(src/f'frame_{r["index"]:02d}.png')),cv2.COLOR_BGR2GRAY),lut);records=[]
 for scale in [1,2,3,4]:
  im=gray if scale==1 else cv2.resize(gray,None,fx=scale,fy=scale,interpolation=cv2.INTER_CUBIC)
  cs,ids,rejected=det.detectMarkers(im)
  candidates=[('accepted',int(i),c) for c,i in zip(cs,[] if ids is None else ids.flatten())]+[('rejected',None,c) for c in rejected]
  for kind,identity,c in candidates:
   c=c[0]/scale;b=[*c.min(0).tolist(),*c.max(0).tolist()];p=max(r['diagnostic_projections'],key=lambda p:iou(b,p['bbox']));score=iou(b,p['bbox'])
   if score<.2:continue
   records.append(dict(scale=scale,kind=kind,id=identity,corners=c.tolist(),center=c.mean(0).tolist(),nearest_surface=p['path'],iou=score,edge_lengths=np.linalg.norm(c-np.roll(c,1,axis=0),axis=1).tolist()))
 rows.append(dict(index=r['index'],light=r['light_intensity'],records=records))
report=dict(opencv=cv2.__version__,rows=rows,scope='Projection association only;not occlusion proof;decoder receives full frame only')
(out/f'v284_contours_{cv2.__version__}.json').write_text(json.dumps(report,indent=2)+'\n')
for r in rows:print(r['index'],[(x['scale'],x['kind'],x['id'],x['nearest_surface'],round(x['iou'],2)) for x in r['records']])
