"""Identical saved frames; fixed photometry; projections used after decoding only."""
import json,hashlib
from pathlib import Path
from collections import defaultdict,Counter
import cv2,numpy as np
from arm_practice.tag_detection import agreed_tags
from arm_practice.observation_readiness import iou
out=Path(__file__).resolve().parents[3]/'outputs/r12';cv2.setNumThreads(1)
lut=np.round((np.arange(256)/255.)**2*255).astype(np.uint8)
det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11));rows=[]
for kind,folder in [('positive','mission_holdout_v282'),('negative','mission_holdout_negative_v282')]:
 src=out/folder
 for r in json.loads((src/'manifest.json').read_text())['rows']:
  raw=cv2.cvtColor(cv2.imread(str(src/f'frame_{r["index"]:02d}.png')),cv2.COLOR_BGR2GRAY)
  for mode in ['baseline','gamma2']:
   gray=raw if mode=='baseline' else cv2.LUT(raw,lut);obs=[];votes={};rejected_all={}
   for scale in [1,2,3,4]:
    scaled=gray if scale==1 else cv2.resize(gray,None,fx=scale,fy=scale,interpolation=cv2.INTER_CUBIC)
    corners,ids,rejected=det.detectMarkers(scaled);votes[scale]=[] if ids is None else ids.flatten().tolist()
    if ids is not None:obs.extend((scale,int(identity),c[0]/scale) for c,identity in zip(corners,ids.flatten()))
    rejected_all[scale]=[[*(c[0]/scale).min(0).tolist(),*(c[0]/scale).max(0).tolist()] for c in rejected]
   tags=agreed_tags(obs);reason='accepted' if tags else 'no_scale_decoded' if not obs else 'single_scale_only' if len({x[0] for x in obs})==1 else 'consensus_rejected'
   b=next(p['bbox'] for p in r['diagnostic_projections'] if p['path'].endswith('/Puck4/top/Surface'));x0,y0=np.maximum(0,np.floor(b[:2]).astype(int));x1,y1=np.minimum([gray.shape[1],gray.shape[0]],np.ceil(b[2:]).astype(int));roi=gray[y0:y1,x0:x1];q=np.quantile(roi,[.05,.5,.95])
   rows.append(dict(kind=kind,index=r['index'],offset=r['translation_from_reference_m'],light=r['light_intensity'],mode=mode,pixel_sha256=hashlib.sha256(gray.tobytes()).hexdigest(),votes=votes,reason=reason,tags=tags,roi_quantiles=q.tolist(),roi_white_clip=float(np.mean(roi>=254)),rejected_near={scale:sum(iou(c,b)>.2 for c in cs) for scale,cs in rejected_all.items()}))
groups=defaultdict(list)
for r in rows:groups[(r['kind'],r['mode'],r['light'])].append(r)
summary=[dict(kind=k[0],mode=k[1],light=k[2],frames=len(rs),id4=sum(any(t['id']==4 for t in r['tags']) for r in rs),any_detection=sum(bool(r['tags']) for r in rs),reasons=dict(Counter(r['reason'] for r in rs)),median_roi_quantiles=np.median([r['roi_quantiles'] for r in rs],axis=0).tolist(),max_white_clip=max(r['roi_white_clip'] for r in rs),rejected_near_frames=sum(any(r['rejected_near'].values()) for r in rs)) for k,rs in groups.items()]
report=dict(opencv=cv2.__version__,rows=rows,summary=summary,scope='Offline fixed gamma2 comparison. ROI includes border/background and is diagnostic only;rejected contour overlap does not prove correct contour;no runtime change.')
(out/f'v283_environment_{cv2.__version__}.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(summary))
