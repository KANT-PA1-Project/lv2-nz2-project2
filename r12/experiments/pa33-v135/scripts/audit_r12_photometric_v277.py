"""Exact saved-image audit; fixture ROI used only for photometric scoring."""
import json
from pathlib import Path
from collections import defaultdict,Counter
import cv2,numpy as np
from arm_practice.tag_detection import agreed_tags
from arm_practice.observation_readiness import iou
out=Path(__file__).resolve().parents[3]/'outputs/r12';src=out/'mission_repeat_v276';meta=json.loads((src/'manifest.json').read_text())['rows'];cv2.setNumThreads(1)
det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11));rows=[]
for r in meta:
 b=next(p['bbox'] for p in r['diagnostic_projections'] if p['path'].endswith('/Puck4/top/Surface'))
 im=cv2.imread(str(src/f'frame_{r["index"]:02d}.png'));gray=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
 x0,y0=np.floor(b[:2]).astype(int);x1,y1=np.ceil(b[2:]).astype(int);roi=gray[y0:y1,x0:x1]
 obs=[];votes={};rejected_near={}
 for scale in [1,2,3,4]:
  scaled=gray if scale==1 else cv2.resize(gray,None,fx=scale,fy=scale,interpolation=cv2.INTER_CUBIC)
  corners,ids,rejected=det.detectMarkers(scaled);votes[scale]=[] if ids is None else ids.flatten().tolist()
  if ids is not None:obs.extend((scale,int(identity),c[0]/scale) for c,identity in zip(corners,ids.flatten()))
  rejected_near[scale]=sum(iou([*(c[0]/scale).min(0).tolist(),*(c[0]/scale).max(0).tolist()],b)>.2 for c in rejected)
 tags=agreed_tags(obs);reason='accepted' if tags else 'no_scale_decoded' if not obs else 'single_scale_only' if len({x[0] for x in obs})==1 else 'consensus_rejected'
 q=np.quantile(roi,[.05,.5,.95]);rows.append(dict(index=r['index'],offset_m=r['translation_from_reference_m'],light=r['light_intensity'],repeat=r['repeat'],votes=votes,rejected_quads_near_roi=rejected_near,reason=reason,accepted_ids=[t['id'] for t in tags],roi_q05=float(q[0]),roi_median=float(q[1]),roi_q95=float(q[2]),roi_span=float(q[2]-q[0]),roi_white_clip_fraction=float(np.mean(roi>=254)),roi_black_clip_fraction=float(np.mean(roi<=1)),roi_bbox=b))
groups=defaultdict(list)
for r in rows:groups[(r['offset_m'],r['light'])].append(r)
summary=[]
for (offset,light),rs in groups.items():
 summary.append(dict(offset_m=offset,light=light,frames=len(rs),reasons=dict(Counter(r['reason'] for r in rs)),median_roi_q05=float(np.median([r['roi_q05'] for r in rs])),median_roi_q95=float(np.median([r['roi_q95'] for r in rs])),median_roi_span=float(np.median([r['roi_span'] for r in rs])),median_white_clip_fraction=float(np.median([r['roi_white_clip_fraction'] for r in rs])),frames_with_rejected_quad_near_roi=sum(any(r['rejected_quads_near_roi'].values()) for r in rs)))
report=dict(opencv=cv2.__version__,summary=summary,rows=rows,scope='Bounding ROI includes margin/background;not calibrated black/white cell values. Rejected quads near ROI do not prove correct tag contour. No image or runtime changes.')
(out/f'v277_photometric_{cv2.__version__}.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(summary,indent=2))
