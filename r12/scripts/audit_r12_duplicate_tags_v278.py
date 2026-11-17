"""Compare observed tag quadrilaterals and diagnostic surface projections."""
import json
from pathlib import Path
import cv2,numpy as np
from arm_practice.tag_detection import detect_tags
from arm_practice.observation_readiness import iou
out=Path(__file__).resolve().parents[3]/'outputs/r12';src=out/'mission_repeat_v276';m=json.loads((src/'manifest.json').read_text())['rows'];prior=json.loads((out/'v277_photometric_4.14.0.json').read_text())['rows'];cv2.setNumThreads(1)
det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11));rows=[]
for p in prior:
 if len(p['accepted_ids'])<2:continue
 index=p['index'];tags=detect_tags(cv2.imread(str(src/f'frame_{index:02d}.png')),det);records=[]
 for tag in tags:
  c=np.array(tag['corners']);b=[*c.min(0).tolist(),*c.max(0).tolist()]
  best=max(m[index]['diagnostic_projections'],key=lambda x:iou(b,x['bbox']))
  records.append(dict(**tag,bbox=b,nearest_surface=best['path'],projection_iou=iou(b,best['bbox'])))
 pairs=[]
 for i,a in enumerate(records):
  for b in records[i+1:]:
   ca=np.array(a['corners']);cb=np.array(b['corners']);rms=min(float(np.sqrt(np.mean(np.sum((ca-np.roll(cb,k,axis=0))**2,axis=1)))) for k in range(4))
   pairs.append(dict(ids=[a['id'],b['id']],bbox_iou=iou(a['bbox'],b['bbox']),center_distance_px=float(np.linalg.norm(ca.mean(0)-cb.mean(0))),corner_rms_px=rms,same_nearest_surface=a['nearest_surface']==b['nearest_surface'],shared_scales=sorted(set(a['agreeing_scales'])&set(b['agreeing_scales']))))
 rows.append(dict(index=index,tags=records,pairs=pairs))
report=dict(opencv=cv2.__version__,frames=len(rows),rows=rows,scope='Projected nearest surface is diagnostic association only,not occlusion proof;no merge or acceptance changes')
(out/'v278_duplicate_audit.json').write_text(json.dumps(report,indent=2)+'\n')
for r in rows:print(r['index'],[(t['nearest_surface'],t['agreeing_scales']) for t in r['tags']],r['pairs'])
