"""Read-only per-scale evidence audit. No relaxation of runtime acceptance."""
import json,sys
from pathlib import Path
from collections import Counter
import cv2
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'src'))
from arm_practice.tag_detection import agreed_tags
out=root.parents[1]/'outputs/r12';src=out/'tag_surface_v254';meta=json.loads((src/'results.json').read_text())['rows'];cv2.setNumThreads(1);det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11));rows=[]
for i,row in enumerate(meta):
 gray=cv2.cvtColor(cv2.imread(str(src/f'frame_{i+1:02d}.png')),cv2.COLOR_BGR2GRAY);obs=[];votes={}
 for scale in (1,2,3,4):
  im=gray if scale==1 else cv2.resize(gray,None,fx=scale,fy=scale,interpolation=cv2.INTER_CUBIC)
  corners,ids,_=det.detectMarkers(im);votes[str(scale)]=[] if ids is None else ids.flatten().tolist()
  if ids is not None:obs.extend((scale,int(tag),corner[0]/scale) for corner,tag in zip(corners,ids.flatten()))
 accepted=[x['id'] for x in agreed_tags(obs)];correct_scales=[scale for scale,ids in votes.items() if row['tag'] in ids];wrong=[tag for ids in votes.values() for tag in ids if tag!=row['tag']]
 reason='accepted' if accepted==[row['tag']] else ('no_scale_decode' if not obs else 'conflicting_ids' if wrong else 'single_scale_only' if len(correct_scales)==1 else 'spatial_grouping_rejection')
 rows.append(dict(frame=i,tag=row['tag'],distance_m=row['distance_m'],angle_deg=row['angle_deg'],votes=votes,accepted=accepted,reason=reason))
r=dict(total=len(rows),reason_counts=dict(Counter(x['reason'] for x in rows)),native_only_lost=[x['frame'] for x in rows if x['tag'] in x['votes']['1'] and x['reason']!='accepted'],scale_correct_counts={str(k):sum(x['tag'] in x['votes'][str(k)] for x in rows) for k in (1,2,3,4)},rows=rows,scope='Existing exploratory48frames;expected IDs only for scoring;runtime not changed')
(out/'v257_scale_vote_audit.json').write_text(json.dumps(r,indent=2)+'\n');print({k:v for k,v in r.items() if k!='rows'})
