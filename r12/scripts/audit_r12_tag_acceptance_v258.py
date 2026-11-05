"""Offline acceptance comparison on previously consumed V230 evaluation images."""
import json,sys
from pathlib import Path
import cv2
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'src'))
from arm_practice.tag_detection import agreed_tags
out=root.parents[1]/'outputs/r12';src=out/'tag_render_v230';meta=json.loads((src/'manifest.json').read_text())['rows'];cv2.setNumThreads(1);det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11));rows=[]
for row in meta:
 gray=cv2.cvtColor(cv2.imread(str(src/f'frame_{row["index"]:03d}.png')),cv2.COLOR_BGR2GRAY);obs=[]
 for scale in (1,2,3,4):
  im=gray if scale==1 else cv2.resize(gray,None,fx=scale,fy=scale,interpolation=cv2.INTER_CUBIC)
  corners,ids,_=det.detectMarkers(im)
  if ids is not None:obs.extend((scale,int(tag),c[0]/scale) for c,tag in zip(corners,ids.flatten()))
 rows.append(dict(index=row['index'],expected=row['tag_id'],strict=[x['id'] for x in agreed_tags(obs)],any_scale=sorted({x[1] for x in obs}),native=sorted({x[1] for x in obs if x[0]==1})))
summary={}
for mode in ['native','strict','any_scale']:
 positives=[r for r in rows if r['expected']>=0];negatives=[r for r in rows if r['expected']<0]
 summary[mode]=dict(correct_positive_frames=sum(r[mode]==[r['expected']] for r in positives),positive_total=len(positives),wrong_id_positive_frames=sum(any(t!=r['expected'] for t in r[mode]) for r in positives),negative_false_positive_frames=sum(bool(r[mode]) for r in negatives),negative_total=len(negatives))
r=dict(summary=summary,rows=rows,scope='Previously evaluated controlled V230 images; same renderer,72positive/72negative; not new holdout or mission/hardware validation',runtime_changed=False)
(out/'v258_acceptance_audit.json').write_text(json.dumps(r,indent=2)+'\n');print(summary)
