import json
from pathlib import Path
from collections import Counter,defaultdict
import cv2
from arm_practice.tag_detection import agreed_tags
from arm_practice.observation_readiness import iou
root=Path(__file__).resolve().parents[3]/'outputs/r12';d=root/'joint_observation_v264'
truth=json.loads((d/'results.json').read_text())['rows'];responses=json.loads((d/'inference.json').read_text())['responses']
det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11));rows=[];groups=defaultdict(Counter)
for t,r in zip(truth,responses):
 image=cv2.imread(str(d/r['source'].replace('.npz','.png')));gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY);obs=[];votes={}
 for scale in (1,2,3,4):
  im=gray if scale==1 else cv2.resize(gray,None,fx=scale,fy=scale,interpolation=cv2.INTER_CUBIC)
  corners,ids,_=det.detectMarkers(im);votes[scale]=[] if ids is None else ids.flatten().tolist()
  if ids is not None:obs.extend((scale,int(identity),corner[0]/scale) for corner,identity in zip(corners,ids.flatten()))
 accepted=agreed_tags(obs)
 reason='accepted' if accepted else 'no_scale_decoded' if not obs else 'single_scale_only' if len({x[0] for x in obs})==1 else 'consensus_rejected'
 ds=sorted([x for x in r['response']['detections'] if x['class_id']<4],key=lambda x:-x['confidence'])
 expected=('red' if t['tag']==0 else 'green')+('_cylinder' if t['curved'] else '_cuboid')
 row=dict(source=r['source'],angle=t['angle_deg'],curved=t['curved'],repeat=t['repeat'],votes=votes,reason=reason,accepted=accepted,
  expected_class=expected,top_class=None if not ds else ds[0]['class_name'],
  predictions=[dict(name=x['class_name'],confidence=x['confidence'],bbox=x['bbox']) for x in ds],
  top_two_iou=iou(ds[0]['bbox'],ds[1]['bbox']) if len(ds)>1 else None,
  wrong_ids=[x[1] for x in obs if x[1]!=t['tag']]);rows.append(row)
 groups[str(t['angle_deg'])][reason]+=1
 if ds and ds[0]['class_name']!=expected:groups[str(t['angle_deg'])]['wrong_top_class']+=1
summary=dict(opencv_version=cv2.__version__,frames=len(rows),by_angle={k:dict(v) for k,v in groups.items()},wrong_raw_ids=sum(len(r['wrong_ids']) for r in rows),runtime_changed=False,training_started=False,scope='Per-scale re-decode of exact V264 saved BGR pixels;not new independent images')
(root/'v265_scale_votes.json').write_text(json.dumps(dict(summary=summary,rows=rows),indent=2)+'\n');print(json.dumps(summary,indent=2))
