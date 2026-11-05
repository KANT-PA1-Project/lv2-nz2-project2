"""Authored geometry used only for offline scoring, never control inputs."""
import json,math
from pathlib import Path
import numpy as np
from arm_practice.observation_readiness import iou
root=Path(__file__).resolve().parents[3]/'outputs/r12';folder=root/'joint_observation_v262'
truth=json.loads((folder/'results.json').read_text())['rows']
inference=json.loads((folder/'inference.json').read_text())['responses']
assert len(truth)==len(inference)==12
rows=[]
for t,r in zip(truth,inference):
 expected=('red' if t['tag']==0 else 'green')+('_cylinder' if t['curved'] else '_cuboid')
 with np.load(folder/r['source']) as f:depth=f['depth']
 mask=np.isfinite(depth)&(depth>.01)&(depth<t['distance_m']+.061)
 yy,xx=np.where(mask);bbox=[int(xx.min()),int(yy.min()),int(xx.max()+1),int(yy.max()+1)]
 # Camera default looks along -Z; object center is 15.03mm behind tag plane.
 center_z=t['distance_m']+.01503
 accepted=[x['id'] for x in r['response']['tags']]
 candidates=[]
 checks={c['index']:c for c in r['checks']}
 for index,d in enumerate(r['response']['detections']):
  if d['class_id']>=4:continue
  p=d.get('pose',{});center=p.get('center_camera_m')
  error=None if center is None else math.sqrt(center[0]**2+center[1]**2+(center[2]-center_z)**2)
  candidates.append(dict(index=index,prediction=d['class_name'],class_correct=d['class_name']==expected,
    confidence=d['confidence'],silhouette_iou=iou(d['bbox'],bbox),center_error_m=error,
    center_z_error_m=None if center is None else center[2]-center_z,
    content_ready=checks[index]['content_ready'],content_reason=checks[index]['content_reason']))
 rows.append(dict(source=r['source'],expected_class=expected,expected_id=t['tag'],distance_to_tag_m=t['distance_m'],
  diagnostic_body_center_z_m=center_z,diagnostic_silhouette_bbox=bbox,accepted_ids=accepted,
  wrong_accepted_ids=[x for x in accepted if x!=t['tag']],candidates=candidates))
passed=[c for r in rows for c in r['candidates'] if c['content_ready']]
allc=[c for r in rows for c in r['candidates']]
summary=dict(frames=12,accepted_tag_frames=sum(bool(r['accepted_ids']) for r in rows),
 wrong_accepted_ids=sum(len(r['wrong_accepted_ids']) for r in rows),
 top_score_class_correct=sum(max(r['candidates'],key=lambda c:c['confidence'])['class_correct'] for r in rows),
 content_passes=len(passed),content_pass_wrong_class=sum(not c['class_correct'] for c in passed),
 content_pass_center_error_range_m=[min(c['center_error_m'] for c in passed),max(c['center_error_m'] for c in passed)],
 all_candidate_center_error_range_m=[min(c['center_error_m'] for c in allc),max(c['center_error_m'] for c in allc)],
 performance_pass=False,runtime_changed=False,
 conclusion='Single-label cylinder misclassification passes existing content gates; overlap rejection alone cannot validate shape.',
 limitations='12 simple front views,2 IDs,no mission clutter or independent repetitions;render depth silhouette and authored center are diagnostic ground truth only.')
(root/'v263_joint_truth_audit.json').write_text(json.dumps(dict(summary=summary,rows=rows),indent=2)+'\n')
print(json.dumps(summary,indent=2))
