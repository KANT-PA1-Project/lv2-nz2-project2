"""Independent archive diagnostics; labels never enter controller observations."""
import json
from collections import Counter
from pathlib import Path
from arm_practice.observation_readiness import finite, iou

root=Path(__file__).resolve().parents[3]/'outputs/r12'
data=json.loads((root/'v247_worker_replay/results.json').read_text())
old=json.loads((root/'v243_visible_puck_audit.json').read_text())['annotations']
extra=json.loads((root/'v246_annotations.json').read_text())['annotations']
labels={('mission_low_speed_drive_v21','r07_camera1'):old[0],
        ('mission_low_speed_drive_v21','r07_camera2'):old[1],
        ('terminal_observation_v166','r07_camera1'):extra[0],
        ('terminal_observation_v166','arm_camera'):extra[1]}
rows=[];pairs=[]
for s in data['samples']:
 scene=Path(s['scene']).parent.name
 ds=s['detection_records']
 for index,d in enumerate(ds):
  if not 0<=d['class_id']<4:continue
  p=d.get('pose');depth=d.get('depth_surface_m')
  if p is None:reason='pose_not_computed'
  elif p.get('translation_valid') is not True:reason='translation_invalid'
  elif not finite(p.get('confidence')) or not .25<=p['confidence']<=1:reason='pose_confidence_below_gate'
  else:
   c=p.get('center_camera_m')
   if not isinstance(c,list) or len(c)!=3 or not all(finite(x) for x in c) or c[2]<=0:reason='invalid_center'
   elif not finite(depth) or not .02<depth<10:reason='invalid_surface_depth'
   elif abs(depth-c[2])>=.1:reason='depth_mismatch'
   else:reason='pose_depth_only_pass'
  rows.append(dict(scene=scene,camera=s['camera'],repeat=s['repeat'],detection_index=index,reason=reason))
 if s['repeat']!=0:continue
 for i,a in enumerate(ds):
  if not 0<=a['class_id']<4 or a['confidence']<.25:continue
  for j,b in enumerate(ds[i+1:],i+1):
   if not 0<=b['class_id']<4 or b['confidence']<.25 or a['class_id']==b['class_id']:continue
   overlap=iou(a['bbox'],b['bbox'])
   if overlap<.5:continue
   annotation=labels.get((scene,s['camera']))
   matches=[]
   if annotation is not None:
    for obj in annotation['objects']:
     overlaps=[iou(x['bbox'],obj['bbox']) for x in (a,b)]
     if max(overlaps)>=.5:matches.append(dict(label=obj['class_name'],ious=overlaps))
   pairs.append(dict(scene=scene,camera=s['camera'],indices=[i,j],iou=overlap,
     predictions=[dict(class_name=x['class_name'],confidence=x['confidence'],pose_computed='pose' in x) for x in (a,b)],
     label_status='view_unannotated' if annotation is None else 'annotated_view',matches=matches))
report=dict(scope='Independent pose/depth diagnostics only, no tag or freshness bypass',
 samples=len(data['samples']),candidate_records=len(rows),counts=dict(Counter(r['reason'] for r in rows)),
 repeat_zero_counts=dict(Counter(r['reason'] for r in rows if r['repeat']==0)),
 conflicts_repeat_zero=pairs,execution_authorized=False,performance_pass=False,
 limitations=['Repeated views are not independent scenes','Approximate prior labels, no new independent review','Pose/depth pass does not validate identity, rotation, physical accuracy or pickup'],rows=rows)
(root/'v261_pose_conflict_audit.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='rows'},indent=2))
