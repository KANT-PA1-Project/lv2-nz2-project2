"""Small visual audit only; annotations are approximate and never control inputs."""
import json
from pathlib import Path
root=Path(__file__).resolve().parents[1];out=root.parents[1]/'outputs/r12'
r=json.loads((out/'v241_worker_replay/results.json').read_text())
annotations=[dict(sample_index=4,objects=[dict(bbox=[86,389,111,423],class_name='red_cylinder'),dict(bbox=[105,359,130,390],class_name='red_cuboid')],ignore=[]),dict(sample_index=5,objects=[dict(bbox=[175,317,197,346],class_name='red_cuboid'),dict(bbox=[198,330,220,360],class_name='red_cuboid')],ignore=[[195,122,225,149]])]
def intersection(a,b):return max(0,min(a[2],b[2])-max(a[0],b[0]))*max(0,min(a[3],b[3])-max(a[1],b[1]))
def area(a):return (a[2]-a[0])*(a[3]-a[1])
def iou(a,b):
 v=intersection(a,b);return v/(area(a)+area(b)-v)
reports=[]
for mode in ['baseline','candidate_size_0_10m']:
 counts=dict(tp_localization=0,fp=0,fn=0,class_correct=0,class_wrong=0,ignored=0);details=[]
 for ann in annotations:
  row=r['samples'][ann['sample_index']];pred=[d for d in row['detection_records'] if 'pose' in d]
  if mode!='baseline':
   pred=[d for d in pred if d['depth_surface_m'] is not None and max(d['bbox'][2]-d['bbox'][0],d['bbox'][3]-d['bbox'][1])*d['depth_surface_m']/400<=.1]
  used=set()
  for d in sorted(pred,key=lambda d:-d['confidence']):
   if any(intersection(d['bbox'],box)/area(d['bbox'])>.5 for box in ann['ignore']):counts['ignored']+=1;continue
   candidates=[(iou(d['bbox'],o['bbox']),j) for j,o in enumerate(ann['objects']) if j not in used]
   score,j=max(candidates,default=(0,-1))
   if score>=.5:
    used.add(j);counts['tp_localization']+=1;correct=d['class_name']==ann['objects'][j]['class_name'];counts['class_correct' if correct else 'class_wrong']+=1
    details.append(dict(camera=row['camera'],predicted=d['class_name'],annotated=ann['objects'][j]['class_name'],iou=score))
   else:counts['fp']+=1
  counts['fn']+=len(ann['objects'])-len(used)
 reports.append(dict(mode=mode,counts=counts,matches=details))
result=dict(scope='2 archived views,4 manually annotated visible instances; not4 unique physical objects or full recall benchmark',annotations=annotations,iou_threshold=.5,reports=reports,filter_deployed=False,limitations=['Approximate model-authored visual boxes; no independent label review','Small distant pair excluded as ignore region','Only two views; candidate threshold selected after observing this data'])
(out/'v243_visible_puck_audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(reports))
