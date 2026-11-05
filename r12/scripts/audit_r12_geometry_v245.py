"""Post-hoc geometry audit, not an independently validated rejection rule."""
import json
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[1];out=root.parents[1]/'outputs/r12'
a=json.loads((out/'v243_visible_puck_audit.json').read_text())['annotations']
s=json.loads((out/'v241_worker_replay/results.json').read_text())['samples']
r=json.loads((out/'v244_detector_audit.json').read_text())['rows']
def area(b):return max(0,b[2]-b[0])*max(0,b[3]-b[1])
def inter(a,b):return max(0,min(a[2],b[2])-max(a[0],b[0]))*max(0,min(a[3],b[3])-max(a[1],b[1]))
def iou(a,b):
 v=inter(a,b);return v/(area(a)+area(b)-v)
results=[]
for row in r:
 if row['mode'] not in ('full1280','fixed_tiles800'):continue
 ann=a[0 if row['camera']=='r07_camera1' else 1];sample=s[ann['sample_index']]
 with np.load(sample['scene'],allow_pickle=False) as f:depth=f['depth'] if row['camera']=='r07_camera1' else f[row['camera']+'_depth']
 details=[]
 for d in row['detections']:
  x,y,u,v=d['bbox'];cx=int((x+u)/2);cy=int((y+v)/2)
  patch=depth[max(0,cy-2):min(480,cy+3),max(0,cx-2):min(640,cx+3)]
  valid=patch[np.isfinite(patch)&(patch>.02)&(patch<10)]
  z=float(np.median(valid)) if valid.size else None
  details.append(dict(d,depth_m=z,extent_m=None if z is None else max(u-x,v-y)*z/400))
 for limit in (.10,.12,.15):
  counts=dict(tp=0,fp=0,fn=0,class_wrong=0,ignored=0);used=set();rejected=[]
  for d in sorted(details,key=lambda d:-d['confidence']):
   if d['extent_m'] is None or d['extent_m']>limit:rejected.append(d);continue
   if any(inter(d['bbox'],box)/area(d['bbox'])>.5 for box in ann['ignore']):counts['ignored']+=1;continue
   score,j=max([(iou(d['bbox'],o['bbox']),j) for j,o in enumerate(ann['objects']) if j not in used],default=(0,-1))
   if score>=.5:
    used.add(j);counts['tp']+=1;counts['class_wrong']+=d['class_name']!=ann['objects'][j]['class_name']
   else:counts['fp']+=1
  counts['fn']=len(ann['objects'])-len(used)
  results.append(dict(camera=row['camera'],mode=row['mode'],limit_m=limit,counts=counts,detections=details))
report=dict(scope='same2 exploratory views; no independent generalization or latency claim',runtime_changed=False,results=results)
(out/'v245_geometry_audit.json').write_text(json.dumps(report,indent=2)+'\n')
for mode in ('full1280','fixed_tiles800'):
 for limit in (.10,.12,.15):
  matched=[x for x in results if x['mode']==mode and x['limit_m']==limit]
  print(mode,limit,{k:sum(x['counts'][k] for x in matched) for k in matched[0]['counts']})
