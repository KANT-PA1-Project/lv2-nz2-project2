"""Compare archived replay outputs to previously fixed visual annotations."""
import json
from pathlib import Path
root=Path(__file__).resolve().parents[1];out=root.parents[1]/'outputs/r12'
old=json.loads((out/'v241_worker_replay/results.json').read_text())['samples']
annotations={}
for a in json.loads((out/'v243_visible_puck_audit.json').read_text())['annotations']:
 s=old[a['sample_index']];annotations[(Path(s['scene']).parent.name,s['camera'])]=a
for a in json.loads((out/'v246_annotations.json').read_text())['annotations']:
 annotations[('terminal_observation_v166','r07_camera1' if a['sample_index']==0 else 'arm_camera')]=a
samples=json.loads((out/'v247_worker_replay/results.json').read_text())['samples']
def area(a):return max(0,a[2]-a[0])*max(0,a[3]-a[1])
def inter(a,b):return max(0,min(a[2],b[2])-max(a[0],b[0]))*max(0,min(a[3],b[3])-max(a[1],b[1]))
def iou(a,b):
 v=inter(a,b);return v/(area(a)+area(b)-v)
rows=[]
for sample in samples:
 key=(Path(sample['scene']).parent.name,sample['camera'])
 if key not in annotations:continue
 ann=annotations[key];used=set();counts=dict(tp=0,fp=0,fn=0,class_wrong=0,shape_unscored=0,depth_agree_tp=0,depth_agree_fp=0,ignored=0);matches=[]
 for d in sorted([d for d in sample['detection_records'] if 'pose' in d],key=lambda d:-d['confidence']):
  if any(inter(d['bbox'],b)/area(d['bbox'])>.5 for b in ann['ignore']):counts['ignored']+=1;continue
  score,j=max([(iou(d['bbox'],o['bbox']),j) for j,o in enumerate(ann['objects']) if j not in used],default=(0,-1))
  if score>=.5:
   used.add(j);counts['tp']+=1;counts['depth_agree_tp']+=bool(d['depth_consistent']);label=ann['objects'][j]['class_name'];unknown=label.endswith('_unknown');counts['shape_unscored']+=unknown
   wrong=d['class_name'].split('_')[0]!=label.split('_')[0] if unknown else d['class_name']!=label
   counts['class_wrong']+=wrong
   matches.append(dict(object_index=j,label=label,predicted=d['class_name'],confidence=d['confidence'],depth_consistent=d['depth_consistent'],pose_valid=d['pose'].get('valid'),translation_valid=d['pose'].get('translation_valid'),tag_ids=d.get('tag_ids',[])))
  else:counts['fp']+=1;counts['depth_agree_fp']+=bool(d['depth_consistent'])
 counts['fn']=len(ann['objects'])-len(used)
 rows.append(dict(scene=key[0],camera=key[1],repeat=sample['repeat'],counts=counts,matches=matches))
report=dict(scope='Previously used3 informative views,6 visible instances plus blank wrist; repeats are not independent examples',rows=rows,per_repeat=[dict(repeat=i,counts={k:sum(r['counts'][k] for r in rows if r['repeat']==i) for k in rows[0]['counts']}) for i in range(3)],runtime_changed=False)
(out/'v248_pose_match_audit.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report['per_repeat']))
for row in rows:
 if row['repeat']==0:print(row)
