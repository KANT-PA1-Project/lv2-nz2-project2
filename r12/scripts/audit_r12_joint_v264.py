"""Score static oblique views; authored labels are diagnostic only."""
import json
from pathlib import Path
from collections import defaultdict
root=Path(__file__).resolve().parents[3]/'outputs/r12';d=root/'joint_observation_v264'
truth=json.loads((d/'results.json').read_text())['rows'];data=json.loads((d/'inference.json').read_text());assert data['completed']
assert len(truth)==len(data['responses'])==24
rows=[];groups=defaultdict(lambda:dict(frames=0,top_class_correct=0,tag_correct=0,wrong_tag=0,content_pass=0,wrong_class_content_pass=0))
for t,r in zip(truth,data['responses']):
 expected=('red' if t['tag']==0 else 'green')+('_cylinder' if t['curved'] else '_cuboid')
 ds=r['response']['detections'];pucks=[x for x in ds if 0<=x['class_id']<4]
 top=max(pucks,key=lambda x:x['confidence']) if pucks else None
 tags=[x['id'] for x in r['response']['tags']]
 passes=[ds[c['index']] for c in r['checks'] if c['content_ready']]
 row=dict(source=r['source'],expected_class=expected,angle=t['angle_deg'],repeat=t['repeat'],top_class=None if top is None else top['class_name'],top_class_correct=top is not None and top['class_name']==expected,accepted_ids=tags,tag_correct=tags==[t['tag']],wrong_tag=any(x!=t['tag'] for x in tags),content_pass=len(passes),wrong_class_content_pass=sum(x['class_name']!=expected for x in passes))
 rows.append(row);g=groups[str(t['angle_deg'])];g['frames']+=1
 for k in ['top_class_correct','tag_correct','wrong_tag','content_pass','wrong_class_content_pass']:g[k]+=row[k]
summary=dict(frames=24,by_angle=dict(groups),scope='2 colors,2 shapes,3 angles,2 repeats at20cm;simple isolated scene',runtime_changed=False,training_started=False,performance_pass=False)
(root/'v264_audit.json').write_text(json.dumps(dict(summary=summary,rows=rows),indent=2)+'\n');print(json.dumps(summary,indent=2))
