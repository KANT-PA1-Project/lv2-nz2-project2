import json
from pathlib import Path
from collections import defaultdict
out=Path(__file__).resolve().parents[3]/'outputs/r12';m=json.loads((out/'mission_repeat_v276/manifest.json').read_text());d=json.loads((out/'v276_transport_benchmark.json').read_text());reports=[]
for e in d['reports']:
 groups=defaultdict(list)
 for r in e['rows']:
  t=m['rows'][r['index']];groups[(t['translation_from_reference_m'],t['light_intensity'])].append(r)
 result=[]
 for (offset,light),rows in groups.items():
  hit=[bool(r['accepted']) and set(r['accepted'])=={4} for r in rows];gap=0;maxgap=0
  for h in hit:
   gap=0 if h else gap+1;maxgap=max(gap,maxgap)
  result.append(dict(offset_m=offset,light_intensity=light,frames=len(rows),id4_frames=sum(hit),duplicate_same_id_frames=sum(len(r['accepted'])>1 and set(r['accepted'])=={4} for r in rows),wrong_id_frames=sum(any(x!=4 for x in r['accepted']) for r in rows),longest_miss_run=maxgap,sequence=hit))
 reports.append(dict(environment=e['environment'],completed=e['completed'],groups=result))
r=dict(frames=len(m['rows']),reports=reports,scope='10 consecutive static renders per fixed pose/light;not10 independent scenes;ID4 is diagnostic target,other decoded IDs require spatial review',performance_pass=False,runtime_changed=False,training_started=False)
(out/'v276_repeatability.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
