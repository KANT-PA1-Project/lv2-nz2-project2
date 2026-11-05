#!/usr/bin/env python3
"""Sample measured TCP trajectory against detailed CAD nonadjacent surfaces."""
from pathlib import Path
import json
# Reuse the existing detailed-mesh loader and collision functions, without its sweep.
p=Path(__file__).with_name('inspect_r12_geometry.py')
exec(compile(p.read_text().split('base=collisions')[0],str(p),'exec'))
data=json.loads((ROOT/'reports/local/r12_tcp_tracking.json').read_text())
results=[]
for case in data['results']:
 samples=case['samples']; indices=sorted(set(range(0,len(samples),10))|{len(samples)-1})
 hits=[]
 for i in indices:
  nonadj=[pair for pair in collisions(samples[i]['q']) if frozenset(pair) not in adjacent]
  if nonadj:hits.append({'sample':i,'pairs':nonadj})
 results.append({'name':case['name'],'checked_samples':len(indices),'nonadjacent_hits':hits})
print(json.dumps(results),flush=True)
(ROOT/'reports/local/r12_tcp_collisions.json').write_text(json.dumps({'scope':'Sampled nonadjacent CAD triangle intersections only; adjacent baseline overlaps excluded. Not continuous collision proof or object grasp verification.','results':results},indent=2))
