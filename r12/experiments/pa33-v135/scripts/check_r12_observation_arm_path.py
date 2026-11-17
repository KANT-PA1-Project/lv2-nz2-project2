"""Offline sampled CAD self-collision check for observation candidate motions."""
import json
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[1]
p=root/'scripts/inspect_r12_geometry.py'
space={'__file__':str(p),'__name__':'geometry_library'}
exec(compile(p.read_text().split('base=collisions')[0],str(p),'exec'),space)
a=root.parent.parent/'outputs/r12/v26_object_docking.json';data=json.loads(a.read_text());results=[]
for index,candidate in enumerate(data['candidates'][:5]):
 hits=[]
 for step,q in enumerate(np.linspace([0,.33,.33,.33,0],candidate['arm_q'],41)):
  pairs=[pair for pair in space['collisions'](q) if frozenset(pair) not in space['adjacent']]
  if pairs:hits.append({'sample':step,'pairs':pairs})
 results.append({'candidate_index':index,'sample_count':41,'self_collision_samples':hits})
result={'results':results,'scope':'Detailed CAD nonadjacent triangle surface intersections, sampled linear joint path. Does not test environment, gripper geometry, swept-volume continuity or execute motion.'}
(root.parent.parent/'outputs/r12/v26_observation_arm_collision.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
