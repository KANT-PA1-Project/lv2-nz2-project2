"""Sampled self-collision audit; never execute accepted endpoint solutions."""
import json,math
from pathlib import Path
import numpy as np
from audit_r12_usd_arm_path import CollisionModel
root=Path(__file__).resolve().parents[1];out=root.parent.parent/'outputs/r12/learned_ik_v1';report=json.loads((out/'evaluation.json').read_text());cfg=json.loads((root/'configs/r12_robot.json').read_text());home=np.array(cfg['home']);model=CollisionModel();rows=[]
accepted=[r for r in report['rows'] if r['accepted']];home_hits=model.hits(home)
for i,r in enumerate(accepted):
 q=np.array(r['q']);endpoint=model.hits(q);item={'index':r['index'],'endpoint_hits':endpoint,'path_checked':False}
 # Bounded, deterministic evenly spaced subset across the accepted test set.
 if i in set(np.linspace(0,len(accepted)-1,20,dtype=int)):
  count=math.ceil(np.max(np.abs(q-home))/.05)+1;hits=[]
  for j,t in enumerate(np.linspace(0,1,count)):
   h=model.hits(home+t*(q-home))
   if h:hits.append({'sample':j,'pairs':h})
  item.update(path_checked=True,samples=count,path_hits=hits)
 rows.append(item)
 if i%20==0:print('checked endpoints',i,flush=True)
result={'home_q':home.tolist(),'home_hits':home_hits,'accepted_endpoint_count':len(accepted),'colliding_endpoint_count':sum(bool(r['endpoint_hits']) for r in rows),'paths_checked':sum(r['path_checked'] for r in rows),'colliding_sampled_paths':sum(bool(r.get('path_hits')) for r in rows),'max_joint_sample_spacing_rad':.05,'rows':rows,'scope':'Composed USD enabled cube self-collision only, open gripper, adjacent disabled pairs excluded. Straight joint interpolation from the same label home; sampled subset paths, not trajectory planner. Environment, continuous collision, velocity/torque and perception uncertainty not validated. No commands executed.'}
(out/'collision_audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='rows'}))
