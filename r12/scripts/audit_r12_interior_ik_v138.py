"""Offline margin IK coverage and sampled self-collision audit; no commands."""
import json,math
from pathlib import Path
import numpy as np
from arm_practice.learned_ik import ArmFK,finish
from audit_r12_usd_arm_path import CollisionModel
root=Path(__file__).resolve().parents[1];out=root.parent.parent/'outputs/r12';bank=out/'learned_ik_v1'
cfg=json.loads((root/'configs/r12_robot.json').read_text());home=np.array(cfg['home']);fk=ArmFK(root/'models/r12/r12_arm.urdf',cfg['joint_names'],[.12475,0,0]);model=CollisionModel()
a=json.loads((bank/'evaluation.json').read_text());data=np.load(bank/'dataset.npz');rows=[]
for r in [x for x in a['rows'] if x['accepted']]:
 q,err,n=finish(fk,data['xyz'][r['index']],np.array(r['q']),np.array(cfg['lower'])+.025,np.array(cfg['upper'])-.025,home)
 hits=[];count=math.ceil(np.max(abs(q-home))/.025)+1
 if err<=1:
  for j,t in enumerate(np.linspace(0,1,count)):
   h=model.hits(home+t*(q-home))
   if h:hits.append(dict(sample=j,pairs=h))
 rows.append(dict(index=r['index'],q=q.tolist(),error_mm=err,nfev=n,endpoint_pass=err<=1,path_checked=err<=1,path_samples=count if err<=1 else 0,path_hits=hits))
result=dict(complete=True,count=len(rows),endpoint_pass_count=sum(x['endpoint_pass'] for x in rows),colliding_path_count=sum(bool(x['path_hits']) for x in rows),rows=rows,scope='Offline .025rad interior bounds, same trained model and target domain; sampled nominal home paths only. Not dynamics, corrected trajectory envelope, environment collision or physical certification.')
(out/'v138_interior_ik_coverage.json').write_text(json.dumps(result,indent=2)+'\n');print({k:v for k,v in result.items() if k!='rows'},flush=True)
