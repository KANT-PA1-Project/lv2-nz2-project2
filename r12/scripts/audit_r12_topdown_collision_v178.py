"""Reject geometric IK endpoints/paths that intersect configured USD colliders."""
import json
from pathlib import Path
import numpy as np
from audit_r12_usd_arm_path import CollisionModel
root=Path(__file__).resolve().parents[1];out=root.parent.parent/'outputs/r12';bank=out/'learned_ik_topdown_v176';model=CollisionModel();records=[]
for method in ['mlp','flow']:
 rows=json.loads((bank/(method+'_evaluation.json')).read_text())['rows'];hits=[]
 for row in rows:
  pairs=model.hits(row['q'])
  if pairs:hits.append(dict(index=row['index'],pairs=pairs))
 records.append(dict(method=method,tested=len(rows),colliding_endpoints=len(hits),hits=hits))
# Labels from home also need collision filtering; common start does not prove safe motion.
with np.load(bank/'dataset.npz') as data:labels=data['labels'];ids=data['train']
label_hits=[]
for i in ids:
 pairs=model.hits(np.r_[0,labels[i],0])
 if pairs:label_hits.append(dict(index=int(i),pairs=pairs))
r=dict(complete=True,home_hits=model.hits(np.zeros(5)),models=records,training_labels_tested=len(ids),training_label_collisions=len(label_hits),training_label_hits=label_hits,scope='Configured USD cube collider self-intersection only, excludes adjacent disabled pairs. No visual mesh clearance, environment or continuous path certification.',execution_authorized=False)
(out/'v178_topdown_collision_audit.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({k:v for k,v in r.items() if k not in ['training_label_hits','models']}));print([(m['method'],m['colliding_endpoints'],m['tested'],m['hits'][:2]) for m in records])
