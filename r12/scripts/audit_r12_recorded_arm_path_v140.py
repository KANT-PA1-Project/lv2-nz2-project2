"""Audit saved encoder and corrected-reference samples; no control output."""
import json
from pathlib import Path
import numpy as np
from audit_r12_usd_arm_path import CollisionModel
root=Path(__file__).resolve().parents[1];m=CollisionModel();cfg=json.loads((root/'configs/r12_robot.json').read_text());e=json.loads((root/'reports/local/arm_interior_target_v137/evaluation.json').read_text());hits=[];count=0;outside=[]
for r in e['results']:
 for i,s in enumerate(r['samples']):
  for kind,q in [('measured_encoder',np.array(s['q'])),('corrected_reference',np.array(s['reference'])+np.array(s['bias']))]:
   count+=1
   if np.any(q<np.array(cfg['lower'])-1e-8) or np.any(q>np.array(cfg['upper'])+1e-8):outside.append(dict(sample=i,kind=kind))
   h=m.hits(q)
   if h:hits.append(dict(sample=i,kind=kind,pairs=h))
report=dict(complete=True,samples_checked=count,hits=hits,outside_limits=outside,scope='V137 recorded target-phase 20Hz encoder and reference+bias self-collision samples only; commanded reference reconstructed, home phase not recorded; not continuous swept volume, uncertainty envelope, environment or grasp validation.')
p=root.parent.parent/'outputs/r12/v140_recorded_arm_path.json';p.write_text(json.dumps(report,indent=2)+'\n');print(report)
