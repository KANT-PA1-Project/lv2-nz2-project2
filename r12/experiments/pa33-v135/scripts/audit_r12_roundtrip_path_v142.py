"""Audit saved encoder and corrected-reference samples; no control output."""
import json
from pathlib import Path
import numpy as np
from audit_r12_usd_arm_path import CollisionModel
root=Path(__file__).resolve().parents[1];m=CollisionModel();cfg=json.loads((root/'configs/r12_robot.json').read_text());e=json.loads((root/'reports/local/arm_roundtrip_v141_recorded/evaluation.json').read_text());hits=[];count=0;outside=[]
segments=[]
for r in e['results']:
 segments.extend([('home_before_'+str(r['index']),r['home_samples']),('target_'+str(r['index']),r['samples'])])
segments.append(('final_home',e['final_home_samples']))
for segment, samples in segments:
 for i,s in enumerate(samples):
  for kind,q in [('measured_encoder',np.array(s['q'])),('corrected_reference',np.array(s['reference'])+np.array(s['bias']))]:
   count+=1
   if np.any(q<np.array(cfg['lower'])-1e-8) or np.any(q>np.array(cfg['upper'])+1e-8):outside.append(dict(segment=segment,sample=i,kind=kind))
   h=m.hits(q)
   if h:hits.append(dict(segment=segment,sample=i,kind=kind,pairs=h))
report=dict(complete=True,samples_checked=count,hits=hits,outside_limits=outside,scope='V141 recorded all home/target/final-home 20Hz encoder and reference+bias self-collision samples; commanded reference reconstructed; not continuous swept volume, uncertainty envelope, environment or grasp validation.')
p=root.parent.parent/'outputs/r12/v142_roundtrip_path_audit.json';p.write_text(json.dumps(report,indent=2)+'\n');print(report)
