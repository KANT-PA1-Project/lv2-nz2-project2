"""Held-out recovery audit: NN proposal failure then same-home solver; no deployment."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from arm_practice.learned_ik import ArmFK,finish
root=Path(__file__).resolve().parents[1];out=root.parent.parent/'outputs/r12';src=out/'learned_ik_v2_workspace';cfg=json.loads((root/'configs/r12_robot.json').read_text());exp=json.loads((src/'experiment.json').read_text());lo=np.array(exp['lower']);hi=np.array(exp['upper']);home=np.array(exp['home']);fk=ArmFK(root/'models/r12/r12_arm.urdf',cfg['joint_names'],[.12475,0,0])
with np.load(src/'dataset.npz') as d:xyz=d['xyz'];labels=d['labels'];train=d['train']
reports={}
for name in ['learned_ik_v2_workspace','learned_ik_flow_v174']:
 report=json.loads((out/name/'evaluation.json').read_text());failures=[r for r in report['rows'] if r.get('q') is not None and not r['accepted']];rows=[]
 for r in failures:
  q,e,n=finish(fk,xyz[r['index']],home,lo,hi,home)
  rows.append(dict(index=r['index'],proposal_solver_error_mm=r['error_mm'],home_recovery_error_mm=e,recovered=e<=1.,solver_nfev=n))
 reports[name]={'supported':report['test_supported_count'],'unsupported_rejected':report['test_rejected_count'],'original_pass':report['test_endpoint_pass_count'],'recovery_pass':report['test_endpoint_pass_count']+sum(r['recovered'] for r in rows),'failures':rows}
# Audit label smoothness using training-only spatial neighbors; do not tune on held-out failures.
dist,ix=cKDTree(xyz[train]).query(xyz[train],k=2);dq=labels[train,:4]-labels[train[ix[:,1]],:4];wrapped=np.arctan2(np.sin(dq),np.cos(dq));near=dist[:,1]<.01
r={'complete':True,'models':reports,'training_neighbor_diagnostic':{'pairs_closer_than_10mm':int(near.sum()),'pairs_with_any_joint_change_above_1rad':int((near & (np.abs(wrapped).max(axis=1)>1)).sum()),'scope':'Nearest-neighbor pairs may be counted twice; discontinuity indicator, not proof of branch cause.'},'scope':'Position-only recovery test; unchanged 1mm threshold. Recovery does not improve neural predictions. Outside-support rejection remains mandatory. No collision/path/orientation or hardware validation.', 'execution_authorized':False}
(out/'v175_ik_recovery_audit.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
