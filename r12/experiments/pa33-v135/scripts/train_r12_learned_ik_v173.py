"""All labels start from one home posture. Select network by held-out TCP mm."""
import json,time,hashlib
from pathlib import Path
import numpy as np
import torch
from arm_practice.learned_ik import ArmFK,CloudGate,network,finish,LearnedIK
from arm_practice.kinematics import forward_transform
root=Path(__file__).resolve().parents[1];out=root.parent.parent/'outputs/r12/learned_ik_v2_workspace'
out.mkdir(exist_ok=False);torch.set_num_threads(2);torch.manual_seed(173);rng=np.random.default_rng(173)
cfg=json.loads((root/'configs/r12_robot.json').read_text());lo=np.array([-np.pi]*4+[-.35]);hi=np.array([np.pi]*4+[.35]);home=np.array(cfg['home']);scale=(hi-lo)/2
(out/'experiment.json').write_text(json.dumps(dict(seed=173,lower=lo.tolist(),upper=hi.tolist(),home=home.tolist(),scope='SIMULATION GEOMETRY EXPLORATION ONLY: +/-pi search coordinates, not mechanical limits. Joint5 fixed at home. Active robot config unchanged. Position learning precedes orientation and trajectory validation.',label_start='identical home for every target',samples=12000),indent=2)+'\n')
urdf=root/'models/r12/r12_arm.urdf';offset=json.loads((root/'configs/r12_tcp_calibration.json').read_text())['translation_m'];fk=ArmFK(urdf,cfg['joint_names'],offset)
for q in rng.uniform(lo,hi,(12,5)):
 expected=(forward_transform(urdf,'arm_link5',dict(zip(cfg['joint_names'],q)))@np.r_[offset,1])[:3];assert np.allclose(fk(q),expected,atol=1e-10)
# Sample targets with FK, but never use sampled joint configurations as labels.
angles=rng.uniform(-np.pi,np.pi,12000);radii=rng.uniform(.15,.45,12000);heights=rng.uniform(.08,.40,12000)
xyz=np.column_stack([radii*np.cos(angles),radii*np.sin(angles),heights]);labels=[];label_errors=[]
for i,x in enumerate(xyz):
 q,e,n=finish(fk,x,home,lo,hi,home);labels.append(q);label_errors.append(e)
 if i%300==0:print('labels',i,flush=True)
labels=np.array(labels);good=np.array(label_errors)<=1.;ids=np.flatnonzero(good);rng.shuffle(ids)
ntrain=int(len(ids)*.75);nval=int(len(ids)*.125);train=ids[:ntrain];val=ids[ntrain:ntrain+nval];test=ids[ntrain+nval:];assert len(test)>100
np.savez(out/'dataset.npz',xyz=xyz,labels=labels,label_error_mm=label_errors,label_start_q=np.tile(home,(len(xyz),1)),train=train,validation=val,test=test)
gate=CloudGate(xyz[train]);val=np.array([i for i in val if gate.contains(xyz[i])]);test_inside=[i for i in test if gate.contains(xyz[i])];assert len(val)>20 and len(test_inside)>20
mean=xyz[train].mean(0);std=xyz[train].std(0);X=torch.tensor((xyz[train]-mean)/std,dtype=torch.float32);Y=torch.tensor(labels[train,:4]/scale[:4],dtype=torch.float32)
net=network();opt=torch.optim.Adam(net.parameters(),lr=.002);best=float('inf');best_epoch=0
for epoch in range(1601):
 opt.zero_grad();loss=((net(X)-Y)**2).mean();loss.backward();opt.step()
 if epoch%20==0:
  with torch.no_grad():pred=net(torch.tensor((xyz[val]-mean)/std,dtype=torch.float32)).numpy()*scale[:4]
  errors=[np.linalg.norm(fk(np.r_[np.clip(q,lo[:4],hi[:4]),home[4]])-xyz[i])*1000 for q,i in zip(pred,val)];metric=float(np.percentile(errors,95))
  if metric<best:
   best=metric;best_epoch=epoch;torch.save(dict(weights=net.state_dict(),train_xyz=xyz[train],mean=mean,std=std,scale=scale,lower=lo,upper=hi,home=home,tolerance_mm=1.,urdf_sha256=hashlib.sha256(urdf.read_bytes()).hexdigest()),out/'model.pt')
  print('epoch',epoch,'validation_proposal_p95_mm',metric,flush=True)
model=LearnedIK(out/'model.pt',fk);rows=[]
for i in test:
 start=time.perf_counter();r=model.solve(xyz[i]);r['elapsed_ms']=(time.perf_counter()-start)*1000;r['index']=int(i)
 if r.get('q') is not None:
  start=time.perf_counter();q,e,n=finish(fk,xyz[i],home,lo,hi,home);r.update(home_solver_error_mm=e,home_solver_nfev=n,home_solver_elapsed_ms=(time.perf_counter()-start)*1000)
 rows.append(r)
inside=[r for r in rows if 'q' in r]
def stats(key):
 a=np.array([r[key] for r in inside]);return dict(median=float(np.median(a)),p95=float(np.percentile(a,95)),max=float(a.max()))
assert not model.solve([10,10,10])['accepted'];assert not model.solve([float('nan'),0,0])['accepted']
report=dict(seed=173,total_labels=len(xyz),label_successes=int(good.sum()),train_count=len(train),validation_supported_count=len(val),test_total=len(test),test_supported_count=len(inside),test_rejected_count=len(test)-len(inside),test_endpoint_pass_count=sum(r['accepted'] for r in inside),tolerance_mm=1.,best_epoch=best_epoch,selection='validation proposal TCP p95 mm, not optimizer loss',metrics={k:stats(k) for k in ['proposal_error_mm','error_mm','home_solver_error_mm','solver_nfev','home_solver_nfev','elapsed_ms','home_solver_elapsed_ms']},fk_parity_cases=12,outside_and_nonfinite_rejection_passed=True,scope='Position-only CAD TCP. All label solves initialized at identical home; Cartesian targets sampled directly; labels solved only from home. Training-only Delaunay hull plus nearest25mm and simplex diameter80mm gates are conservative support heuristics, not reachability proof. Joint5 fixed at home. No orientation/path/collision/actuator or hardware acceptance. Runtime not switched; active limits preserved; experiment uses unrestricted-angle search coordinates, not physical ranges.',rows=rows)
(out/'evaluation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k!='rows'},indent=2))
