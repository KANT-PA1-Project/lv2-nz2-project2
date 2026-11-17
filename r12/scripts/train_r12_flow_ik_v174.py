"""R12 adaptation: conditional flow matching + differentiable FK; not paper reproduction."""
import json,time,hashlib,os
from pathlib import Path
import numpy as np
import torch
from arm_practice.learned_ik import ArmFK,CloudGate,finish
root=Path(__file__).resolve().parents[1];base=root.parent.parent/'outputs/r12';source=base/'learned_ik_v2_workspace';out=base/'learned_ik_flow_v174';out.mkdir(exist_ok=False)
spec=dict(method='Conditional flow matching with FK consistency and bounded solver finish',sources=['https://arxiv.org/abs/2609.29908','https://arxiv.org/abs/2606.15148'],adaptation='Small single-robot position-only MLP velocity field; not MorphIK morphology transformer or MimicIK temporal policy reproduction. Same-home labels, joint5 fixed; no trajectory or grasp claims.',runtime_published=False)
(out/'experiment.json').write_text(json.dumps(spec,indent=2)+'\n')
print('WAITING_FOR_V173_DATASET',flush=True)
deadline=time.monotonic()+1800
while not (source/'dataset.npz').exists():
 if time.monotonic()>deadline:raise RuntimeError('Dataset dependency timeout')
 time.sleep(5)
# Wait until the npz writer has closed a complete archive.
while True:
 try:
  with np.load(source/'dataset.npz') as d:data={k:d[k].copy() for k in d.files}
  break
 except (OSError,ValueError,EOFError):
  if time.monotonic()>deadline:raise
  time.sleep(2)
torch.set_num_threads(2);torch.manual_seed(174);rng=np.random.default_rng(174)
cfg=json.loads((root/'configs/r12_robot.json').read_text());exp=json.loads((source/'experiment.json').read_text());lo=np.array(exp['lower']);hi=np.array(exp['upper']);home=np.array(exp['home']);scale=(hi-lo)/2
fk=ArmFK(root/'models/r12/r12_arm.urdf',cfg['joint_names'],[.12475,0,0]);xyz=data['xyz'];labels=data['labels'];train=data['train'];gate=CloudGate(xyz[train]);val=[int(i) for i in data['validation'] if gate.contains(xyz[i])];test=[int(i) for i in data['test'] if gate.contains(xyz[i])];assert len(val)>20 and len(test)>20
mean=xyz[train].mean(0);std=xyz[train].std(0);X=torch.tensor((xyz[train]-mean)/std,dtype=torch.float32);Y=torch.tensor(labels[train,:4]/scale[:4],dtype=torch.float32);targets=torch.tensor(xyz[train],dtype=torch.float32);sc=torch.tensor(scale[:4],dtype=torch.float32)
net=torch.nn.Sequential(torch.nn.Linear(8,128),torch.nn.SiLU(),torch.nn.Linear(128,128),torch.nn.SiLU(),torch.nn.Linear(128,4));opt=torch.optim.AdamW(net.parameters(),lr=.001)
def tfk(q):
 yaw,a,b,c=q.unbind(-1);r=.024*torch.cos(a)-.1854*torch.sin(a)+.149*torch.cos(a+b)+.1911*torch.cos(a+b+c);z=.3345+.024*torch.sin(a)+.1854*torch.cos(a)+.149*torch.sin(a+b)+.1911*torch.sin(a+b+c)
 return torch.stack([r*torch.cos(yaw),r*torch.sin(yaw),z],-1)
qs=rng.uniform(lo[:4],hi[:4],(20,4));assert np.max(np.abs(tfk(torch.tensor(qs)).numpy()-np.array([fk(np.r_[q,0]) for q in qs])))<1e-9
# Fixed validation/test noise for reproducible candidate selection, never best-of-test resampling.
def propose(ids,seed):
 g=torch.Generator().manual_seed(seed);z=torch.randn((len(ids),4),generator=g);cond=torch.tensor((xyz[ids]-mean)/std,dtype=torch.float32)
 with torch.no_grad():
  for k in range(16):z=z+net(torch.cat([z,cond,torch.full((len(ids),1),k/16)],1))/16
 return np.clip(z.numpy()*scale[:4],lo[:4],hi[:4])
best=float('inf')
for step in range(3001):
 ix=torch.randint(len(train),(512,));y=Y[ix];noise=torch.randn_like(y);t=torch.rand((len(ix),1));zt=(1-t)*noise+t*y;v=net(torch.cat([zt,X[ix],t],1));pred=zt+(1-t)*v
 loss=((v-(y-noise))**2).mean()+.1*((tfk(pred*sc)-targets[ix])/.1).square().mean();opt.zero_grad();loss.backward();opt.step()
 if step%100==0:
  q=propose(val,174);errors=[np.linalg.norm(fk(np.r_[j,0])-xyz[i])*1000 for j,i in zip(q,val)];metric=float(np.percentile(errors,95));print('step',step,'validation_proposal_p95_mm',metric,flush=True)
  if metric<best:
   best=metric;torch.save(dict(weights=net.state_dict(),mean=mean,std=std,scale=scale,lower=lo,upper=hi,home=home,train_xyz=xyz[train],step=step,integration_steps=16,selection='validation proposal p95 mm',sources=spec['sources']),out/'model.pt')
bestmodel=torch.load(out/'model.pt',weights_only=False);net.load_state_dict(bestmodel['weights']);proposals=propose(test,175);rows=[]
for i,j in zip(test,proposals):
 seed=np.r_[j,0];q,error,n=finish(fk,xyz[i],seed,lo,hi,home);rows.append(dict(index=i,proposal_error_mm=float(np.linalg.norm(fk(seed)-xyz[i])*1000),error_mm=error,accepted=error<=1,solver_nfev=n,q=q.tolist()))
report=dict(complete=True,method=spec,selected_step=bestmodel['step'],train_count=len(train),validation_supported_count=len(val),test_total=len(data['test']),test_supported_count=len(test),test_rejected_count=len(data['test'])-len(test),test_endpoint_pass_count=sum(r['accepted'] for r in rows),tolerance_mm=1.,metrics={k:{'median':float(np.median([r[k] for r in rows])),'p95':float(np.percentile([r[k] for r in rows],95)),'max':float(max(r[k] for r in rows))} for k in ['proposal_error_mm','error_mm']},rows=rows,execution_authorized=False)
(out/'evaluation.json').write_text(json.dumps(report,indent=2)+'\n');print('EVALUATION_COMPLETE',json.dumps({k:v for k,v in report.items() if k!='rows'}),flush=True)
