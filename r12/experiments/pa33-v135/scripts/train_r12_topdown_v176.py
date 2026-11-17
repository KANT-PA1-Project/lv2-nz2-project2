"""Top-down, fixed elbow branch geometric IK study; no runtime deployment."""
import json,time
from pathlib import Path
import numpy as np
import torch
from scipy.optimize import least_squares
from scipy.spatial import Delaunay,cKDTree
from arm_practice.learned_ik import ArmFK
root=Path(__file__).resolve().parents[1];out=root.parent.parent/'outputs/r12/learned_ik_topdown_v176';out.mkdir(exist_ok=False)
rng=np.random.default_rng(176);torch.manual_seed(176);torch.set_num_threads(2)
phi=float(np.arctan2(.1854,.024));lo=np.array([-np.pi,-np.pi,-np.pi]);hi=np.array([np.pi,phi,np.pi]);home=np.zeros(3)
spec={'seed':176,'labels':3000,'label_start_q':[0]*5,'pitch_lower':lo.tolist(),'pitch_upper':hi.tolist(),'elbow_branch':'q3 <= atan2(185.4,24), chosen geometric branch, not mechanical limit','scope':'Top-down approach only, joint5=0, yaw obtained analytically atan2(y,x). Learn pitch from radius/height; radial symmetry removes yaw wrap discontinuity. No collision, trajectory, load, hardware or pick/place certification. Active limits unchanged.','sources':['https://arxiv.org/abs/2609.29908','https://arxiv.org/abs/2606.15148']}
(out/'experiment.json').write_text(json.dumps(spec,indent=2)+'\n')
def rz(q):
 a,b,c=np.moveaxis(np.asarray(q),-1,0);return np.stack([.024*np.cos(a)-.1854*np.sin(a)+.149*np.cos(a+b)+.1911*np.cos(a+b+c),.3345+.024*np.sin(a)+.1854*np.cos(a)+.149*np.sin(a+b)+.1911*np.sin(a+b+c)],-1)
def solve(target,seed):
 fit=least_squares(lambda q:np.r_[rz(q)-target,.1*(sum(q)+np.pi/2)],np.clip(seed,lo+1e-9,hi-1e-9),bounds=(lo,hi),max_nfev=150,ftol=1e-11,xtol=1e-11,gtol=1e-11)
 q=fit.x;return q,float(np.linalg.norm(rz(q)-target)*1000),float(abs(sum(q)+np.pi/2)*180/np.pi)
xyz=rng.uniform([.15,.08],[.31,.22],(3000,2));labels=[];errs=[];angles=[]
for i,x in enumerate(xyz):
 q,e,a=solve(x,home);labels.append(q);errs.append(e);angles.append(a)
 if i%300==0:print('labels',i,flush=True)
labels=np.array(labels);good=(np.array(errs)<=1)&(np.array(angles)<=1);ids=np.flatnonzero(good);rng.shuffle(ids);n=int(len(ids)*.75);m=int(len(ids)*.125);train=ids[:n];val=ids[n:n+m];test=ids[n+m:]
np.savez(out/'dataset.npz',rz=xyz,labels=labels,label_error_mm=errs,label_angle_deg=angles,label_start_q=np.zeros((3000,5)),train=train,validation=val,test=test)
hull=Delaunay(xyz[train]);tree=cKDTree(xyz[train])
def supported(x):
 i=int(hull.find_simplex(x))
 if i<0 or tree.query(x)[0]>.025:return False
 p=xyz[train][hull.simplices[i]];return np.linalg.norm(p[:,None]-p[None,:],axis=-1).max()<=.08
val=np.array([i for i in val if supported(xyz[i])]);inside=np.array([i for i in test if supported(xyz[i])]);assert len(val)>50 and len(inside)>50
cfg=json.loads((root/'configs/r12_robot.json').read_text());fk=ArmFK(root/'models/r12/r12_arm.urdf',cfg['joint_names'],[.12475,0,0])
for i in ids[:20]:assert np.allclose(fk(np.r_[0,labels[i],0])[[0,2]],rz(labels[i]),atol=1e-9)
mean=xyz[train].mean(0);std=xyz[train].std(0);X=torch.tensor((xyz[train]-mean)/std,dtype=torch.float32);Y=torch.tensor(labels[train]/np.pi,dtype=torch.float32);targets=torch.tensor(xyz[train],dtype=torch.float32)
def tfk(q):
 a,b,c=q.unbind(-1);return torch.stack([.024*torch.cos(a)-.1854*torch.sin(a)+.149*torch.cos(a+b)+.1911*torch.cos(a+b+c),.3345+.024*torch.sin(a)+.1854*torch.cos(a)+.149*torch.sin(a+b)+.1911*torch.sin(a+b+c)],-1)
summary={}
for method in ['mlp','flow']:
 net=torch.nn.Sequential(torch.nn.Linear(2 if method=='mlp' else 6,128),torch.nn.SiLU(),torch.nn.Linear(128,128),torch.nn.SiLU(),torch.nn.Linear(128,3));opt=torch.optim.AdamW(net.parameters(),lr=.001);best=float('inf')
 def propose(ids,seed):
  cond=torch.tensor((xyz[ids]-mean)/std,dtype=torch.float32)
  with torch.no_grad():
   if method=='mlp':q=net(cond)
   else:
    q=torch.randn((len(ids),3),generator=torch.Generator().manual_seed(seed))
    for k in range(16):q=q+net(torch.cat([q,cond,torch.full((len(ids),1),k/16)],1))/16
  return np.clip(q.numpy()*np.pi,lo,hi)
 for step in range(3001):
  ix=torch.randint(len(train),(256,))
  if method=='mlp':pred=net(X[ix]);loss=(pred-Y[ix]).square().mean()
  else:
   y=Y[ix];noise=torch.randn_like(y);t=torch.rand((len(ix),1));zt=(1-t)*noise+t*y;v=net(torch.cat([zt,X[ix],t],1));pred=zt+(1-t)*v;loss=(v-(y-noise)).square().mean()
  q=pred*np.pi;loss=loss+.1*((tfk(q)-targets[ix])/.1).square().mean()+.01*(q.sum(-1)+np.pi/2).square().mean();opt.zero_grad();loss.backward();opt.step()
  if step%100==0:
   q=propose(val,176);mm=np.linalg.norm(rz(q)-xyz[val],axis=1)*1000;deg=np.abs(q.sum(-1)+np.pi/2)*180/np.pi;metric=float(np.percentile(mm,95));angle=float(np.percentile(deg,95));print(method,step,'validation_p95_mm',metric,'angle_p95_deg',angle,flush=True)
   # Physical endpoint validation metric, never optimizer loss. Orientation is also reported.
   if metric<best:
    best=metric;torch.save(dict(weights=net.state_dict(),mean=mean,std=std,lower=lo,upper=hi,train_rz=xyz[train],step=step,method=method,spec=spec),out/(method+'.pt'))
 model=torch.load(out/(method+'.pt'),weights_only=False);net.load_state_dict(model['weights']);q=propose(inside,177);rows=[]
 for i,seed in zip(inside,q):
  final,e,a=solve(xyz[i],seed);rows.append(dict(index=int(i),proposal_error_mm=float(np.linalg.norm(rz(seed)-xyz[i])*1000),proposal_angle_deg=float(abs(sum(seed)+np.pi/2)*180/np.pi),error_mm=e,angle_deg=a,accepted=e<=1 and a<=1,q=np.r_[0,final,0].tolist()))
 def stats(k):v=np.array([r[k] for r in rows]);return dict(median=float(np.median(v)),p95=float(np.percentile(v,95)),max=float(v.max()))
 summary[method]=dict(selected_step=model['step'],supported=len(inside),unsupported=len(test)-len(inside),pass_count=sum(r['accepted'] for r in rows),metrics={k:stats(k) for k in ['proposal_error_mm','proposal_angle_deg','error_mm','angle_deg']},rows=rows)
 (out/(method+'_evaluation.json')).write_text(json.dumps(summary[method],indent=2)+'\n')
 print('COMPLETE',method,summary[method]['pass_count'],len(inside),flush=True)
report=dict(complete=True,label_successes=int(good.sum()),train_count=len(train),test_total=len(test),tolerance_mm=1,orientation_tolerance_deg=1,models={k:{a:b for a,b in v.items() if a!='rows'} for k,v in summary.items()},execution_authorized=False,scope=spec['scope'])
(out/'evaluation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report),flush=True)
