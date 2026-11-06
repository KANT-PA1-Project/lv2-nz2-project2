"""PPO residual actuator training on actual R12 PhysX transitions."""
import argparse,json,time,sys
from pathlib import Path
import torch
from torch import nn
from torch.distributions import Normal
from isaaclab.app import add_launcher_args,launch_simulation
from swerve_env import Env,Config,ROOT
p=argparse.ArgumentParser();p.add_argument('--iterations',type=int,default=500);p.add_argument('--num-envs',type=int,default=8);p.add_argument('--seed',type=int,default=20261030);add_launcher_args(p);args=p.parse_args();torch.set_num_threads(4);torch.manual_seed(args.seed)
sys.path.insert(0,str(ROOT/'src'))
from arm_practice.swerve_rl import Policy
cfg=Config();cfg.scene.num_envs=args.num_envs;cfg.seed=args.seed
with launch_simulation(cfg.sim,args):
 env=Env(cfg);obs,_=env.reset();obs=obs['policy'];net=Policy().to(env.device);opt=torch.optim.Adam(net.parameters(),lr=3e-4);history=[];start=time.monotonic();horizon=64
 for it in range(args.iterations):
  oo=[];aa=[];lp=[];vv=[];rr=[];dd=[]
  for k in range(horizon):
   with torch.no_grad():dist,val=net(obs);act=dist.sample();logp=dist.log_prob(act).sum(-1)
   oo.append(obs.clone());aa.append(act);lp.append(logp);vv.append(val)
   ob,r,term,trunc,_=env.step(act);rr.append(r);dd.append((term|trunc).float());obs=ob['policy']
  with torch.no_grad():_,last=net(obs)
  gae=torch.zeros(args.num_envs,device=env.device);adv=torch.zeros_like(torch.stack(rr))
  for k in reversed(range(horizon)):
   delta=rr[k]+.99*(last if k==horizon-1 else vv[k+1])*(1-dd[k])-vv[k];gae=delta+.99*.95*(1-dd[k])*gae;adv[k]=gae
  ret=(adv+torch.stack(vv)).flatten();ad=adv.flatten();ad=(ad-ad.mean())/(ad.std()+1e-8);o=torch.stack(oo).flatten(0,1);a=torch.stack(aa).flatten(0,1);old=torch.stack(lp).flatten()
  for _ in range(4):
   for ix in torch.randperm(len(ad),device=env.device).split(256):
    dist,val=net(o[ix]);ratio=(dist.log_prob(a[ix]).sum(-1)-old[ix]).exp();loss=-torch.minimum(ratio*ad[ix],ratio.clamp(.8,1.2)*ad[ix]).mean()+.5*(val-ret[ix]).square().mean()-.003*dist.entropy().sum(-1).mean();opt.zero_grad();loss.backward();nn.utils.clip_grad_norm_(net.parameters(),1);opt.step()
  row={'iteration':it+1,'transitions':(it+1)*horizon*args.num_envs,'mean_reward':float(torch.stack(rr).mean()),'elapsed_s':time.monotonic()-start};history.append(row)
  if it%5==0 or it==args.iterations-1:
   print(json.dumps(row),flush=True);torch.save({'model':net.state_dict(),'iteration':it+1,'seed':args.seed,'method':'PPO PhysX scalar residual on encoder/gyro adaptive gain prior; 2 IMUs and 8 encoder observations; randomized actuation gain .75-1.25'},ROOT/'models/swerve/rl/policy.pt');(ROOT/'reports/local/rl/training.json').write_text(json.dumps(history,indent=2))
 # Same held-out commands and gain settings, learned vs zero residual.
 results=[];env.evaluation=True
 for mode in ['baseline','adaptive_prior','ppo']:
  torch.manual_seed(20261104)
  for trial in range(4):
   ob,_=env.reset();obs=ob['policy'];errors=[];rewards=[]
   for k in range(140):
    with torch.no_grad():act=net(obs)[0].mean if mode=='ppo' else (net.prior(obs) if mode=='adaptive_prior' else torch.zeros(args.num_envs,1,device=env.device))
    ob,r,_,_,_=env.step(act);obs=ob['policy']
    if k>=70:errors.append((env.actual_twist()-env.command).cpu());rewards.append(r.cpu())
   error=torch.stack(errors);results.append({'mode':mode,'trial':trial,'rmse':error.square().mean(dim=(0,1)).sqrt().tolist(),'reward':float(torch.stack(rewards).mean())})
 (ROOT/'reports/local/rl/evaluation.json').write_text(json.dumps({'heldout_seed':20261104,'num_envs':args.num_envs,'results':results},indent=2));print('PPO_COMPLETE',flush=True);env.close()
