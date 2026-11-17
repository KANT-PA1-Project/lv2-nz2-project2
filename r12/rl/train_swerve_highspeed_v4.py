"""PPO residual actuator training on actual R12 PhysX transitions."""
import argparse,json,time,sys
from pathlib import Path
import torch
from torch import nn
from torch.distributions import Normal
from isaaclab.app import add_launcher_args,launch_simulation
from swerve_highspeed_v4_env import Env,Config,ROOT
p=argparse.ArgumentParser();p.add_argument('--resume',action='store_true');p.add_argument('--require-actuator-graphs',action='store_true');p.add_argument('--checkpoint',type=Path);p.add_argument('--experiment',default='rl_slew_control_v4');p.add_argument('--iterations',type=int,default=500);p.add_argument('--num-envs',type=int,default=8);p.add_argument('--seed',type=int,default=20261030);add_launcher_args(p);args=p.parse_args();torch.set_num_threads(4);torch.manual_seed(args.seed)
sys.path.insert(0,str(ROOT/'src'))
from arm_practice.swerve_highspeed_v4_rl import Policy
model_dir=ROOT/'models/swerve'/args.experiment;report_dir=ROOT/'reports/local'/args.experiment
if Path(args.experiment).name!=args.experiment:raise ValueError('Experiment must be a directory name')
model_dir.mkdir(parents=True,exist_ok=True);report_dir.mkdir(parents=True,exist_ok=True)
if (model_dir/'policy.pt').exists():raise ValueError('Use a new experiment name to preserve checkpoints')
cfg=Config();cfg.scene.robot.init_state.joint_pos={**{p+'_'+kind+'_joint':0. for p in ['lh1','rh1','lh2','rh2'] for kind in ['steer','wheel']},**{f'arm_joint{i}':.33 if i in [2,3,4] else 0. for i in range(1,6)},'gripper_left_joint':.02768613,'gripper_right_joint':-.02768613};cfg.scene.num_envs=args.num_envs;cfg.seed=args.seed;cfg.sim.device=args.device or "cuda:0"
with launch_simulation(cfg.sim,args):
 env=Env(cfg);obs,_=env.reset();obs=obs['policy'];net=Policy().to(env.device);opt=torch.optim.Adam(net.parameters(),lr=3e-4);history=[];start=time.monotonic();horizon=64
 runtime={"simulation_device":str(env.device),"policy_device":str(next(net.parameters()).device),"num_envs":args.num_envs,"cuda_name":torch.cuda.get_device_name() if str(env.device).startswith("cuda") else None};print(json.dumps({"runtime":runtime}),flush=True);(report_dir/"runtime.json").write_text(json.dumps(runtime,indent=2))
 if str(env.device)!=str(next(net.parameters()).device):raise RuntimeError("Simulation/policy device mismatch")
 if args.resume or args.checkpoint:

  checkpoint=torch.load(args.checkpoint or ROOT/'models/swerve/rl_highspeed_v2/policy.pt',map_location=env.device,weights_only=True);net.load_state_dict(checkpoint['model']);opt.load_state_dict(checkpoint['optimizer'])
 def verify_graph():
  runtime=env.robot._actuator_control._actuator_runtime
  count=len(runtime.native_actuator_graphs or ())
  if count!=2:raise RuntimeError(f"Expected two actuator CUDA graphs, got {count}")
  if runtime.adapter.is_stateful:raise RuntimeError("Graph/eager comparison requires stateless R12 DC motors")
  runtime.compute(env.robot.actuators,env.physics_dt)
  graph=env.robot._physx_actuator_wrapper.joint_f_2d
  import warp as wp
  expected=wp.to_torch(graph).clone()
  runtime._run_native_actuator_kernels(env.robot.actuators,env.physics_dt)
  actual=wp.to_torch(graph)
  torch.testing.assert_close(actual,expected,rtol=1e-5,atol=1e-6)
  return {"graph_count":count,"graph_eager_max_abs_error":float((actual-expected).abs().max())}
 if args.require_actuator_graphs:
  check=verify_graph();print(json.dumps({"graph_verification":check}),flush=True);(report_dir/"graph_verification.json").write_text(json.dumps(check,indent=2))
 for it in range(args.iterations):
  env.speed_cap=.899795667 if (args.resume or args.checkpoint) else .25+(.899795667-.25)*min(1,it/max(1,args.iterations*.3))
  oo=[];aa=[];lp=[];vv=[];rr=[];raw_rr=[];dd=[];sensor_flags=[];termination_flags=[];truncation_flags=[]
  for k in range(horizon):
   with torch.no_grad():dist,val=net(obs);act=dist.sample();logp=dist.log_prob(act).sum(-1)
   oo.append(obs.clone());aa.append(act);lp.append(logp);vv.append(val)
   ob,r,term,trunc,info=env.step(act);raw_rr.append(r.clone())
   if trunc.any():
    with torch.no_grad():_,terminal_value=net(info['final_obs']['policy'])
    r=r+.99*terminal_value*trunc.float()*(~term).float()
   rr.append(r);dd.append((term|trunc).float());obs=ob['policy'];sensor_flags.append(env.sensor_bad.float().clone());termination_flags.append(term.float());truncation_flags.append(trunc.float())
  with torch.no_grad():_,last=net(obs)
  gae=torch.zeros(args.num_envs,device=env.device);adv=torch.zeros_like(torch.stack(rr))
  for k in reversed(range(horizon)):
   delta=rr[k]+.99*(last if k==horizon-1 else vv[k+1])*(1-dd[k])-vv[k];gae=delta+.99*.95*(1-dd[k])*gae;adv[k]=gae
  ret=(adv+torch.stack(vv)).flatten();ad=adv.flatten();ad=(ad-ad.mean())/(ad.std()+1e-8);o=torch.stack(oo).flatten(0,1);a=torch.stack(aa).flatten(0,1);old=torch.stack(lp).flatten()
  for _ in range(4):
   for ix in torch.randperm(len(ad),device=env.device).split(256):
    dist,val=net(o[ix]);ratio=(dist.log_prob(a[ix]).sum(-1)-old[ix]).exp();loss=-torch.minimum(ratio*ad[ix],ratio.clamp(.8,1.2)*ad[ix]).mean()+.5*(val-ret[ix]).square().mean()-.003*dist.entropy().sum(-1).mean();opt.zero_grad();loss.backward();nn.utils.clip_grad_norm_(net.parameters(),1);opt.step()
  row={'iteration':it+1,'transitions':(it+1)*horizon*args.num_envs,'curriculum_speed_cap_m_s':env.speed_cap,'mean_reward':float(torch.stack(raw_rr).mean()),'mean_bootstrapped_reward':float(torch.stack(rr).mean()),'elapsed_s':time.monotonic()-start};row.update(sensor_bad_fraction=float(torch.stack(sensor_flags).mean()),termination_fraction=float(torch.stack(termination_flags).mean()),timeout_fraction=float(torch.stack(truncation_flags).mean()));row['reward_components_last_step']={k:float(v.mean()) for k,v in env.reward_components.items()};history.append(row)
  if it%5==0 or it==args.iterations-1:
   if args.require_actuator_graphs:row.update(verify_graph())
   if not torch.isfinite(torch.stack(rr)).all():raise RuntimeError('Nonfinite training rewards')
   print(json.dumps(row),flush=True);torch.save({'model':net.state_dict(),'optimizer':opt.state_dict(),'iteration':it+1,'seed':args.seed,'actuator_contract':'body_twist_steering_slew_v4','parent_checkpoint':str(args.checkpoint),'method':'PPO latent7 obs35: bounded body correction .04/.04/.15, steering .02rad, slew24.45rad/s2, wheelkd.08 with unchanged torque limits; fresh v4'},model_dir/'policy.tmp.pt');(model_dir/'policy.tmp.pt').replace(model_dir/'policy.pt');(report_dir/'training.json').write_text(json.dumps(history,indent=2))
 # Same held-out commands and gain settings, learned vs zero residual.
 results=[];env.evaluation=True;env.speed_cap=.899795667
 for mode in ['baseline','adaptive_prior','ppo']:
  torch.manual_seed(20261104)
  for trial in range(4):
   ob,_=env.reset();obs=ob['policy'];errors=[];rewards=[]
   for k in range(140):
    with torch.no_grad():act=net(obs)[0].mean if mode=='ppo' else (net.prior(obs) if mode=='adaptive_prior' else torch.zeros(args.num_envs,7,device=env.device))
    ob,r,_,_,_=env.step(act);obs=ob['policy']
    if k>=70:errors.append((env.actual_twist()-env.command).cpu());rewards.append(r.cpu())
   error=torch.stack(errors);results.append({'mode':mode,'trial':trial,'rmse':error.square().mean(dim=(0,1)).sqrt().tolist(),'reward':float(torch.stack(rewards).mean())})
 (report_dir/'evaluation.json').write_text(json.dumps({'heldout_seed':20261104,'num_envs':args.num_envs,'results':results},indent=2))
 (report_dir/'completion.json').write_text(json.dumps({'completed':True,'schema':'r12-body-steering-slew-v4','iteration':args.iterations,'runtime_compatible':False,'performance_accepted':False}))
 print('PPO_COMPLETE',flush=True);env.close()
