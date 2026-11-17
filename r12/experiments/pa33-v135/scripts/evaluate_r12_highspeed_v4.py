"""Fixed-command held-out PhysX audit; no automatic deployment."""
import sys,json,math,argparse,time
from pathlib import Path
import torch
root=Path(__file__).resolve().parents[1];sys.path[:0]=[str(root/'rl'),str(root/'src')]
from isaaclab.app import add_launcher_args,launch_simulation
from swerve_highspeed_v4_env import Env,Config
from arm_practice.swerve_highspeed_v4_rl import Policy
p=argparse.ArgumentParser();p.add_argument('--checkpoint',type=Path,required=True);p.add_argument('--output',type=Path,required=True);add_launcher_args(p);args=p.parse_args();torch.set_num_threads(4)
cfg=Config();cfg.scene.num_envs=8;cfg.episode_length_s=20.;cfg.seed=20261109;cfg.sim.device=args.device or "cuda:0"
if args.output.exists():raise ValueError('Preserve prior evaluation; choose a new output file')
args.output.parent.mkdir(parents=True,exist_ok=True)
cfg.scene.robot.init_state.joint_pos={**{p+'_'+kind+'_joint':0. for p in ['lh1','rh1','lh2','rh2'] for kind in ['steer','wheel']},**{f'arm_joint{i}':.33 if i in [2,3,4] else 0. for i in range(1,6)},'gripper_left_joint':.02768613,'gripper_right_joint':-.02768613}
results=[]
with launch_simulation(cfg.sim,args):
 env=Env(cfg);env.evaluation=True;net=Policy().to(env.device);net.load_state_dict(torch.load(args.checkpoint,map_location=env.device,weights_only=True)['model']);net.eval()
 for speed in [.02,.05,.1,.3,.6,.8,.899795667]:
  for name,requested in [('forward',[speed,0,0]),('lateral',[0,speed,0]),('diagonal',[speed/math.sqrt(2),speed/math.sqrt(2),0]),('spin',[0,0,1.5]),('combined',[speed*.8,speed*.6,.8])]:
   c=torch.tensor(requested,device=env.device);x=torch.tensor([.1,.1,-.1,-.1],device=env.device);y=torch.tensor([.1,-.1,.1,-.1],device=env.device)
   peak=torch.sqrt((c[0]-c[2]*y)**2+(c[1]+c[2]*x)**2).max();c*=min(1,.899795667/float(peak))
   for mode in ['baseline','adaptive_prior','ppo']:
    torch.manual_seed(20261109);env.reset();env.command[:]=c;env.gain[:]=torch.linspace(.85,1.15,8,device=env.device)[:,None];obs=env._get_observations()['policy'];errors=[];tilts=[];failed=False;stops=[]
    for k in range(240):
     if k==200:env.command[:]=0
     with torch.no_grad():act=net(obs)[0].mean if mode=='ppo' else (net.prior(obs) if mode=='adaptive_prior' else torch.zeros(8,7,device=env.device))
     ob,r,term,trunc,_=env.step(act);obs=ob['policy'];failed=failed or bool(term.any()) or bool(trunc.any())
     quat=env.robot.data.root_pose_w.torch[:,3:];tilts.append(2*torch.asin(quat[:,:2].norm(dim=1).clamp(0,1))*180/math.pi)
     if 120<=k<200:errors.append(env.actual_twist()-c)
     if k>=230:stops.append(env.actual_twist())
    err=torch.stack(errors);rmse=err.square().mean(dim=0).sqrt();stop=torch.stack(stops).square().mean(dim=0).sqrt();tilt=float(torch.stack(tilts).max())
    passed=not failed and bool((rmse[:,:2]<=.03).all()) and bool((rmse[:,2]<=.1).all()) and bool((stop[:,:2]<=.02).all()) and bool((stop[:,2]<=.04).all()) and tilt<4
    row={'case':name,'speed_tier':speed,'mode':mode,'requested_twist':requested,'feasible_twist':c.tolist(),'rmse_per_env':rmse.tolist(),'stop_rmse_per_env':stop.tolist(),'max_tilt_deg':tilt,'terminated':failed,'passed':passed};results.append(row);print(json.dumps(row),flush=True)
    dest=args.output;temp=dest.with_suffix('.tmp');temp.write_text(json.dumps({'seed':20261109,'scope':'Flat floor, no payload; assumed source mass, efficiency .8, continuous torque .2 stall. Command feasibility scaling disclosed. Includes braking; not high-speed navigation or hardware validation.','results':results},indent=2));temp.replace(dest)
 env.close()
print('FIXED_EVALUATION_COMPLETE',flush=True)
