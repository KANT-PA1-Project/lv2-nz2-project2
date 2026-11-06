"""Fixed-command held-out PhysX audit; no automatic deployment."""
import sys,json,math,argparse,time
from pathlib import Path
import torch
root=Path(__file__).resolve().parents[1];sys.path[:0]=[str(root/'rl'),str(root/'src')]
from isaaclab.app import add_launcher_args,launch_simulation
from swerve_highspeed_v2_env import Env,Config
from arm_practice.swerve_highspeed_v2_rl import Policy
p=argparse.ArgumentParser();p.add_argument('--checkpoint',type=Path,required=True);p.add_argument('--output',type=Path,required=True);add_launcher_args(p);args=p.parse_args();torch.set_num_threads(4)
cfg=Config();cfg.scene.num_envs=8;cfg.episode_length_s=20.;cfg.seed=20261109;cfg.sim.device=args.device or "cuda:0"
if args.output.exists():raise ValueError('Preserve prior evaluation; choose a new output file')
args.output.parent.mkdir(parents=True,exist_ok=True)
cfg.scene.robot.init_state.joint_pos={**{p+'_'+kind+'_joint':0. for p in ['lh1','rh1','lh2','rh2'] for kind in ['steer','wheel']},**{f'arm_joint{i}':.33 if i in [2,3,4] else 0. for i in range(1,6)},'gripper_left_joint':.02768613,'gripper_right_joint':-.02768613}

class ProbeEnv(Env):
 def _get_dones(self):
  result=super()._get_dones()
  self.before_reset={"quat":self.robot.data.root_pose_w.torch[:,3:].cpu().tolist(),"velocity":self.robot.data.root_vel_w.torch.cpu().tolist(),"sensor_bad":self.sensor_bad.cpu().tolist(),"episode_steps":self.episode_length_buf.cpu().tolist()}
  return result
with launch_simulation(cfg.sim,args):
 env=ProbeEnv(cfg);env.evaluation=True;obs,_=env.reset();rows=[]
 print(json.dumps({"device":str(env.device),"max_episode_length":env.max_episode_length}),flush=True)
 print(json.dumps({"body_names":env.robot.body_names,"initial_body_poses":env.robot.data.body_pose_w.torch[0].cpu().tolist(),"default_root":env.robot.data.default_root_pose.torch[0].cpu().tolist()}),flush=True)
 for k in range(80):
  env.command[:]=torch.tensor([.05,0,0],device=env.device)
  obs,r,term,trunc,_=env.step(torch.zeros(8,4,device=env.device))
  if k<5 or k%10==0:
   row={"step":k,"term":term.cpu().tolist(),"trunc":trunc.cpu().tolist(),"reward":r.cpu().tolist(),"before_reset":env.before_reset,"imu1":env.scene['imu1'].data.ang_vel_b.torch.cpu().tolist(),"imu2":env.scene['imu2'].data.ang_vel_b.torch.cpu().tolist()};rows.append(row);print(json.dumps(row),flush=True)
 args.output.write_text(json.dumps(rows,indent=2));env.close()
