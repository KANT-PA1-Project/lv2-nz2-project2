"""R12 PhysX residual swerve RL. Actor observes mounted IMUs and all encoders."""
from pathlib import Path
import math,torch
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg,AssetBaseCfg
from isaaclab.actuators import ImplicitActuatorCfg,DCMotorCfg
from isaaclab.envs import DirectRLEnv,DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ImuCfg
from isaaclab_physx.physics import PhysxCfg
from isaaclab.utils import configclass
ROOT=Path(__file__).resolve().parents[1]
@configclass
class Scene(InteractiveSceneCfg):
 ground=AssetBaseCfg(prim_path='/World/Ground',spawn=sim_utils.GroundPlaneCfg())
 robot=ArticulationCfg(prim_path='{ENV_REGEX_NS}/Robot',spawn=sim_utils.UsdFileCfg(usd_path=str(ROOT/'models/r12/r12_mobile_rl.usda')),actuators={
 'steer':DCMotorCfg(joint_names_expr=['.*_steer_joint'],joint_effort_limit=.54,joint_velocity_limit=7.330382858,saturation_effort=2.7,stiffness=5.,damping=.2),
 'wheel':DCMotorCfg(joint_names_expr=['.*_wheel_joint'],joint_effort_limit=.108,joint_velocity_limit=29.321531434,saturation_effort=.54,stiffness=0.,damping=.08),
 'gripper':ImplicitActuatorCfg(joint_names_expr=['gripper_.*_joint'],joint_effort_limit=10.,stiffness=1000.,damping=20.),
 'arm':ImplicitActuatorCfg(joint_names_expr=['arm_joint.*'],joint_effort_limit=2.,stiffness=80.,damping=8.)})
 imu1=ImuCfg(prim_path='{ENV_REGEX_NS}/Robot/base_link',offset=ImuCfg.OffsetCfg(pos=(.057,0,.16597),rot=(0.,0.,0.,1.)))
 imu2=ImuCfg(prim_path='{ENV_REGEX_NS}/Robot/base_link',offset=ImuCfg.OffsetCfg(pos=(-.057,0,.16597),rot=(0.,0.,1.,0.)))
@configclass
class Config(DirectRLEnvCfg):
 decimation=6
 episode_length_s=8.
 compute_final_obs=True
 action_space=7
 observation_space=35
 state_space=0
 sim=sim_utils.SimulationCfg(dt=1/120,render_interval=6,device='cpu',physics=PhysxCfg())
 scene=Scene(num_envs=8,env_spacing=5.,replicate_physics=True,clone_in_fabric=False)
class Env(DirectRLEnv):
 def __init__(self,cfg,**kwargs):
  super().__init__(cfg,**kwargs);self.robot=self.scene['robot'];d=self.device;n=self.num_envs
  if self.robot.body_names[0]!="base_link":raise RuntimeError("RL root must be base_link; refusing invalid reset/reward frame")
  self.steer=torch.tensor([self.robot.joint_names.index(p+'_steer_joint') for p in ['lh1','rh1','lh2','rh2']],device=d)
  self.wheel=torch.tensor([self.robot.joint_names.index(p+'_wheel_joint') for p in ['lh1','rh1','lh2','rh2']],device=d)
  self.command=torch.zeros(n,3,device=d);self.actions=torch.zeros(n,7,device=d);self.previous_actions=torch.zeros_like(self.actions);self.steering=torch.zeros(n,4,device=d);self.wheels=torch.zeros_like(self.steering)
  self.gain=torch.ones(n,4,device=d);self.prevgyro=torch.zeros(n,3,device=d);self.lastgyro=torch.zeros(n,3,device=d);self.evaluation=False;self.speed_cap=.25;self.sensor_bad=torch.zeros(n,dtype=torch.bool,device=d)
 def _pre_physics_step(self,actions):
  previous_wheels=self.wheels.clone()
  self.prevgyro=self.lastgyro.clone()
  if not self.evaluation:
   switch=(self.episode_length_buf>0)&(self.episode_length_buf%40==0)
   if switch.any():self._sample_commands(switch.nonzero().flatten())
  self.previous_actions=self.actions.clone();self.actions=torch.tanh(actions)
  v=self.command.clone()+self.actions[:,:3]*self.actions.new_tensor([.04,.04,.15])
  moving=self.command.norm(dim=1)>1e-6
  x=torch.tensor([.1,.1,-.1,-.1],device=self.device);y=torch.tensor([.1,-.1,.1,-.1],device=self.device)
  u=v[:,0,None]-v[:,2,None]*y;w=v[:,1,None]+v[:,2,None]*x
  peak=torch.sqrt(u*u+w*w).amax(dim=1,keepdim=True);scale=(.899795667/peak.clamp(min=.899795667));u*=scale;w*=scale
  beta=torch.tensor([-math.pi/2,-math.pi/2,0,math.pi],device=self.device)
  raw=(torch.atan2(w,u)-beta+math.pi)%(2*math.pi)-math.pi
  angle=torch.where(raw>math.pi/2,raw-math.pi,torch.where(raw< -math.pi/2,raw+math.pi,raw))
  # End-stop branch hysteresis prevents numerical +/-90 degree flips.
  hold=((raw.abs()-math.pi/2).abs()<.06)&(self.steering.abs()>.7)
  angle=torch.where(hold,self.steering.sign()*math.pi/2,angle)
  angle=(angle+.02*self.actions[:,3:]).clamp(-math.pi/2,math.pi/2)
  direction=beta+angle;speed=(u*direction.cos()+w*direction.sin())/.03068719889482
  measured=self.robot.data.joint_pos.torch[:,self.steer];error=angle-measured
  self.steering+=torch.clamp(angle-self.steering,-7.330382858*self.step_dt,7.330382858*self.step_dt)
  self.steering=torch.where(moving[:,None],self.steering,measured)
  self.wheels=speed*error.cos().clamp(min=0)*self.gain
  ready=moving&~self.sensor_bad
  self.wheels*=ready[:,None];self.wheels.clamp_(-29.321531434,29.321531434)
  self.wheels=previous_wheels+(self.wheels-previous_wheels).clamp(-24.45*self.step_dt,24.45*self.step_dt)
  self.wheels[self.sensor_bad]=0
 def _apply_action(self):
  self.robot.actuators.target_command.set_position_index(value=self.robot.data.default_joint_pos.torch)
  self.robot.actuators.target_command.set_position_index(value=self.steering,joint_ids=self.steer)
  self.robot.actuators.target_command.set_velocity_index(value=self.wheels,joint_ids=self.wheel)
 def measured(self):
  a=self.scene['imu1'].data;b=self.scene['imu2'].data;flip=torch.tensor([-1.,-1.,1.],device=self.device)
  w1=a.ang_vel_b.torch;w2=b.ang_vel_b.torch*flip;f1=a.lin_acc_b.torch;f2=b.lin_acc_b.torch*flip
  gyro=(w1+w2)/2;alpha=(gyro-self.prevgyro)/self.step_dt;self.lastgyro=gyro.clone()
  r=torch.tensor([0.,0.,.16597],device=self.device).expand_as(gyro)
  accel=(f1+f2)/2-torch.cross(alpha,r,dim=-1)-torch.cross(gyro,torch.cross(gyro,r,dim=-1),dim=-1)
  self.sensor_bad=((w1-w2).norm(dim=1)>.35)|(~torch.isfinite(torch.cat([w1,w2,f1,f2],dim=1)).all(dim=1))
  return gyro,accel
 def _get_observations(self):
  gyro,accel=self.measured();q=self.robot.data.joint_pos.torch[:,self.steer];vel=self.robot.data.joint_vel.torch[:,self.wheel]
  # Motor-side encoder quantization, then external 4x conversion.
  vel=torch.round(vel/4*self.step_dt*4096/(2*math.pi))*2*math.pi/4096/self.step_dt*4
  q=torch.round(q*4096/(2*math.pi))*2*math.pi/4096
  heading=q+torch.tensor([-math.pi/2,-math.pi/2,0,math.pi],device=self.device)
  ux=.0306872*vel*heading.cos();uy=.0306872*vel*heading.sin()
  estimate=torch.stack([ux.mean(dim=1),uy.mean(dim=1),gyro[:,2]],dim=1)
  obs=torch.cat([(self.command-estimate)/torch.tensor([.9,.9,1.5],device=self.device),self.command/torch.tensor([.9,.9,1.5],device=self.device),q/(math.pi/2),vel/29.321531434,gyro/2,accel/10,self.actions,self.steering/(math.pi/2),self.wheels/29.321531434],dim=1)
  return {'policy':torch.nan_to_num(obs).clamp(-10,10)}
 def actual_twist(self):
  q=self.robot.data.root_pose_w.torch[:,3:];yaw=torch.atan2(2*(q[:,3]*q[:,2]+q[:,0]*q[:,1]),1-2*(q[:,1]**2+q[:,2]**2))
  v=self.robot.data.root_vel_w.torch
  return torch.stack([yaw.cos()*v[:,0]+yaw.sin()*v[:,1],-yaw.sin()*v[:,0]+yaw.cos()*v[:,1],v[:,5]],dim=1)
 def _get_rewards(self):
  delta=self.actual_twist()-self.command
  coarse=torch.exp(-.5*((delta/delta.new_tensor([.15,.15,.3]))**2).sum(dim=1))
  fine=torch.exp(-.5*((delta/delta.new_tensor([.03,.03,.1]))**2).sum(dim=1))
  quat=self.robot.data.root_pose_w.torch[:,3:];tilt=2*torch.asin(quat[:,:2].norm(dim=1).clamp(0,1))
  tilt_penalty=(tilt/(4*math.pi/180)).square()
  effort=.02*self.actions.square().mean(dim=1)
  smooth=.02*(self.actions-self.previous_actions).square().mean(dim=1)
  tracking=.3*coarse+.7*fine
  self.reward_components={'coarse':coarse.detach(),'fine':fine.detach(),'tilt_penalty':tilt_penalty.detach(),'action_penalty':effort.detach(),'smooth_penalty':smooth.detach()}
  return tracking-tilt_penalty-effort-smooth-self.sensor_bad.float()
 def _get_dones(self):
  q=self.robot.data.root_pose_w.torch[:,3:];bad=(q[:,:2].norm(dim=1)>.08716)|(~torch.isfinite(q).all(dim=1));timeout=self.episode_length_buf>=self.max_episode_length-1
  return bad,timeout
 def _reset_idx(self,ids):
  if ids is None:ids=self.robot._ALL_INDICES
  super()._reset_idx(ids);n=len(ids)
  pose=self.robot.data.default_root_pose.torch[ids].clone();pose[:,:3]+=self.scene.env_origins[ids];pose[:,2]+=.008
  self.robot.write_root_pose_to_sim_index(root_pose=pose,env_ids=ids);self.robot.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n,6,device=self.device),env_ids=ids)
  q=self.robot.data.default_joint_pos.torch[ids].clone();self.robot.write_joint_position_to_sim_index(position=q,env_ids=ids);self.robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q),env_ids=ids)
  self._sample_commands(ids)
  self.gain[ids]=.85+.3*torch.rand(n,4,device=self.device)
  self.actions[ids]=0;self.previous_actions[ids]=0;self.steering[ids]=q[:,self.steer];self.wheels[ids]=0;self.prevgyro[ids]=0;self.lastgyro[ids]=0;self.sensor_bad[ids]=False;self.robot.reset(ids)

 def _sample_commands(self,ids):
  n=len(ids)
  theta=torch.rand(n,device=self.device)*2*math.pi;speed=(.25+.75*torch.rand(n,device=self.device))*self.speed_cap
  self.command[ids]=torch.stack([speed*theta.cos(),speed*theta.sin(),(torch.rand(n,device=self.device)*2-1)*1.5],dim=1)
  self.command[ids[torch.arange(n,device=self.device)%4==0],2]=0
  if not self.evaluation:
   kind=torch.rand(n,device=self.device);low=kind<.4;stop=kind<.1
   self.command[ids[low],:2]*=(.005+.145*torch.rand(int(low.sum()),1,device=self.device))/speed[low,None].clamp(min=1e-6)
   self.command[ids[low],2]*=.2
   self.command[ids[stop]]=0
  # Preserve requested holonomic direction while respecting all wheel speed limits.
  c=self.command[ids];px=torch.tensor([.1,.1,-.1,-.1],device=self.device);py=torch.tensor([.1,-.1,.1,-.1],device=self.device)
  peak=torch.sqrt((c[:,0,None]-c[:,2,None]*py)**2+(c[:,1,None]+c[:,2,None]*px)**2).amax(dim=1)
  self.command[ids]*=(.899795667/peak.clamp(min=.899795667))[:,None]
