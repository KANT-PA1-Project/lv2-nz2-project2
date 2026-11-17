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
 robot=ArticulationCfg(prim_path='{ENV_REGEX_NS}/Robot',spawn=sim_utils.UsdFileCfg(usd_path=str(ROOT/'assets/vendor/R12_NO_USB_URDF_USD/usd/r12_no_usb.usdc')),actuators={
 'steer':DCMotorCfg(joint_names_expr=['.*_steer_joint'],joint_effort_limit=.54,joint_velocity_limit=7.330382858,saturation_effort=2.7,stiffness=5.,damping=.2),
 'wheel':DCMotorCfg(joint_names_expr=['.*_wheel_joint'],joint_effort_limit=.108,joint_velocity_limit=29.321531434,saturation_effort=.54,stiffness=0.,damping=.02),
 'arm':ImplicitActuatorCfg(joint_names_expr=['arm_joint.*'],joint_effort_limit=2.,stiffness=40.,damping=4.)})
 imu1=ImuCfg(prim_path='{ENV_REGEX_NS}/Robot/base_link',offset=ImuCfg.OffsetCfg(pos=(.057,0,.16597),rot=(0.,0.,0.,1.)))
 imu2=ImuCfg(prim_path='{ENV_REGEX_NS}/Robot/base_link',offset=ImuCfg.OffsetCfg(pos=(-.057,0,.16597),rot=(0.,0.,1.,0.)))
@configclass
class Config(DirectRLEnvCfg):
 decimation=6
 episode_length_s=8.
 action_space=1
 observation_space=21
 state_space=0
 sim=sim_utils.SimulationCfg(dt=1/120,render_interval=6,device='cpu',physics=PhysxCfg())
 scene=Scene(num_envs=8,env_spacing=5.,replicate_physics=True,clone_in_fabric=False)
class Env(DirectRLEnv):
 def __init__(self,cfg,**kwargs):
  super().__init__(cfg,**kwargs);self.robot=self.scene['robot'];d=self.device;n=self.num_envs
  self.steer=torch.tensor([self.robot.joint_names.index(p+'_steer_joint') for p in ['lh1','rh1','lh2','rh2']],device=d)
  self.wheel=torch.tensor([self.robot.joint_names.index(p+'_wheel_joint') for p in ['lh1','rh1','lh2','rh2']],device=d)
  self.command=torch.zeros(n,3,device=d);self.actions=torch.zeros(n,1,device=d);self.previous_actions=torch.zeros_like(self.actions);self.steering=torch.zeros(n,4,device=d);self.wheels=torch.zeros_like(self.steering)
  self.gain=torch.ones(n,1,device=d);self.prevgyro=torch.zeros(n,3,device=d);self.evaluation=False;self.speed_cap=.25;self.sensor_bad=torch.zeros(n,dtype=torch.bool,device=d)
 def _pre_physics_step(self,actions):
  self.previous_actions=self.actions.clone();self.actions=actions.clamp(-1,1)
  v=self.command.clone()
  v[:,:2]*=(.899795667/v[:,:2].norm(dim=1,keepdim=True).clamp(min=.899795667));v[:,2].clamp_(-1.5,1.5)
  moving=self.command.norm(dim=1)>.001
  x=torch.tensor([.1,.1,-.1,-.1],device=self.device);y=torch.tensor([.1,-.1,.1,-.1],device=self.device)
  u=v[:,0,None]-v[:,2,None]*y;w=v[:,1,None]+v[:,2,None]*x
  beta=torch.tensor([-math.pi/2,-math.pi/2,0,math.pi],device=self.device)
  raw=(torch.atan2(w,u)-beta+math.pi)%(2*math.pi)-math.pi
  angle=torch.where(raw>math.pi/2,raw-math.pi,torch.where(raw< -math.pi/2,raw+math.pi,raw))
  # End-stop branch hysteresis prevents numerical +/-90 degree flips.
  hold=((raw.abs()-math.pi/2).abs()<.06)&(self.steering.abs()>.7)
  angle=torch.where(hold,self.steering.sign()*math.pi/2,angle)
  direction=beta+angle;speed=(u*direction.cos()+w*direction.sin())/.0306872
  measured=self.robot.data.joint_pos.torch[:,self.steer];error=angle-measured
  self.steering+=torch.clamp(angle-self.steering,-7.330382858*self.step_dt,7.330382858*self.step_dt)
  self.steering=torch.where(moving[:,None],self.steering,measured)
  self.wheels=speed*error.cos().clamp(min=0)*self.gain*(1+.3*self.actions)
  ready=(error.abs().amax(dim=1)<.25)&moving&~self.sensor_bad
  self.wheels*=ready[:,None];self.wheels.clamp_(-29.321531434,29.321531434)
 def _apply_action(self):
  self.robot.actuators.target_command.set_position_index(value=self.steering,joint_ids=self.steer)
  self.robot.actuators.target_command.set_velocity_index(value=self.wheels,joint_ids=self.wheel)
 def measured(self):
  a=self.scene['imu1'].data;b=self.scene['imu2'].data;flip=torch.tensor([-1.,-1.,1.],device=self.device)
  w1=a.ang_vel_b.torch;w2=b.ang_vel_b.torch*flip;f1=a.lin_acc_b.torch;f2=b.lin_acc_b.torch*flip
  gyro=(w1+w2)/2;alpha=(gyro-self.prevgyro)/self.step_dt;self.prevgyro=gyro.clone()
  r=torch.tensor([0.,0.,.16597],device=self.device).expand_as(gyro)
  accel=(f1+f2)/2-torch.cross(alpha,r,dim=-1)-torch.cross(gyro,torch.cross(gyro,r,dim=-1),dim=-1)
  self.sensor_bad=((w1-w2).norm(dim=1)>.35)|(~torch.isfinite(torch.cat([w1,w2,f1,f2],dim=1)).all(dim=1))
  return gyro,accel
 def _get_observations(self):
  gyro,accel=self.measured();q=self.robot.data.joint_pos.torch[:,self.steer];vel=self.robot.data.joint_vel.torch[:,self.wheel]
  q=torch.round(q*4096/(2*math.pi))*2*math.pi/4096
  heading=q+torch.tensor([-math.pi/2,-math.pi/2,0,math.pi],device=self.device)
  ux=.0306872*vel*heading.cos();uy=.0306872*vel*heading.sin()
  estimate=torch.stack([ux.mean(dim=1),uy.mean(dim=1),gyro[:,2]],dim=1)
  obs=torch.cat([(self.command-estimate)/torch.tensor([.9,.9,1.5],device=self.device),self.command/torch.tensor([.9,.9,1.5],device=self.device),q/(math.pi/2),vel/29.321531434,gyro/2,accel/10,self.actions],dim=1)
  return {'policy':torch.nan_to_num(obs).clamp(-10,10)}
 def actual_twist(self):
  q=self.robot.data.root_pose_w.torch[:,3:];yaw=torch.atan2(2*(q[:,3]*q[:,2]+q[:,0]*q[:,1]),1-2*(q[:,1]**2+q[:,2]**2))
  v=self.robot.data.root_vel_w.torch
  return torch.stack([yaw.cos()*v[:,0]+yaw.sin()*v[:,1],-yaw.sin()*v[:,0]+yaw.cos()*v[:,1],v[:,5]],dim=1)
 def _get_rewards(self):
  error=(self.actual_twist()-self.command)/torch.tensor([.15,.15,.3],device=self.device)
  tracking=torch.exp(-.5*(error**2).sum(dim=1))
  quat=self.robot.data.root_pose_w.torch[:,3:];tilt=quat[:,:2].square().sum(dim=1)
  return tracking-10*tilt-.001*self.actions.square().sum(dim=1)-.001*(self.actions-self.previous_actions).square().sum(dim=1)-self.sensor_bad.float()
 def _get_dones(self):
  q=self.robot.data.root_pose_w.torch[:,3:];bad=(q[:,:2].norm(dim=1)>.08716)|(~torch.isfinite(q).all(dim=1));timeout=self.episode_length_buf>=self.max_episode_length-1
  return bad,timeout
 def _reset_idx(self,ids):
  if ids is None:ids=self.robot._ALL_INDICES
  super()._reset_idx(ids);n=len(ids)
  pose=self.robot.data.default_root_pose.torch[ids].clone();pose[:,:3]+=self.scene.env_origins[ids];pose[:,2]+=.008
  self.robot.write_root_pose_to_sim_index(root_pose=pose,env_ids=ids);self.robot.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n,6,device=self.device),env_ids=ids)
  q=self.robot.data.default_joint_pos.torch[ids].clone();self.robot.write_joint_position_to_sim_index(position=q,env_ids=ids);self.robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q),env_ids=ids)
  theta=torch.rand(n,device=self.device)*2*math.pi;speed=(.25+.75*torch.rand(n,device=self.device))*self.speed_cap
  self.command[ids]=torch.stack([speed*theta.cos(),speed*theta.sin(),(torch.rand(n,device=self.device)*2-1)*1.5],dim=1)
  self.command[ids[torch.arange(n,device=self.device)%4==0],2]=0
  self.gain[ids]=.75+.5*torch.rand(n,1,device=self.device)
  self.actions[ids]=0;self.previous_actions[ids]=0;self.steering[ids]=q[:,self.steer];self.wheels[ids]=0;self.prevgyro[ids]=0;self.sensor_bad[ids]=False;self.robot.reset(ids)
