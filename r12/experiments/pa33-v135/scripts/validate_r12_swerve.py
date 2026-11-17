#!/usr/bin/env python3
"""Open-floor dynamic tests of learned swerve actuation, no root velocity override."""
import sys,json,numpy as np,torch,math
from pathlib import Path
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'src'))
from arm_practice.swerve import SwervePolicy,actuate
net=SwervePolicy();net.load_state_dict(torch.load(root/'models/swerve/policy.pt',weights_only=True));net.eval();torch.set_num_threads(2)
from isaacsim import SimulationApp
app=SimulationApp({'headless':True})
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
world=World(stage_units_in_meters=1.,physics_dt=1/120)
add_reference_to_stage(str(root/'assets/vendor/R12_NO_USB_URDF_USD/usd/r12_no_usb.usdc'),'/World/R12')
world.scene.add_default_ground_plane()
robot=world.scene.add(SingleArticulation('/World/R12',name='robot'))
world.reset();nominal_position,nominal_orientation=robot.get_world_pose();nominal_position=nominal_position.copy();nominal_position[2]+=.008;print("NOMINAL_ROOT",nominal_position,flush=True);names=robot.dof_names
steer=[names.index(p+'_steer_joint') for p in ['lh1','rh1','lh2','rh2']];wheel=[names.index(p+'_wheel_joint') for p in ['lh1','rh1','lh2','rh2']];arm=[names.index(f'arm_joint{i}') for i in range(1,6)]
kp=np.zeros((1,len(names)));kd=np.full_like(kp,2.);kp[0,steer]=40;kd[0,steer]=4;kp[0,arm]=40;kd[0,arm]=4
robot._articulation_view.set_gains(kps=kp,kds=kd);effort=np.ones_like(kp);effort[0,arm]=2;robot._articulation_view.set_max_efforts(effort)
cases=[('forward',[.12,0,0]),('backward',[-.12,0,0]),('lateral',[0,.12,0]),('diagonal',[.085,.085,0]),('spin',[0,0,.3]),('combined',[.08,.04,.2])];results=[]
for name,command in cases:
 robot.set_world_pose(nominal_position,nominal_orientation);robot.set_linear_velocity(np.zeros(3));robot.set_angular_velocity(np.zeros(3));robot.set_joint_positions(np.zeros(len(names)));robot.set_joint_velocities(np.zeros(len(names)))
 q=np.zeros(len(names));vel=np.zeros(len(names));rows=[]
 for step in range(120*9):
  cmd=np.array(command) if 120<=step<120*7 else np.zeros(3)
  if step%6==0:
   a,w=actuate(net,cmd,robot.get_joint_positions()[steer],.05,q[steer]);q[steer]=a;vel[wheel]=w
   robot.apply_action(ArticulationAction(joint_positions=q,joint_velocities=vel))
  world.step(render=False)
  if step%6==0:
   pos,quat=robot.get_world_pose();w,x,y,z=quat;yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z));v=robot.get_linear_velocity();omega=robot.get_angular_velocity();R=np.array([[np.cos(yaw),np.sin(yaw)],[-np.sin(yaw),np.cos(yaw)]])
   measured=np.r_[R@v[:2],omega[2]]
   rows.append({'time':step/120,'position':pos.tolist(),'measured_twist':measured.tolist(),'q_steer':robot.get_joint_positions()[steer].tolist(),'wheel_speeds':robot.get_joint_velocities()[wheel].tolist()})
 measured=np.array([r['measured_twist'] for r in rows if 4<=r['time']<7]);rms=np.sqrt(np.mean((measured-command)**2,axis=0));final=np.array(rows[-1]['measured_twist'])
 result={'case':name,'command':command,'steady_mean':measured.mean(axis=0).tolist(),'steady_rmse':rms.tolist(),'stop_twist':final.tolist(),'passed':bool(max(rms[:2])<.03 and rms[2]<.08 and np.linalg.norm(final[:2])<.02 and abs(final[2])<.04),'trace':rows}
 results.append(result);print('SWERVE',json.dumps({k:v for k,v in result.items() if k!='trace'}),flush=True)
 (root/'reports/local/swerve/physics.json').write_text(json.dumps({'scope':'Open-floor nominal wheel-actuated PhysX. No sensors, SLAM, navigation or payload validation. Source mass/friction nominal.','results':results},indent=2))
world.stop();app.close()
