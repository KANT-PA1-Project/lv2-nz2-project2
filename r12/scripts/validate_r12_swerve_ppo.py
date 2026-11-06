#!/usr/bin/env python3
"""Open-floor dynamic tests of learned swerve actuation, no root velocity override."""
import sys,json,numpy as np,torch,math
import argparse
p=argparse.ArgumentParser();p.add_argument("--gain",type=float,default=1.);p.add_argument("--reversal",action="store_true");p.add_argument("--envelope",action="store_true");p.add_argument("--speed",type=float,default=.12);args=p.parse_args()
from pathlib import Path
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'src'))
from arm_practice.swerve import SwervePolicy,actuate
from arm_practice.swerve_rl import ResidualController
from arm_practice.motion_envelope import MotionEnvelope,WheelSlew
sys.path.insert(0,str(root/'scripts/sensor_runtime'))
from dual_imu import DualImuFusion
net=SwervePolicy();net.load_state_dict(torch.load(root/'models/swerve/policy.pt',weights_only=True));net.eval();torch.set_num_threads(2)
from isaacsim import SimulationApp
app=SimulationApp({'headless':True})
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.extensions import enable_extension
enable_extension('isaacsim.sensors.physics');app.update()
import isaacsim.sensors.physics as physics_sensors
from isaacsim.sensors.physics.bindings import _sensor
physics_sensors._sensor=_sensor
from isaacsim.sensors.physics.impl.imu_sensor import IMUSensor
from dual_imu import Sample
world=World(stage_units_in_meters=1.,physics_dt=1/120)
add_reference_to_stage(str(root/'assets/vendor/R12_NO_USB_URDF_USD/usd/r12_no_usb.usdc'),'/World/R12')
world.scene.add_default_ground_plane()
robot=world.scene.add(SingleArticulation('/World/R12',name='robot'))
imus=[]
for i,x in enumerate([.057,-.057]):
 imus.append(world.scene.add(IMUSensor(f'/World/R12/base_link/opencr_{i+1}',name=f'imu{i}',frequency=120,translation=np.array([x,0,.16597]),orientation=np.array([1,0,0,0]) if i==0 else np.array([0,0,0,1]))))
world.reset();nominal_position,nominal_orientation=robot.get_world_pose();nominal_position=nominal_position.copy();nominal_position[2]+=.008;print("NOMINAL_ROOT",nominal_position,flush=True);names=robot.dof_names
steer=[names.index(p+'_steer_joint') for p in ['lh1','rh1','lh2','rh2']];wheel=[names.index(p+'_wheel_joint') for p in ['lh1','rh1','lh2','rh2']];arm=[names.index(f'arm_joint{i}') for i in range(1,6)]
kp=np.zeros((1,len(names)));kd=np.full_like(kp,2.);kp[0,steer]=40;kd[0,steer]=4;kp[0,arm]=40;kd[0,arm]=4
robot._articulation_view.set_gains(kps=kp,kds=kd);effort=np.ones_like(kp);effort[0,arm]=2;robot._articulation_view.set_max_efforts(effort)
cases=[('forward',[.12,0,0]),('backward',[-.12,0,0]),('lateral',[0,.12,0]),('diagonal',[.085,.085,0]),('spin',[0,0,.3]),('combined',[.08,.04,.2])];cases=[(n,(np.array(c)*np.array([args.speed/.12,args.speed/.12,min(args.speed/.12,4/3)])).tolist()) for n,c in cases];results=[]
for name,command in cases:
 envelope=MotionEnvelope(speed=args.speed,yaw_speed=.4);wheel_slew=WheelSlew()
 robot.set_world_pose(nominal_position,nominal_orientation);robot.set_linear_velocity(np.zeros(3));robot.set_angular_velocity(np.zeros(3));robot.set_joint_positions(np.zeros(len(names)));robot.set_joint_velocities(np.zeros(len(names)))
 q=np.zeros(len(names));vel=np.zeros(len(names));rows=[];previous=None;controller=ResidualController(root/'models/swerve/rl/policy.pt');fusion=DualImuFusion([[.057,0,.16597],[-.057,0,.16597]],[[0,0,0,1],[0,0,1,0]])
 for step in range(120*(14 if args.reversal else 9)):
  cmd=np.array(command) if 120<=step<120*7 else (-np.array(command) if args.reversal and 120*7<=step<120*12 else np.zeros(3))
  if step%6==0:
   measured_q=np.rint(robot.get_joint_positions()*4096/(2*np.pi))*2*np.pi/4096
   measured_v=np.zeros_like(measured_q) if previous is None else ((measured_q-previous+np.pi)%(2*np.pi)-np.pi)/.05;previous=measured_q.copy()
   readings=[imu.get_current_frame(read_gravity=True) for imu in imus];fused=None
   if all(d['is_valid'] for d in readings):
    try:fused=fusion.update([Sample(step/120,np.asarray(d['ang_vel']),np.asarray(d['lin_acc']),np.eye(3)*.0004,np.eye(3)*.04) for d in readings])
    except ValueError:pass
   if args.envelope:cmd,motion_state=envelope.step(cmd,np.zeros(3) if fused is None else fused['gyro'],np.zeros(3) if fused is None else fused['accel'],.05,valid=fused is not None)
   scale=controller.scale(cmd,measured_q[steer],measured_v[wheel],fused['gyro'],fused['accel']) if fused is not None else 0.
   a,w=actuate(net,cmd,measured_q[steer],.05,q[steer]);q[steer]=a;vel[wheel]=wheel_slew.step(w*scale*args.gain,interlock=fused is None or (np.linalg.norm(cmd)>1e-6 and np.max(np.abs(w))<1e-9)) if args.envelope else np.clip(w*scale*args.gain,-10,10)
   robot.apply_action(ArticulationAction(joint_positions=q,joint_velocities=vel))
  world.step(render=False)
  if step%6==0:
   pos,quat=robot.get_world_pose();w,x,y,z=quat;yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z));v=robot.get_linear_velocity();omega=robot.get_angular_velocity();R=np.array([[np.cos(yaw),np.sin(yaw)],[-np.sin(yaw),np.cos(yaw)]])
   measured=np.r_[R@v[:2],omega[2]]
   roll=math.atan2(2*(w*x+y*z),1-2*(x*x+y*y));pitch=math.asin(np.clip(2*(w*y-z*x),-1,1))
   rows.append({'roll_pitch_deg':np.degrees([roll,pitch]).tolist(),'applied_command':cmd.tolist(),'time':step/120,'position':pos.tolist(),'measured_twist':measured.tolist(),'q_steer':robot.get_joint_positions()[steer].tolist(),'wheel_speeds':robot.get_joint_velocities()[wheel].tolist()})
 measured=np.array([r['measured_twist'] for r in rows if 4<=r['time']<7]);rms=np.sqrt(np.mean((measured-command)**2,axis=0));final=np.array(rows[-1]['measured_twist'])
 reverse=np.array([r['measured_twist'] for r in rows if 10<=r['time']<12]);reverse_rms=np.sqrt(np.mean((reverse+command)**2,axis=0)) if args.reversal else np.zeros(3)
 tilt=max(np.max(np.abs(r['roll_pitch_deg'])) for r in rows)
 result={'reverse_rmse':reverse_rms.tolist(),'max_tilt_deg':float(tilt),'envelope':args.envelope,'case':name,'gain':args.gain,'command':command,'steady_mean':measured.mean(axis=0).tolist(),'steady_rmse':rms.tolist(),'stop_twist':final.tolist(),'passed':bool(max(reverse_rms[:2])<.03 and reverse_rms[2]<.08 and tilt<4 and max(rms[:2])<.03 and rms[2]<.08 and np.linalg.norm(final[:2])<.02 and abs(final[2])<.04),'trace':rows}
 results.append(result);print('SWERVE',json.dumps({k:v for k,v in result.items() if k!='trace'}),flush=True)
 (root/f'reports/local/swerve/physics_ppo_gain_{args.gain:.2f}{'_envelope_'+str(args.speed)+'_wheel_slew' if args.envelope else ''}{'_reversal' if args.reversal else ''}.json').write_text(json.dumps({'scope':'Open-floor nominal wheel-actuated PhysX. Dual native IMUs, quantized encoders and deployed PPO inference. No SLAM/navigation/payload in this actuator test. Source mass/friction nominal; wheel target gain randomized externally.','results':results},indent=2))
world.stop();app.close()
