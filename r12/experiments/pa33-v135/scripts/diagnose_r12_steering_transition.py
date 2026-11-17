"""Isolated flat-floor PhysX A/B transition audit; not a mission or policy acceptance."""
import argparse,json,sys,os
from collections import deque
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--brake',type=int,choices=[0,1],required=True);p.add_argument('--projection',action='store_true');p.add_argument('--coordinate',action='store_true');p.add_argument('--planned-heading',action='store_true');p.add_argument('--smooth-transition',action='store_true');p.add_argument('--encoder-substep',action='store_true');p.add_argument('--wheel-kd',type=float,default=.08);p.add_argument('--physics-hz',type=int,default=120);p.add_argument('--feedback-hz',type=int,default=120);p.add_argument('--speed',type=float,default=float(np.hypot(.08,.4)) );p.add_argument('--ramp-start-stop',action='store_true');p.add_argument('--encoder-window',type=int,choices=[1,2,4],default=1);p.add_argument('--stop-hold',action='store_true');p.add_argument('--path-rotation-deg',type=float,default=0.);p.add_argument('--yaw-rate',type=float,default=0.);p.add_argument('--yaw-reversal',action='store_true');p.add_argument('--yaw-speed-envelope',action='store_true');p.add_argument('--yaw-zero-dwell',action='store_true');p.add_argument('--motion-stop-frame',type=int,choices=[300,380],default=300);p.add_argument('--integral-substep',action='store_true');p.add_argument('--diagnostic-axis-com',action='store_true');p.add_argument('--diagnostic-inertia-profile',type=Path);p.add_argument('--physical-motor-envelope',action='store_true');a=p.parse_args()
if not np.isfinite(a.yaw_rate) or abs(a.yaw_rate)>.4:raise ValueError('Invalid yaw rate')
if not 0<a.speed<=.899795667:raise ValueError('Invalid speed')
if a.output.exists():raise ValueError('Preserve existing output')
if not 0<a.wheel_kd<=.08:raise ValueError('Invalid wheel gain')
if a.physics_hz not in [120,240,480,960,1920] or a.feedback_hz<20 or a.physics_hz%a.feedback_hz or a.feedback_hz%20:raise ValueError('Invalid physics/feedback rates')
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'src'))
from arm_practice.latest_policy import allocate,dc_effort
from arm_practice.steering_transition import brake_before_large_turn,project_current_heading,coordinate_turn_with_drive
from arm_practice.wheel_integral import WheelIntegral
from arm_practice.wheel_position_hold import WheelPositionHold
from arm_practice.encoder_velocity import wrapped_velocity
from arm_practice.heading_feasibility import choose_heading,world_to_body
angle=np.radians(a.path_rotation_deg)
if not np.isfinite(angle):raise ValueError('Invalid path rotation')
path_rotation=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
planned_heading,planned_margin=choose_heading(np.array([[.08,-.4],[-.08,-.4]])@path_rotation.T) if a.planned_heading else (0.,None)
from isaacsim import SimulationApp
app=SimulationApp({'headless':True})
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from pxr import UsdGeom,UsdPhysics,Gf
import omni.usd
world=World(stage_units_in_meters=1.,physics_dt=1/a.physics_hz,rendering_dt=1/20);world.scene.add_default_ground_plane()
add_reference_to_stage(str(root/'models/r12/r12_mobile_gripper.usda'),'/World/R12')
inertia_profile=None
if a.diagnostic_inertia_profile:
 if a.diagnostic_axis_com:raise ValueError('Choose one diagnostic mass profile')
 inertia_profile=json.loads(a.diagnostic_inertia_profile.read_text())
 for name,data in inertia_profile['wheels'].items():
  prim=omni.usd.get_context().get_stage().GetPrimAtPath('/World/R12/'+name);mass=UsdPhysics.MassAPI(prim);tensor=np.array(data['inertia_tensor']);vals,axes=np.linalg.eigh(tensor)
  if np.linalg.det(axes)<0:axes[:,0]*=-1
  if min(vals)<=0 or not np.isfinite(tensor).all():raise ValueError('Invalid diagnostic inertia')
  quat=Rotation.from_matrix(axes).as_quat();mass.GetMassAttr().Set(data['mass']);mass.GetCenterOfMassAttr().Set(Gf.Vec3f(*data['center_of_mass']));mass.GetDiagonalInertiaAttr().Set(Gf.Vec3f(*vals));mass.GetPrincipalAxesAttr().Set(Gf.Quatf(float(quat[3]),Gf.Vec3f(*quat[:3])))
com_overrides=[]
if a.diagnostic_axis_com:
 for name in ['lh1','rh1','lh2','rh2']:
  prim=omni.usd.get_context().get_stage().GetPrimAtPath('/World/R12/'+name+'_wheel');mass=UsdPhysics.MassAPI(prim);original=mass.GetCenterOfMassAttr().Get();center=Gf.Vec3f(original[0],0,0) if name.endswith('1') else Gf.Vec3f(0,original[1],0)
  mass.GetCenterOfMassAttr().Set(center);com_overrides.append({'wheel':name,'original':list(original),'diagnostic':list(center)})
robot=world.scene.add(SingleArticulation('/World/R12',name='robot',position=np.array([0.,0.,.008]),orientation=np.array([np.cos(planned_heading/2),0.,0.,np.sin(planned_heading/2)])));world.reset();names=robot.dof_names
steer=[names.index(n+'_steer_joint') for n in ['lh1','rh1','lh2','rh2']];wheel=[names.index(n+'_wheel_joint') for n in ['lh1','rh1','lh2','rh2']];arm=[names.index(f'arm_joint{i}') for i in range(1,6)];grip=[names.index('gripper_'+s+'_joint') for s in ['left','right']]
q=robot.get_joint_positions().copy();arm_goal=np.array([0,.33,.33,.33,0]);grip_goal=np.array([.02768613,-.02768613])
kp=np.zeros((1,len(names)));kd=np.zeros_like(kp);kp[0,arm]=80;kd[0,arm]=8;kp[0,grip]=1000;kd[0,grip]=20;robot._articulation_view.set_gains(kps=kp,kds=kd)
eff=np.ones_like(kp);eff[0,steer]=.54;eff[0,wheel]=.108;eff[0,arm]=2;eff[0,grip]=10;robot._articulation_view.set_max_efforts(eff)
previous=None;w=np.zeros(4);enc=np.zeros(4);integral=WheelIntegral();last=robot.get_joint_positions().copy();last[wheel]=np.rint(last[wheel]/4*4096/(2*np.pi))*2*np.pi/4096*4;rows=[]
sub_previous=np.rint(robot.get_joint_positions()[wheel]/4*4096/(2*np.pi))*2*np.pi/4096*4
encoder_history=deque(maxlen=a.encoder_window)
hold=WheelPositionHold();unwrapped_encoder=sub_previous.copy();motion_seen=False
for frame in range(a.motion_stop_frame+20):
 # Same predetermined commands for both cases; encoders are the only feedback.
 command=np.array([.08,-.4,0.]) if 140<=frame<220 else (np.array([-.08,-.4,0.]) if 220<=frame<a.motion_stop_frame else np.zeros(3))
 if a.smooth_transition and 220<=frame<230:
  u=(frame-219)/10.;command[0]=.08-.16*(3*u*u-2*u*u*u)
 command*=a.speed/np.hypot(.08,.4)
 if 140<=frame<a.motion_stop_frame:command[2]=a.yaw_rate
 if a.yaw_reversal and frame>=220:
  t=min(1.,(frame-219)/20.);command[2]=a.yaw_rate*(1-2*(3*t*t-2*t*t*t)) if frame<a.motion_stop_frame else 0.
 if a.yaw_zero_dwell and 220<=frame<a.motion_stop_frame:
  if frame<230:
   t=(frame-219)/10.;command[2]=a.yaw_rate*(1-(3*t*t-2*t*t*t))
  elif frame<270:command[2]=0.
  elif frame<280:
   t=(frame-269)/10.;command[2]=-a.yaw_rate*(3*t*t-2*t*t*t)
  else:command[2]=-a.yaw_rate
 if a.yaw_speed_envelope and a.yaw_rate and 140<=frame<a.motion_stop_frame:
  floor=min(.425,a.speed);command[:2]*=(floor+(a.speed-floor)*min(1.,abs(command[2]/a.yaw_rate)))/a.speed
 if a.ramp_start_stop:
  u=min(1.,max(0.,(frame-139)/40.),max(0.,(a.motion_stop_frame-frame)/40.))
  command*=3*u*u-2*u*u*u
 command[:2]=path_rotation@command[:2]
 motion_seen=motion_seen or np.linalg.norm(command)>1e-6
 command_world=command.copy();command[:2]=world_to_body(command_world[:2],planned_heading)
 q[arm]+=np.clip(arm_goal-q[arm],-.15*.05,.15*.05);q[grip]+=np.clip(grip_goal-q[grip],-.04*.05,.04*.05)
 current=robot.get_joint_positions();target,desired=allocate(command,current[steer],previous,np.zeros(4));braking=np.zeros(4,dtype=bool)
 if a.projection:desired=project_current_heading(command,current[steer])
 if a.coordinate:target,desired=coordinate_turn_with_drive(command,current[steer],target,enc)
 if a.brake:target,desired,braking=brake_before_large_turn(target,current[steer],desired,enc)
 w+=np.clip(desired-w,-24.45*.05,24.45*.05);previous=target;q[steer]=target
 feed=np.zeros(4) if a.integral_substep else integral.step(w,enc,.05,a.wheel_kd,enabled=np.linalg.norm(command)>1e-6)
 substeps=[]
 for sub in range(a.physics_hz//20):
  pos=robot.get_joint_positions();vel=robot.get_joint_velocities()
  if sub%(a.physics_hz//a.feedback_hz)==0:
   sub_measured=np.rint(pos[wheel]/4*4096/(2*np.pi))*2*np.pi/4096*4;sub_encoder=wrapped_velocity(sub_measured,sub_previous,1/a.feedback_hz);sub_previous=sub_measured.copy()
   unwrapped_encoder+=sub_encoder/a.feedback_hz
   hold_torque=hold.step(unwrapped_encoder,enabled=a.stop_hold and motion_seen and np.linalg.norm(command)<1e-6 and np.max(abs(w))<1e-6)
   encoder_history.append(sub_encoder.copy());sub_encoder=np.mean(encoder_history,axis=0)
   if a.integral_substep:feed=integral.step(w,sub_encoder,1/a.feedback_hz,a.wheel_kd,enabled=np.linalg.norm(command)>1e-6)
   wheel_feedback=sub_encoder if a.encoder_substep else vel[wheel];torque=np.zeros(len(names));torque[steer]=dc_effort(q[steer],pos[steer],vel[steer],True);torque[wheel]=dc_effort(w,pos[wheel],wheel_feedback,wheel_kd=a.wheel_kd,wheel_feedforward=feed+hold_torque)
  applied_torque=torque.copy()
  if a.physical_motor_envelope:
   # Motor plant back-EMF uses physical shaft speed; controller still receives encoder only.
   upper=np.clip(.54*(1-vel[wheel]/29.321531434),-.108,.108);lower=np.clip(.54*(-1-vel[wheel]/29.321531434),-.108,.108)
   applied_torque[wheel]=np.clip(torque[wheel],lower,upper)
  substeps.append({'native':vel[wheel].tolist(),'encoder':sub_encoder.tolist(),'torque':applied_torque[wheel].tolist()});robot.apply_action(ArticulationAction(joint_positions=q,joint_efforts=applied_torque));world.step(render=False)

 now=robot.get_joint_positions();measured=np.rint(now[wheel]/4*4096/(2*np.pi))*2*np.pi/4096*4;enc=wrapped_velocity(measured,last[wheel],.05);last=now.copy();last[wheel]=measured
 # Ground truth below is logged for offline evaluation only, never sent to allocator.
 T=UsdGeom.Xformable(omni.usd.get_context().get_stage().GetPrimAtPath('/World/R12/base_link')).ComputeLocalToWorldTransform(0)
 rows.append({'frame':frame,'command':command.tolist(),'command_world':command_world.tolist(),'actual_xyz':list(T.ExtractTranslation()),'actual_rpy':Rotation.from_matrix(np.array(T)[:3,:3].T.copy()).as_euler('xyz').tolist(),'steer':now[steer].tolist(),'wheel_encoder':enc.tolist(),'braking':braking.tolist(),'substeps':substeps,'arm_error_rad':float(np.max(abs(now[arm]-arm_goal)))})
 if frame%40==0:print('TRANSITION_FRAME',frame,flush=True)
a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps({'scope':'Isolated flat-floor transition physics audit, no perception/navigation or mission acceptance','brake':bool(a.brake),'projection':a.projection,'coordinate':a.coordinate,'wheel_kd':a.wheel_kd,'encoder_substep':a.encoder_substep,'smooth_transition':a.smooth_transition,'planned_heading':planned_heading,'steering_margin':planned_margin,'physical_motor_envelope':a.physical_motor_envelope,'diagnostic_inertia_profile':inertia_profile,'diagnostic_axis_com':a.diagnostic_axis_com,'com_overrides':com_overrides,'integral_substep':a.integral_substep,'motion_stop_frame':a.motion_stop_frame,'yaw_zero_dwell':a.yaw_zero_dwell,'yaw_reversal':a.yaw_reversal,'yaw_speed_envelope':a.yaw_speed_envelope,'command_frame':'body' if a.yaw_rate else 'world','yaw_rate':a.yaw_rate,'path_rotation_deg':a.path_rotation_deg,'stop_hold':a.stop_hold,'encoder_window':a.encoder_window,'requested_speed':a.speed,'ramp_start_stop':a.ramp_start_stop,'physics_hz':a.physics_hz,'feedback_hz':a.feedback_hz,'rows':rows},indent=2));print('TRANSITION_COMPLETE',flush=True);app.close()
