#!/usr/bin/env python3
"""Wheel-actuated R12 navigation in supplied maze. No pose teleport during driving."""
import sys,json,math,numpy as np,torch
from pathlib import Path
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'src'))
from arm_practice.navigation import Grid,Policy
policy=Policy();policy.load_state_dict(torch.load(root/'models/navigation/waypoint_policy.pt',weights_only=True));policy.eval();torch.set_num_threads(2)
grid=Grid(root/'models/navigation/maze_grid.npz');cfg=json.loads((root/'models/navigation/mission_navigation.json').read_text());routes=json.loads((root/'models/navigation/routes.json').read_text())['routes']
from isaacsim import SimulationApp
app=SimulationApp({'headless':True})
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
world=World(stage_units_in_meters=1.,physics_dt=1/120)
add_reference_to_stage(str(root/'models/navigation/mission_scene.usda'),'/World/Mission')
add_reference_to_stage(str(root/'assets/vendor/R12_NO_USB_URDF_USD/usd/r12_no_usb.usdc'),'/World/R12')
world.scene.add_default_ground_plane()
robot=world.scene.add(SingleArticulation('/World/R12',name='robot',position=np.array([*cfg['start'],.008]),orientation=np.array([0,0,0,1.])))
world.reset();names=robot.dof_names;print('DOFS',names,flush=True)
steer=[names.index(p+'_steer_joint') for p in ['lh1','rh1','lh2','rh2']];wheel=[names.index(p+'_wheel_joint') for p in ['lh1','rh1','lh2','rh2']];arm=[names.index(f'arm_joint{i}') for i in range(1,6)]
kp=np.zeros((1,len(names)));kd=np.full_like(kp,2.);kp[0,steer]=40;kd[0,steer]=4;kp[0,arm]=40;kd[0,arm]=4
robot._articulation_view.set_gains(kps=kp,kds=kd)
maxeff=np.ones_like(kp);maxeff[0,arm]=2;robot._articulation_view.set_max_efforts(maxeff)
q=np.zeros(len(names));q[steer[:2]]=np.pi/2;robot.set_joint_positions(q)
for _ in range(240):robot.apply_action(ArticulationAction(joint_positions=q,joint_velocities=np.zeros(len(names))));world.step(render=False)
xy_offsets=np.array([[.1,.1],[.1,-.1],[-.1,.1],[-.1,-.1]])
base_directions=np.array([-np.pi/2,-np.pi/2,0,np.pi]);radius=.0306872
log=[];results=[];totalstep=0
for segment in routes:
 path=np.array(segment['path']);stops=0;distance=0;previous=None;done=False;failure=None
 for step in range(120*180):
  pos,quat=robot.get_world_pose();w,x,y,z=quat;yaw=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z));xy=pos[:2]
  if previous is not None:distance+=float(np.linalg.norm(xy-previous))
  previous=xy.copy()
  if np.linalg.norm(xy-path[-1])<.05:done=True;break
  if not np.isfinite(pos).all() or abs(pos[2])>.2:failure='unstable_base';break
  cy,cx=grid.cell(xy)
  if not(0<=cy<grid.clear.shape[0] and 0<=cx<grid.clear.shape[1]) or grid.clear[cy,cx]<.26:failure='clearance_violation';break
  if step%6==0:
   v,stop=grid.command(policy,xy,path);stops+=int(stop)
   yaw_error=(np.pi-yaw+np.pi)%(2*np.pi)-np.pi;omega=np.clip(1.5*yaw_error,-.4,.4)
   rot=np.array([[np.cos(yaw),np.sin(yaw)],[-np.sin(yaw),np.cos(yaw)]]);local=rot@v
   velocities=local+omega*np.column_stack([-xy_offsets[:,1],xy_offsets[:,0]])
   speed=np.linalg.norm(velocities,axis=1);angles=(np.arctan2(velocities[:,1],velocities[:,0])-base_directions+np.pi)%(2*np.pi)-np.pi
   reverse=np.abs(angles)>np.pi/2;angles[reverse]-=np.sign(angles[reverse])*np.pi;speed[reverse]*=-1
   measured=robot.get_joint_positions()[steer];targets=measured+np.clip(angles-measured,-.05,.05)
   alignment=np.maximum(0,np.cos(angles-measured));speed*=alignment
   q[steer]=targets;vel=np.zeros(len(names));vel[wheel]=np.clip(speed/radius,-10,10)
   # Exclude wheel position targets: velocity control with zero stiffness.
   robot.apply_action(ArticulationAction(joint_positions=q,joint_velocities=vel))
  world.step(render=False);totalstep+=1
  if step%12==0:
   log.append({'segment':segment['to'],'time_s':totalstep/120,'xy':xy.tolist(),'yaw':yaw,'base_z':float(pos[2]),'clearance_m':float(grid.clear[cy,cx]),'target_velocity':v.tolist()})
  if step%1200==0:print('PROGRESS',segment['to'],step/120,xy.tolist(),flush=True)
 result={'from':segment['from'],'to':segment['to'],'success':done,'seconds':step/120,'distance_m':distance,'safety_stops':stops,'failure':failure,'final_xy':xy.tolist()};results.append(result);print('SEGMENT',json.dumps(result),flush=True)
 if not done:break
# Physically stop wheels and measure return settling, not copied target state.
for _ in range(360):robot.apply_action(ArticulationAction(joint_positions=q,joint_velocities=np.zeros(len(names))));world.step(render=False)
final,_=robot.get_world_pose();velocity=robot.get_linear_velocity()
report={'scope':'Known-map learned local controller, wheel-actuated PhysX, home arm, no puck transport or vision','results':results,'completed_segments':sum(r['success'] for r in results),'required_segments':len(routes),'final_position':final.tolist(),'final_linear_velocity':velocity.tolist(),'trace':log}
(root/'reports/local/navigation/physics.json').write_text(json.dumps(report,indent=2));world.stop();app.close()
