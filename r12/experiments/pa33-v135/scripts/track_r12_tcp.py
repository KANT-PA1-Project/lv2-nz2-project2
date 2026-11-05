#!/usr/bin/env python3
"""Cartesian IK + gravity-compensated joint PD tracking, simulation only."""
from pathlib import Path
import sys,json,xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'src'))
from arm_practice.kinematics import forward_transform
urdf=root/'models/r12/r12_tcp.urdf';names=[f'arm_joint{i}' for i in range(1,6)]
def fk(q,link='grasp_tcp'):return forward_transform(urdf,link,dict(zip(names,q)))
masses=[]
for link in ET.parse(urdf).getroot().findall('link'):
 if link.get('name') not in [f'arm_link{i}' for i in range(1,6)]:continue
 inert=link.find('inertial');o=inert.find('origin');masses.append((link.get('name'),float(inert.find('mass').get('value')),np.r_[np.fromstring(o.get('xyz'),sep=' '),1]))
def potential(q):return sum(m*9.81*(fk(q,n)@com)[2] for n,m,com in masses)
def gravity(q):
 e=np.eye(5)*1e-5
 return np.array([(potential(q+d)-potential(q-d))/2e-5 for d in e])
q0=np.array([0,.04,-.08,.04,0]);center=fk(q0)[:3,3]
t=np.linspace(0,1,241);smooth=(1-np.cos(2*np.pi*t))/2
paths={
 'line':center+np.outer(smooth,[0,.025,0]),
 'circle':center+np.column_stack([np.zeros(len(t)),.012*np.sin(2*np.pi*t),.012*(1-np.cos(2*np.pi*t))]),
 'approach_lift':center+np.column_stack([-.012*np.sin(np.pi*t)**2,np.zeros(len(t)),.018*np.sin(np.pi*t)**4])}
plans={}
import hashlib
plan_key=hashlib.sha256(urdf.read_bytes()+b''.join(points.tobytes() for points in paths.values())).hexdigest()[:16]
cache=root/'reports/local'/f'r12_tcp_plans_{plan_key}.npz'
for name,points in paths.items():
 if cache.exists():
  saved=np.load(cache);plans[name]=(points,saved[name+'_q'],saved[name+'_g'],float(saved[name+'_err']));continue
 prev=q0.copy();qs=[];errs=[];gs=[]
 for p in points:
  ans=least_squares(lambda q:np.r_[fk(q)[:3,3]-p,1e-5*(q-prev)],prev,bounds=([-1.5,-.32,-.32,-.32,-.32],[1.5,.32,.32,.32,.32]),gtol=1e-11,ftol=1e-11,xtol=1e-11)
  prev=ans.x;qs.append(prev.copy());errs.append(float(np.linalg.norm(fk(prev)[:3,3]-p)));gs.append(gravity(prev))
 if max(errs)>.0001:raise RuntimeError((name,max(errs)))
 plans[name]=(points,np.array(qs),np.array(gs),max(errs))
np.savez(cache,**{name+suffix:value for name,(_,q,g,e) in plans.items() for suffix,value in [('_q',q),('_g',g),('_err',e)]})
from isaacsim import SimulationApp
app=SimulationApp({'headless':True})
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation,SingleXFormPrim
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
world=World(stage_units_in_meters=1.,physics_dt=1/120)
add_reference_to_stage(str(root/'models/r12/r12_tcp.usda'),'/World/R12')
robot=world.scene.add(SingleArticulation('/World/R12',name='robot'));world.reset()
print('INITIAL_GAINS',robot._articulation_view.get_gains(),flush=True)
robot._articulation_view.set_gains(kps=np.full((1,5),40.),kds=np.full((1,5),4.))
robot._articulation_view.set_max_efforts(np.full((1,5),2.))
tcp=SingleXFormPrim('/World/R12/arm_link5/grasp_tcp')
assert robot.dof_names==names,robot.dof_names
results=[]
for name,(points,qs,gs,ikerr) in plans.items():
 # Initialization only. All tracking samples below use actuated physics, never teleportation.
 robot.set_joint_positions(qs[0]);robot.set_joint_velocities(np.zeros(5))
 for _ in range(240):
  robot.apply_action(ArticulationAction(joint_positions=qs[0]+gs[0]/40));world.step(render=True)
 rows=[]
 for k in range(len(qs)-1):
  for sub in range(4):
   a=(sub+1)/4;q=(1-a)*qs[k]+a*qs[k+1];g=(1-a)*gs[k]+a*gs[k+1];p=(1-a)*points[k]+a*points[k+1]
   robot.apply_action(ArticulationAction(joint_positions=q+g/40));world.step(render=True)
   pos,_=tcp.get_world_pose();actual=robot.get_joint_positions();vel=robot.get_joint_velocities()
   rows.append({'target_m':p.tolist(),'actual_m':pos.tolist(),'error_m':float(np.linalg.norm(pos-p)),'q':actual.tolist(),'velocity_rad_s':vel.tolist()})
 errors=np.array([r['error_m'] for r in rows]);speed=max(max(abs(v) for v in r['velocity_rad_s']) for r in rows)
 result={'name':name,'duration_s':8,'ik_max_error_m':ikerr,'tracking_rms_m':float(np.sqrt(np.mean(errors**2))),'tracking_max_m':float(max(errors)),'final_error_m':float(errors[-1]),'max_joint_speed_rad_s':speed,'within_joint_limits':all(np.all(np.abs(r['q'][1:])<=.35) and abs(r['q'][0])<=np.pi/2 for r in rows),'samples':rows}
 result['passed']=result['tracking_max_m']<=.005 and result['within_joint_limits'] and speed<=.4
 results.append(result);print('TRACK_RESULT',json.dumps({k:v for k,v in result.items() if k!='samples'}),flush=True)
report={'tcp':'grasp_tcp','calibration':'CAD only, no real hardware','effort_limit_Nm':2,'stiffness':40,'damping':4,'controller':'position IK + URDF gravity/40 target compensation','acceptance_max_error_m':.005,'orientation_constrained':False,'collision_verified':False,'grasp_verified':False,'results':results}
(root/'reports/local/r12_tcp_tracking.json').write_text(json.dumps(report,indent=2));world.stop();app.close()
