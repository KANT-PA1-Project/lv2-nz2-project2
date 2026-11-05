"""Single Isaac GUI articulation sweep; no grasp claim without contact evidence."""
import os,json,sys
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[1];out=root/'reports/local/gripper_sweep';out.mkdir(parents=True,exist_ok=True)
from isaacsim import SimulationApp
app=SimulationApp({'headless':False,'width':1280,'height':720})
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
from pxr import UsdPhysics,UsdGeom,UsdLux,Gf
import omni.usd
world=World(stage_units_in_meters=1,physics_dt=1/120,rendering_dt=1/60)
add_reference_to_stage(str(root/'models/r12/r12_mobile_gripper.usda'),'/World/R12');stage=omni.usd.get_context().get_stage()
f=UsdPhysics.FixedJoint.Define(stage,'/World/anchor');f.CreateBody1Rel().SetTargets(['/World/R12/base_link'])
UsdLux.DomeLight.Define(stage,'/World/Light').CreateIntensityAttr(1800)
robot=world.scene.add(SingleArticulation('/World/R12',name='r12'));world.reset();names=robot.dof_names
set_camera_view(eye=np.array([.60,-.35,.72]),target=np.array([.31,0,.52]))
cfg=json.loads((root/'configs/r12_gripper.json').read_text());inds=[names.index('gripper_'+s+'_joint') for s in ['left','right']]
q=np.zeros(len(names));kp=np.full((1,len(names)),40.);kd=np.full_like(kp,4.);kp[0,inds]=1000;kd[0,inds]=20;robot._articulation_view.set_gains(kps=kp,kds=kd)
eff=np.full_like(kp,2.);eff[0,inds]=10;robot._articulation_view.set_max_efforts(eff)
rows=[];target=cfg['simulation_finger_displacement_m'][0]
for width in [.020,.075,.030,.075,.020]:
 goal=(width-cfg['cad_zero_aperture_m'])/2
 for frame in range(360):
  target+=float(np.clip(goal-target,-.04/120,.04/120));q[inds]=[target,-target]
  robot.apply_action(ArticulationAction(joint_positions=q));world.step(render=frame%2==0)
 actual=robot.get_joint_positions()[inds];gap=cfg['cad_zero_aperture_m']+actual[0]-actual[1]
 rows.append({'target_width_m':width,'measured_width_m':float(gap),'symmetry_error_m':float(abs(sum(actual))),'error_m':float(abs(gap-width))})
(out/'evaluation.json').write_text(json.dumps({'joint_names':names,'sweep':rows,'passed':all(r['error_m']<.001 and r['symmetry_error_m']<.001 for r in rows),'grasp_test':False},indent=2));print('GRIPPER_SWEEP_COMPLETE',rows,flush=True)
world.stop();app.close()
