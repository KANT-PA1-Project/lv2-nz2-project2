"""Free-base PhysX fold/hold test with encoder feedback and evaluator-only tilt."""
import os,json,numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation
root=Path(__file__).resolve().parents[1];out=root/'reports/local/transport_physics';out.mkdir(exist_ok=True,parents=True)
from isaacsim import SimulationApp
app=SimulationApp({'headless':False,'width':1280,'height':720})
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
from pxr import UsdGeom,UsdLux
import omni.usd
world=World(stage_units_in_meters=1.,physics_dt=1/120,rendering_dt=1/60);world.scene.add_default_ground_plane()
add_reference_to_stage(str(root/'models/r12/r12_mobile_gripper.usda'),'/World/R12');stage=omni.usd.get_context().get_stage();UsdLux.DomeLight.Define(stage,'/World/Light').CreateIntensityAttr(1800)
robot=world.scene.add(SingleArticulation('/World/R12',name='r12'));world.reset();names=robot.dof_names
arm=[names.index(f'arm_joint{i}') for i in range(1,6)];grip=[names.index('gripper_'+s+'_joint') for s in ['left','right']]
kp=np.full((1,len(names)),40.);kd=np.full_like(kp,4.);kp[0,arm]=80;kd[0,arm]=8;kp[0,grip]=1000;kd[0,grip]=20;robot._articulation_view.set_gains(kps=kp,kds=kd)
eff=np.ones_like(kp);eff[0,arm]=2;eff[0,grip]=10;robot._articulation_view.set_max_efforts(eff)
set_camera_view(eye=np.array([1.,-1.,1.]),target=np.array([0,0,.5]))
cfg=json.loads((root/'configs/r12_transport.json').read_text());goal=np.array(cfg['arm_target']);q=np.zeros(len(names));q[grip]=[.02768613,-.02768613];rows=[]
for k in range(1080):
 if k>=240:q[arm]+=np.clip(goal-q[arm],-.15/120,.15/120)
 robot.apply_action(ArticulationAction(joint_positions=q));world.step(render=k%2==0)
 if k%6==0:
  t=UsdGeom.Xformable(stage.GetPrimAtPath('/World/R12/base_link')).ComputeLocalToWorldTransform(0);rpy=Rotation.from_matrix(np.array(t)[:3,:3].T.copy()).as_euler('xyz',degrees=True)
  rows.append({'frame':k,'q':robot.get_joint_positions()[arm].tolist(),'velocity':robot.get_joint_velocities()[arm].tolist(),'tilt_deg':rpy[:2].tolist()})
last=rows[-1];error=float(np.max(np.abs(np.array(last['q'])-goal)));tilt=max(np.max(np.abs(r['tilt_deg'])) for r in rows)
report={'samples':rows,'goal':goal.tolist(),'final_error_rad':error,'max_base_tilt_deg':float(tilt),'final_joint_speed_rad_s':max(abs(x) for x in last['velocity']),'passed':bool(error<.04 and tilt<4 and max(abs(x) for x in last['velocity'])<.02),'gains':{'stiffness':80,'damping':8,'effort_limit_Nm':2},'limitations':['Empty gripper only; no payload or physical hardware certification.','Sampled CAD checks are separate; this is not a whole-mission collision certificate.']}
(out/'evaluation.json').write_text(json.dumps(report,indent=2)+'\n');print('TRANSPORT_PHYSICS_COMPLETE', {k:v for k,v in report.items() if k!='samples'},flush=True);world.stop();app.close()
