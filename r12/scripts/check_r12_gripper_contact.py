"""Contact-only pick/lift/release diagnostic. No attachment joints or teleport grasp."""
import json,os
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[1];out=root/'reports/local/gripper_contact';out.mkdir(exist_ok=True)
from isaacsim import SimulationApp
app=SimulationApp({'headless':False,'width':1280,'height':720})
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation,SingleRigidPrim
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
from pxr import UsdGeom,UsdPhysics,UsdShade,UsdLux,Gf
import omni.usd
world=World(stage_units_in_meters=1.,physics_dt=1/120,rendering_dt=1/60);add_reference_to_stage(str(root/'models/r12/r12_mobile_gripper.usda'),'/World/R12');stage=omni.usd.get_context().get_stage();f=UsdPhysics.FixedJoint.Define(stage,'/World/anchor');f.CreateBody1Rel().SetTargets(['/World/R12/base_link'])
UsdLux.DomeLight.Define(stage,'/World/Light').CreateIntensityAttr(1800)
# A raised fixture isolates the gripper from the unfinished mission reach planner.
table=UsdGeom.Cube.Define(stage,'/World/Fixture');table.CreateSizeAttr(1);table.AddTranslateOp().Set(Gf.Vec3d(.344,0,.4799));table.AddScaleOp().Set(Gf.Vec3f(.09,.11,.02));UsdPhysics.CollisionAPI.Apply(table.GetPrim())
cyl=UsdGeom.Cylinder.Define(stage,'/World/Puck');cyl.CreateRadiusAttr(.015);cyl.CreateHeightAttr(.06);cyl.CreateAxisAttr('Z');cyl.AddTranslateOp().Set(Gf.Vec3d(.3438,0,.5201));cyl.CreateDisplayColorAttr([Gf.Vec3f(.8,.02,.02)]);UsdPhysics.CollisionAPI.Apply(cyl.GetPrim());UsdPhysics.RigidBodyAPI.Apply(cyl.GetPrim());UsdPhysics.MassAPI.Apply(cyl.GetPrim()).CreateMassAttr(.025)
mat=UsdShade.Material.Define(stage,'/World/RubberAssumption');phy=UsdPhysics.MaterialAPI.Apply(mat.GetPrim());phy.CreateStaticFrictionAttr(1.);phy.CreateDynamicFrictionAttr(.8);phy.CreateRestitutionAttr(0.)
for path in ['/World/Puck','/World/R12/gripper_left/pad_collision','/World/R12/gripper_right/pad_collision']:
 UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(path)).Bind(mat,materialPurpose='physics')
robot=world.scene.add(SingleArticulation('/World/R12',name='r12'));puck=world.scene.add(SingleRigidPrim('/World/Puck',name='puck'));world.reset();names=robot.dof_names;gi=[names.index('gripper_'+s+'_joint') for s in ['left','right']];ai=names.index('arm_joint2')
kp=np.full((1,len(names)),400.);kd=np.full_like(kp,15.);kp[0,gi]=1000;kd[0,gi]=20;eff=np.full_like(kp,2.);eff[0,gi]=10;robot._articulation_view.set_gains(kps=kp,kds=kd);robot._articulation_view.set_max_efforts(eff)
set_camera_view(eye=np.array([.60,-.35,.72]),target=np.array([.33,0,.53]));cfg=json.loads((root/'configs/r12_gripper.json').read_text());q=np.zeros(len(names));target=(.075-cfg['cad_zero_aperture_m'])/2;angle=0.;rows=[]
for phase,width,desired_angle,frames in [('open',.075,0,240),('close',.027,0,360),('lift',.027,.12,480),('hold',.027,.12,240),('lower',.027,0,480),('release',.075,0,360)]:
 for frame in range(frames):
  target+=float(np.clip((width-cfg['cad_zero_aperture_m'])/2-target,-.04/120,.04/120));angle+=float(np.clip(desired_angle-angle,-.04/120,.04/120));q[gi]=[target,-target];q[ai]=angle;robot.apply_action(ArticulationAction(joint_positions=q));world.step(render=frame%2==0)
  if frame%12==0:
   actual=robot.get_joint_positions();pos,_=puck.get_world_pose();rows.append({'phase':phase,'puck_xyz':pos.tolist(),'measured_aperture_m':float(cfg['cad_zero_aperture_m']+actual[gi[0]]-actual[gi[1]]),'arm_joint2':float(actual[ai])})
base=rows[19]['puck_xyz'][2];held=[r['puck_xyz'][2]-base for r in rows if r['phase']=='hold'];result={'assumptions':{'puck_mass_kg':.025,'static_friction':1.,'dynamic_friction':.8,'finger_force_limit_N':10},'method':'Measured PhysX puck motion under pad contact; no attach joint and no puck pose writes','lift_hold_passed':min(held)>.025,'minimum_held_lift_m':min(held),'release_height_m':rows[-1]['puck_xyz'][2],'release_on_fixture':abs(rows[-1]['puck_xyz'][2]-.5199)<.005,'rows':rows,'mission_pick_place':False}
(out/'evaluation.json').write_text(json.dumps(result,indent=2));print('GRIPPER_CONTACT_COMPLETE',result['lift_hold_passed'],min(held),flush=True);world.stop();app.close()
