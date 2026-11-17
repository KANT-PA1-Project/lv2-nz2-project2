"""Free-base PhysX fold/hold test with encoder feedback and evaluator-only tilt."""
import os,json,numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation
root=Path(__file__).resolve().parents[1];out=root/'reports/local/arm_learned_targets_v136';out.mkdir(exist_ok=True,parents=True)
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':1280,'height':720})
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
from pxr import UsdGeom,UsdLux
import omni.usd
world=World(stage_units_in_meters=1.,physics_dt=1/960,rendering_dt=1/60);world.scene.add_default_ground_plane()
add_reference_to_stage(str(root/'models/r12/r12_mobile_gripper.usda'),'/World/R12');stage=omni.usd.get_context().get_stage();UsdLux.DomeLight.Define(stage,'/World/Light').CreateIntensityAttr(1800)
robot=world.scene.add(SingleArticulation('/World/R12',name='r12'));world.reset();names=robot.dof_names
arm=[names.index(f'arm_joint{i}') for i in range(1,6)];grip=[names.index('gripper_'+s+'_joint') for s in ['left','right']]
kp=np.full((1,len(names)),40.);kd=np.full_like(kp,4.);kp[0,arm]=80;kd[0,arm]=8;kp[0,grip]=1000;kd[0,grip]=20;robot._articulation_view.set_gains(kps=kp,kds=kd)
eff=np.ones_like(kp);eff[0,arm]=2;eff[0,grip]=10;robot._articulation_view.set_max_efforts(eff)
set_camera_view(eye=np.array([1.,-1.,1.]),target=np.array([0,0,.5]))
cfg=json.loads((root/'configs/r12_robot.json').read_text());goal=np.array(cfg['home']);q=np.zeros(len(names));q[grip]=[.02768613,-.02768613];rows=[]
from arm_practice.arm_integral_bias import ArmIntegralBias
bias=ArmIntegralBias(cfg['lower'],cfg['upper'],deadband=np.pi/4096);corrected=q[arm].copy()

from arm_practice.learned_ik import ArmFK,LearnedIK
bank=root.parent.parent/'outputs/r12/learned_ik_v1'
archive=json.loads((bank/'evaluation.json').read_text());dataset=np.load(bank/'dataset.npz')
accepted=[r for r in archive['rows'] if r['accepted']]
used={772,1606,649}
remaining=[r for r in accepted if r['index'] not in used]
selected=[remaining[i] for i in np.linspace(0,len(remaining)-1,5,dtype=int)]
fk=ArmFK(root/'models/r12/r12_arm.urdf',cfg['joint_names'],[.12475,0,0]);model=LearnedIK(bank/'model.pt',fk)
results=[];previous=None;frame=0

def phase(goal,seconds,target_xyz):
 global previous,frame
 samples=[]
 for k in range(int(seconds*960)):
  q[arm]+=np.clip(goal-q[arm],-.15/960,.15/960)
  if k%48==0:
   encoder=np.rint(robot.get_joint_positions()[arm]*4096/(2*np.pi))*2*np.pi/4096
   corrected=bias.step(q[arm],encoder,.05)
  applied=q.copy();applied[arm]=corrected
  robot.apply_action(ArticulationAction(joint_positions=applied));world.step(render=k%16==0);frame+=1
  if k%48==0:
   measured=np.rint(robot.get_joint_positions()[arm]*4096/(2*np.pi))*2*np.pi/4096
   velocity=np.zeros(5) if previous is None else (measured-previous)/.05;previous=measured.copy()
   t=UsdGeom.Xformable(stage.GetPrimAtPath('/World/R12/base_link')).ComputeLocalToWorldTransform(0)
   tilt=Rotation.from_matrix(np.array(t)[:3,:3].T.copy()).as_euler('xyz',degrees=True)[:2]
   samples.append(dict(frame=frame,q=measured.tolist(),velocity=velocity.tolist(),reference=q[arm].tolist(),bias=bias.bias.tolist(),tcp_error_mm=float(np.linalg.norm(fk(measured)-target_xyz)*1000),tilt_deg=tilt.tolist(),evaluator_native_joint_fk_error_mm=float(np.linalg.norm(fk(robot.get_joint_positions()[arm])-target_xyz)*1000)))
 return samples

for item in selected:
 home=np.array(cfg['home']);home_rows=phase(home,float(np.max(np.abs(q[arm]-home))/.15+6),fk(home))
 home_ok=all(max(abs(np.array(x['q'])-home))<=.01 and max(abs(np.array(x['velocity'])))<.02 and x['tcp_error_mm']<=1 for x in home_rows[-20:])
 record=dict(index=item['index'],home_passed=home_ok,home_last_second_max_mm=max(x['tcp_error_mm'] for x in home_rows[-20:]))
 if home_ok:
  target=dataset['xyz'][item['index']];solution=model.solve(target);assert solution['accepted'];goal=np.array(solution['q'])
  samples=phase(goal,float(np.max(np.abs(goal-home))/.15+6),target)
  record.update(solution=solution,target_xyz=target.tolist(),samples=samples,final_second_max_mm=max(x['tcp_error_mm'] for x in samples[-20:]),final_second_speed_max=max(max(abs(np.array(x['velocity']))) for x in samples[-20:]),max_tilt_deg=max(float(np.linalg.norm(x['tilt_deg'])) for x in samples))
  record['evaluator_final_second_max_mm']=max(x['evaluator_native_joint_fk_error_mm'] for x in samples[-20:])
  record['passed']=record['final_second_max_mm']<=1 and record['final_second_speed_max']<.02 and record['max_tilt_deg']<4
 else:record.update(passed=False,reason='home_not_settled')
 results.append(record)
 report=dict(selection='Five evenly spaced accepted held-out rows excluding the three controller-comparison targets',results=results,complete=len(results)==5,scope='Standalone 960Hz plant; encoder FK endpoint mm, same home before each goal, reference slew .15rad/s, integral cap .025rad with half encoder tick deadband, original limits and 2Nm cap. Not orientation, contact retention, environment or hardware validation. Native joint FK is evaluator-only and never used by the controller; CAD accuracy and real hardware remain unverified.')
 (out/'evaluation.json').write_text(json.dumps(report,indent=2,default=lambda x:x.item())+'\n');print('TARGET_COMPLETE',{k:v for k,v in record.items() if k not in ['samples','solution']},flush=True)
print('LEARNED_TARGETS_COMPLETE',flush=True);world.stop();app.close()
