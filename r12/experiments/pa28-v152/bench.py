from pathlib import Path
import os,json,time,sys,traceback
import numpy as np
root=Path(__file__).parent;out=Path(os.environ['FLEET_RUN_DIR']);mode=sys.argv[1]
assert mode in ('smoke','lift_only'), 'bounded preparation permits smoke or lift_only, not release/retreat/stow'
def save(n,d):
 p=out/n;t=p.with_suffix('.tmp');t.write_text(json.dumps(d,indent=2));t.replace(p)
# Fixture FK is construction/evaluation only; never controlfeedback.
exec(compile((root/'audit.py').read_text().split('rows=[]')[0],'audit.py','exec'))
exec(compile((root/'path.py').read_text().split('center=(tg@TCP)')[0],'geometry_checks','exec'))
qfixture=np.array([0,.15,-.3,.15,0]);T=tf(qfixture)['arm_link5'];center=(T@TCP)[:3]+0.0065*T[:3,0]-[0,0,.012];center[2]+=.008
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'fast_shutdown':False,'active_gpu':0,'physics_gpu':0,'multi_gpu':False,'width':640,'height':480})
rows=[];contacts=[];contact_points=[];fault=None;phase='initialization'
try:
 from isaacsim.core.api import World
 from isaacsim.core.prims import SingleArticulation,SingleRigidPrim
 from isaacsim.core.utils.stage import add_reference_to_stage
 from isaacsim.core.utils.types import ArticulationAction
 from pxr import UsdGeom,UsdPhysics,PhysxSchema,PhysicsSchemaTools,UsdShade,Gf
 import omni.usd,omni.physx
 from arm_gripper_lqr import design
 world=World(stage_units_in_meters=1.,physics_dt=1/960,rendering_dt=1/20);stage=omni.usd.get_context().get_stage();spawn=UsdGeom.Xform.Define(stage,'/World/Spawn');spawn.AddTranslateOp().Set(Gf.Vec3d(0,0,.008));add_reference_to_stage(str(root/'models/r12/r12_mobile_gripper.usda'),'/World/Spawn/R12')
 frozen=[]
 for prim in stage.Traverse():
  if prim.IsA(UsdPhysics.Joint) and ('wheel_joint' in prim.GetName() or 'steer_joint' in prim.GetName()):
   prim.SetTypeName('PhysicsFixedJoint');prim.RemoveAPI(UsdPhysics.DriveAPI,'angular');frozen.append(str(prim.GetPath()))
 anchor=UsdPhysics.FixedJoint.Define(stage,'/World/Anchor');anchor.CreateBody1Rel().SetTargets(['/World/Spawn/R12/base_link']);anchor.CreateLocalPos0Attr().Set(Gf.Vec3f(0,0,.008))
 height=center[2]-.0301;support=UsdGeom.Cube.Define(stage,'/World/Support');support.CreateSizeAttr(1);support.AddTranslateOp().Set(Gf.Vec3d(center[0],center[1],height/2));support.AddScaleOp().Set(Gf.Vec3f(.03,.03,height));UsdPhysics.CollisionAPI.Apply(support.GetPrim())
 cy=UsdGeom.Cube.Define(stage,'/World/Puck');cy.CreateSizeAttr(1.);cy.CreateDisplayColorAttr([Gf.Vec3f(.1,.8,.1)]);cy.AddTranslateOp().Set(Gf.Vec3d(*center));cy.AddScaleOp().Set(Gf.Vec3f(.03,.03,.06));UsdPhysics.CollisionAPI.Apply(cy.GetPrim());UsdPhysics.RigidBodyAPI.Apply(cy.GetPrim());UsdPhysics.MassAPI.Apply(cy.GetPrim()).CreateMassAttr(.025)
 mat=UsdShade.Material.Define(stage,'/World/BenchFriction');ma=UsdPhysics.MaterialAPI.Apply(mat.GetPrim());ma.CreateStaticFrictionAttr(.5);ma.CreateDynamicFrictionAttr(.5);ma.CreateRestitutionAttr(0.)
 for path in ['/World/Puck','/World/Support','/World/Spawn/R12/gripper_left/pad_collision','/World/Spawn/R12/gripper_right/pad_collision']:
  UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(path)).Bind(mat,materialPurpose='physics')
 for path in ['/World/Puck','/World/Spawn/R12/gripper_left','/World/Spawn/R12/gripper_right']:
  PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(path)).CreateThresholdAttr(0.)
 robot=world.scene.add(SingleArticulation('/World/Spawn/R12/base_link',name='arm'));puck=world.scene.add(SingleRigidPrim('/World/Puck',name='puck'));world.reset();view=robot._articulation_view;names=robot.dof_names;n=len(names);arm=[names.index('arm_joint'+str(i))for i in range(1,6)];fingers=[names.index('gripper_'+s+'_joint')for s in ['left','right']];ids=arm+fingers;cfg=json.loads((root/'configs/r12_gripper.json').read_text());zero=cfg['cad_zero_aperture_m'];q=robot.get_joint_positions().copy();q[arm]=qfixture;q[fingers]=[(.075-zero)/2,-(.075-zero)/2];robot.set_joint_positions(q);robot.set_joint_velocities(np.zeros(n));view.set_gains(kps=np.zeros((1,n)),kds=np.zeros((1,n)));caps=np.array([2.]*5+[10.]*2);eff=np.zeros((1,n));eff[0,ids]=caps;view.set_max_efforts(eff)
 save('runtime_scene_readback.json',dict(physics_dt=world.get_physics_dt(),scene_attributes={str(p.GetPath()):{str(a.GetName()):str(a.Get()) for a in p.GetAttributes()} for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)},implicit_kp=view.get_gains()[0].tolist(),implicit_kd=view.get_gains()[1].tolist(),fixed_base=bool(view._physics_view.shared_metatype.fixed_base),control_dof_count=n,solver_requested_override=None))
 save('pre_fixture_puck_state.json',dict(pose=[x.tolist() for x in puck.get_world_pose()],linear_velocity=puck.get_linear_velocity().tolist(),angular_velocity=puck.get_angular_velocity().tolist()))
 puck.set_world_pose(position=center,orientation=np.array([1.,0.,0.,0.]));puck.set_linear_velocity(np.zeros(3));puck.set_angular_velocity(np.zeros(3))
 M=np.asarray(view._physics_view.get_generalized_mass_matrices())[0];policy,meta=design(M,ids,n,bool(view._physics_view.shared_metatype.fixed_base));save('lqr_model.json',meta);ref=q.copy();save('initialization.json',dict(mode=mode,center= center.tolist(),q=q.tolist(),dof_names=names,frozen_base_joints=frozen,physics_hz=960,implicit_gains=view.get_gains()[0].tolist(),force_caps=view.get_max_efforts().tolist(),scope='Independent green-cuboid shortened release causal unit; no approach mission; fixtureGT setup and evaluator only; nativejointreadback as simulatedencoders'))
 save('collision_attributes.json',{str(p.GetPath()):{str(a.GetName()):str(a.Get()) for a in p.GetAttributes() if any(k in str(a.GetName()).lower() for k in ['offset','mass','inertia','approximation'])} for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI) or str(p.GetPath())=='/World/Puck'})
 callback_errors=[]
 def callback(headers,data):
  try:
   for h in headers:
    paths=[str(PhysicsSchemaTools.intToSdfPath(x))for x in (h.collider0,h.collider1)]
    if '/World/Puck' not in paths:continue
    impulse=sum(float(np.linalg.norm([data[j].impulse.x,data[j].impulse.y,data[j].impulse.z]))for j in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data))
    if phase in ['release','level_settle','near_support_lower','near_support_settle'] and impulse>0:
     com=np.asarray(puck.get_world_pose()[0],dtype=float)
     for j in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data):
      pt=data[j];xyz=np.array([pt.position.x,pt.position.y,pt.position.z]);iv=np.array([pt.impulse.x,pt.impulse.y,pt.impulse.z]);normal=[pt.normal.x,pt.normal.y,pt.normal.z]
      contact_points.append(dict(step=int(world.current_time_step_index),phase=phase,paths=paths,position_m=xyz.tolist(),impulse_vector_Ns=iv.tolist(),normal=normal,separation_m=float(pt.separation),object_center_evaluator=com.tolist(),raw_moment_impulse_Nms=np.cross(xyz-com,iv).tolist(),sign_convention='raw API; not yet normalized to force on object',evaluator_only=True))
    if impulse>0:contacts.append(dict(step=int(world.current_time_step_index),phase=phase,paths=paths,normal_impulse_Ns=impulse,mean_normal_force_N=impulse*960,evaluator_only=True))
  except Exception as e:callback_errors.append(repr(e))
 sub=omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(callback)
 placeq=np.array([0.,.15327346123403265,-.27269988318568805,.11942642195218953,0.])
 liftq=np.array([0.0, 0.1578675082058109, -0.10569091880967896, -0.052176615765720545, 0.0]);lift_steps=int(np.ceil(max(abs(liftq-qfixture))/.005*960));lift_end=3840+lift_steps;hold_end=lift_end+1920;lower_fast_steps=int(np.ceil(.8*lift_steps));lower_slow_steps=int(np.ceil(.2*lift_steps*5));lower_steps=lower_fast_steps+lower_slow_steps;lower_end=hold_end+lower_steps;support_end=lower_end+8640;near_end=support_end+20160;open_end=near_end+12480;stable_end=open_end+1920;retreat_end=stable_end+24000;return_end=retreat_end+19200;sep_steps=19200;sep_end=retreat_end;total=480 if mode=='smoke' else hold_end
 save('phase_schedule.json',dict(lift_end=lift_end,hold_end=hold_end,lower_end=lower_end,support_end=support_end,open_end=open_end,sep_end=sep_end,near_end=near_end,stable_end=stable_end,retreat_end=retreat_end,return_end=return_end,total=total,physics_hz=960))
 for i in range(total):
  phase='settle' if i<480 else ('close' if i<2880 else 'contact_hold');goalwidth=.075 if i<480 else .027;before=robot.get_joint_positions().copy();qd=robot.get_joint_velocities().copy();old=ref.copy()
  if i>=3840 and i<lift_end:phase='lift';ref[arm]=qfixture+min(1.,(i-3840+1)/lift_steps)*(liftq-qfixture)
  elif i>=lift_end and i<hold_end:phase='retention';ref[arm]=liftq
  elif i>=hold_end and i<lower_end:phase='lower';k=i-hold_end+1;alpha=.8*k/lower_fast_steps if k<=lower_fast_steps else .8+.2*(k-lower_fast_steps)/lower_slow_steps;ref[arm]=liftq+min(1.,alpha)*(placeq-liftq)
  elif i>=lower_end and i<support_end:
   if i==lower_end:
    measured=before[arm].astype(np.float64,copy=True);tool=tf(measured)['arm_link5'];target=(tool@TCP)[:3]
    def level_error(x):
     qtrial=measured.copy();qtrial[1:4]=x;z=tf(qtrial)['arm_link5'];return np.r_[(z@TCP)[[0,2]]-target[[0,2]],.15*z[2,0]]
    solution=least_squares(level_error,measured[1:4],bounds=(low[1:4],high[1:4]),gtol=1e-12,ftol=1e-12,xtol=1e-12);correction=np.zeros(5);correction[1:4]=solution.x-measured[1:4]
    save('encoder_level_solver.json',dict(residual=float(np.linalg.norm(solution.fun)),minimum_duration_s=float(max(abs(correction))/.005),input_dtype=str(measured.dtype)))
    if np.linalg.norm(solution.fun)>1e-6 or max(abs(correction))/.005>8:raise RuntimeError('encoder_level_IK_budget_gate')
    placeq=placeq+correction
    if any(placeq<low) or any(placeq>high):raise RuntimeError('encoder_level_joint_limit_gate')
    save('encoder_level_plan.json',dict(encoder=measured.tolist(),correction=correction.tolist(),corrected_reference=placeq.tolist(),target_tcp_from_encoder=target.tolist(),control_object_GT=False,reference_rate=.005))
   phase='encoder_level' if i<support_end-960 else 'level_settle';ref[arm]+=np.clip(placeq-ref[arm],-.005/960,.005/960)
  elif i>=support_end and i<near_end:
   if i==support_end:
    measured=before[arm].astype(np.float64,copy=True);target=(tf(measured)['arm_link5']@TCP)[:3];target[2]-=.002
    def near_error(x):
     qt=measured.copy();qt[1:4]=x;z=tf(qt)['arm_link5'];return np.r_[(z@TCP)[[0,2]]-target[[0,2]],.15*z[2,0]]
    sol=least_squares(near_error,measured[1:4],bounds=(low[1:4],high[1:4]),gtol=1e-12,ftol=1e-12,xtol=1e-12);correction=np.zeros(5);correction[1:4]=sol.x-measured[1:4]
    save('near_support_plan.json',dict(encoder=measured.tolist(),correction=correction.tolist(),target_from_encoder=target.tolist(),fixed_excitation_down_m=.002,control_object_GT=False,reference_rate=.001,residual=float(np.linalg.norm(sol.fun))))
    if np.linalg.norm(sol.fun)>1e-6 or max(abs(correction))/.001>20:raise RuntimeError('near_support_IK_budget_gate')
    placeq+=correction
    if any(placeq<low)or any(placeq>high):raise RuntimeError('near_support_joint_limit_gate')
   phase='near_support_lower' if i<near_end-960 else 'near_support_settle';ref[arm]+=np.clip(placeq-ref[arm],-.001/960,.001/960)
  elif i>=near_end and i<open_end:phase='release';ref[arm]=placeq;goalwidth=.075
  elif i>=open_end and i<stable_end:phase='release_stable_hold';goalwidth=.075;ref[arm]=placeq
  elif i>=stable_end and i<retreat_end:
   if i==stable_end:
    measured=before[arm].astype(np.float64,copy=True);target=(tf(measured)['arm_link5']@TCP)[:3];target[2]+=.010
    def retreat_error(x):
     qt=measured.copy();qt[1:4]=x;z=tf(qt)['arm_link5'];return np.r_[(z@TCP)[[0,2]]-target[[0,2]],.15*z[2,0]]
    sol=least_squares(retreat_error,measured[1:4],bounds=(low[1:4],high[1:4]),gtol=1e-12,ftol=1e-12,xtol=1e-12);delta=np.zeros(5);delta[1:4]=sol.x-measured[1:4];retreat_start=ref[arm].astype(np.float64,copy=True);retreat_goal=retreat_start+delta;retreat_steps=max(1,int(np.ceil(max(abs(delta))/.005*960)))
    save('retreat_plan.json',dict(encoder=measured.tolist(),delta=delta.tolist(),reference_goal=retreat_goal.tolist(),minimum_duration_s=retreat_steps/960,control_object_GT=False,residual=float(np.linalg.norm(sol.fun))))
    if np.linalg.norm(sol.fun)>1e-6 or retreat_steps>24000:raise RuntimeError('retreat_IK_budget_gate')
    if any(retreat_goal<low)or any(retreat_goal>high):raise RuntimeError('retreat_joint_limit_gate')
   phase='tcp_retreat';goalwidth=.075;ref[arm]=retreat_start+min(1.,(i-stable_end+1)/retreat_steps)*(retreat_goal-retreat_start)
  elif i>=retreat_end and i<return_end:
   if i==retreat_end:
    return_start=ref[arm].astype(np.float64,copy=True);return_steps=max(1,int(np.ceil(max(abs(qfixture-return_start))/.005*960)));save('return_plan.json',dict(goal=qfixture.tolist(),scope='return initial unit fixture configuration, not travel stow',minimum_duration_s=return_steps/960))
    if return_steps>19200:raise RuntimeError('return_budget_gate')
   phase='arm_return_initial_fixture';goalwidth=.075;ref[arm]=return_start+min(1.,(i-retreat_end+1)/return_steps)*(qfixture-return_start)
  elif i>=return_end:phase='placement_hold';goalwidth=.075;ref[arm]=qfixture
  goal=np.array([(goalwidth-zero)/2,-(goalwidth-zero)/2]);finger_rate=.002 if phase=='release' else .01;ref[fingers]+=np.clip(goal-ref[fingers],-finger_rate/960,finger_rate/960)
  rv=(ref-old)*960;grav=np.asarray(view._physics_view.get_gravity_compensation_forces())[0][np.array(ids)+meta['offset']];error=np.r_[before[ids]-ref[ids],qd[ids]-rv[ids]];command=policy.command(error,equilibrium_input=grav,lower=-caps,upper=caps);effort=np.zeros(n);effort[ids]=command;robot.apply_action(ArticulationAction(joint_efforts=effort));world.step(render=False);after=robot.get_joint_positions().copy();vel=robot.get_joint_velocities().copy();fd=(after-before)*960
  save('latest_step.json',dict(step=i+1,phase=phase,q=after.tolist(),qd=vel.tolist(),fd=fd.tolist(),puck_pose=[x.tolist() for x in puck.get_world_pose()]))
  if callback_errors:raise RuntimeError('contact_callback_error')
  if np.max(abs(vel[arm]))>=.02 or np.max(abs(fd[arm]))>=.02:raise RuntimeError('arm_speed_gate')
  if np.max(abs(after[arm]-ref[arm]))>.025:raise RuntimeError('arm_tracking_gate')
  if np.max(abs(vel[fingers]))>.04:raise RuntimeError('finger_speed_gate')
  if any(c['step']==int(world.current_time_step_index)and not any(x.endswith('/pad_collision')or x=='/World/Support'for x in c['paths'])for c in contacts[-15:]):raise RuntimeError('unexpected_nonpad_puck_contact')
  pos,quat=puck.get_world_pose()
  if i%96==0:
   _,items=boxes_at(after[arm],float(zero+after[fingers[0]]-after[fingers[1]]));oc=pos-np.array([0,0,.008]);orr=R.from_quat(np.r_[quat[1:],quat[0]]).as_matrix();ob=(oc,orr,np.array([.03,.03,.06]));sp=(np.r_[center[:2],(height/2)-.008],np.eye(3),np.array([.03,.03,height]))
   for name,body,box,*_ in items:
    if overlap(box,sp,.03) and distance(box,sp)<.03-1e-7:raise RuntimeError('environment_3cm_evaluator_gate')
    if not name.endswith('/pad_collision') and overlap(box,ob,.03) and cylinder_distance(box,oc,orr)[0]<.03-1e-7:raise RuntimeError('nonpad_object_3cm_evaluator_gate')
  rows.append(dict(step=i+1,phase=phase,arm_q=after[arm].tolist(),arm_qd=vel[arm].tolist(),arm_fd=fd[arm].tolist(),aperture_m=float(zero+after[fingers[0]]-after[fingers[1]]),command_force_N=command[5:].tolist(),arm_torque_Nm=command[:5].tolist(),puck_position_evaluator=pos.tolist(),puck_quat_evaluator=quat.tolist(),puck_linear_velocity_evaluator=puck.get_linear_velocity().tolist(),puck_angular_velocity_evaluator=puck.get_angular_velocity().tolist()))
  if i%96==0:save('job_status.json',dict(updated_at=time.time(),stage=phase,training=None,progress=dict(completed=i+1,total=total),issue=None))
 save('result.json',dict(completed=True,mode=mode,steps=len(rows),contact_event_count=len(contacts),left_contact=any(any('gripper_left/pad_collision'in x for x in c['paths'])for c in contacts),right_contact=any(any('gripper_right/pad_collision'in x for x in c['paths'])for c in contacts),physical_grasp_verified=False,lift_attempted=mode!='smoke',placement_attempted=False,mission_success=False))
except BaseException as e:
 fault=repr(e);save('failure.json',dict(error=fault,phase=phase,steps=len(rows)));traceback.print_exc()
finally:
 save('per_step.json',rows);save('contacts.json',contacts);save('contact_points.json',contact_points);save('before_close.json',dict(fault=fault))
 try:
  import gc
  sub=None
  gc.collect()
  if 'world' in globals():
   world.stop()
   world.clear()
  for owned_name in ('view','robot','puck','M','grav','command','policy'):
   globals().pop(owned_name,None)
  if 'World' in globals():World.clear_instance()
  globals().pop('world',None)
  gc.collect()
  save('owned_physics_released.json',dict(released=True,phase='before_app_close'))
 except BaseException as cleanup_error:
  fault=fault or repr(cleanup_error)
  save('cleanup_failure.json',dict(error=repr(cleanup_error)))
  traceback.print_exc()
 app.close();save('after_close.json',dict(clean_return=True,fault=fault))
if fault:raise SystemExit(2)
