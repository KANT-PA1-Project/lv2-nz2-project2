"""Native-resolution stationary optical calibration; no joint/base commands."""
from pathlib import Path
import os,sys,json,time,hashlib,traceback
import numpy as np,torch
from scipy.spatial.transform import Rotation
root=Path(__file__).resolve().parent;out=Path(os.environ['FLEET_RUN_DIR']);manifest=json.loads((root/'run_manifest.json').read_text());variant=manifest['variant']
sys.path[:0]=[str(root/'src'),str(root/'scripts/sensor_runtime')];np.random.seed(manifest['seed']);torch.manual_seed(manifest['seed'])
def save(name,obj):
 p=out/name;t=p.with_suffix(p.suffix+'.tmp');t.write_text(json.dumps(obj,indent=2,allow_nan=False)+'\n');t.replace(p)
def status(stage,n,state='running'):
 save('job_status.json',dict(updated_at=time.time(),run_id=os.environ['FLEET_RUN_ID'],host='pa24',pid=os.getpid(),status=state,stage=stage,phase=stage,progress=dict(current=n,total=6,unit='native stationary RGBD and analytic-plane frames'),execution_kind='fixed_USD_FK_native640_optical_axis_comparison',next_action='Compare original-origin wrist axis with original-CAD 30 degree down candidate; no motion authority',motion_authorized=False,isaac_started=True,ros_started=False))
status('Isaac_startup_fixed_fixture_'+variant,0)
source_hashes={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest()for p in [root/'render_axis.py',root/'run_manifest.json',root/'models/r12/r12_arm.urdf',root/'configs/sensors/camera_rig.json',root/'cad_geometry.json']}
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'active_gpu':0,'physics_gpu':0,'multi_gpu':False,'renderer':'RaytracedLighting','anti_aliasing':0,'width':640,'height':480})
import carb,omni.kit.app
from renderer_event_trace import RendererTrace
trace=RendererTrace(out,carb.settings.get_settings(),omni.kit.app.SettingChangeSubscription)
trace.mark('after_SimulationApp')
try:
 from isaacsim.core.api import World
 from isaacsim.core.utils.stage import add_reference_to_stage
 from pxr import Sdf,Usd,UsdGeom,UsdLux,Gf,UsdPhysics,PhysxSchema
 import omni.usd,omni.replicator.core as rep
 from arm_practice.kinematics import forward_transform
 from camera_rig import transforms
 world=World(stage_units_in_meters=1.,physics_dt=1/960,rendering_dt=1/20)
 stage=omni.usd.get_context().get_stage()
 trace.mark('after_World',stage)
 spawn=UsdGeom.Xform.Define(stage,'/World/Spawn');pose=manifest['sensor_derived_spawn_pose'];spawn.AddTranslateOp().Set(Gf.Vec3d(pose[0],pose[1],.008));spawn.AddRotateZOp().Set(float(np.rad2deg(pose[2])))
 add_reference_to_stage(str(root/'models/r12/r12_mobile_gripper.usda'),'/World/Spawn/R12');UsdLux.DomeLight.Define(stage,'/World/Light').CreateIntensityAttr(1800)
 from pxr import PhysicsSchemaTools
 import omni.physx
 from isaacsim.core.prims import SingleArticulation
 from isaacsim.core.utils.types import ArticulationAction
 from arm_gripper_lqr import design
 for prim in stage.Traverse():
  if prim.IsA(UsdPhysics.Joint) and ('wheel_joint' in prim.GetName() or 'steer_joint' in prim.GetName()):
   prim.SetTypeName('PhysicsFixedJoint');prim.RemoveAPI(UsdPhysics.DriveAPI,'angular')
 anchor=UsdPhysics.FixedJoint.Define(stage,'/World/Anchor');anchor.CreateBody1Rel().SetTargets(['/World/Spawn/R12/base_link']);anchor.CreateLocalPos0Attr().Set(Gf.Vec3f(0,0,.008))
 geom=json.loads((root/'cad_geometry.json').read_text());values={f'arm_joint{i+1}':x for i,x in enumerate(manifest['q'])};values.update(geom['gripper_open_positions'])
 base_prim=stage.GetPrimAtPath('/World/Spawn/R12/base_link')
 rig=transforms(root,values);O=np.diag([1.,-1.,-1.,1.]);ann={};products=[];cams={};reference_ann={}
 # Cameras live under a separate root so hiding robot geometry for the
 # explicit plane diagnostic cannot hide the sensors themselves.
 cache=UsdGeom.XformCache();B=np.array(cache.GetLocalToWorldTransform(base_prim)).T
 from arm_practice.native_rgbd.mixed_camera_schema import MixedCameraSchema
 from arm_practice.native_rgbd.renderer_epoch_issuer import RendererEpochIssuer
 from arm_practice.native_rgbd.mission_capture import NativeMissionCapture
 schema=MixedCameraSchema.from_file(root/'configs/mixed_camera_schema.json');rgb_ann={};depth_ann={};rgb_ref={};depth_ref={}
 for name,spec in schema.config['cameras'].items():
  T=B@rig[name];cam=UsdGeom.Camera.Define(stage,'/World/AuditSensors/'+name)
  cam.AddTransformOp().Set(Gf.Matrix4d(*(T@O).T.flatten().tolist()));cam.CreateHorizontalApertureAttr(spec['horizontal_aperture_mm']);cam.CreateVerticalApertureAttr(spec['vertical_aperture_mm']);cam.CreateFocalLengthAttr(spec['focal_length_mm']);cam.CreateClippingRangeAttr(Gf.Vec2f(*spec['clipping_range_m']));cams[name]=cam
  rp=rep.create.render_product(str(cam.GetPath()),tuple(spec['resolution_wh']));products.append(rp)
  rgb=rep.annotators.get('rgb');rgb.attach(rp);ref_time=rep.annotators.get('ReferenceTime');ref_time.attach(rp);rgb_ann[name]=rgb;rgb_ref[name]=ref_time
  for depth_name,ds in schema.config['depth'].items():
   if ds['rgb_camera']==name:
    dep=rep.annotators.get('distance_to_image_plane');dep.attach(rp);depth_ann[depth_name]=dep;depth_ref[depth_name]=ref_time

 # New standalone bench scene: import only appearance materials and target subtree.
 scene_file=root/'models/navigation/mission_scene.usda';layer=Sdf.Layer.FindOrOpen(str(scene_file));Sdf.CopySpec(layer,'/World/FixtureMaterials',stage.GetRootLayer(),'/World/FixtureMaterials');Sdf.CopySpec(layer,'/World/Pucks/Puck0',stage.GetRootLayer(),'/World/VisibilityTarget')
 for prim in stage.Traverse():
  if str(prim.GetPath()).startswith('/World/FixtureMaterials'):
   for attr in prim.GetAttributes():
    value=attr.Get()
    if isinstance(value,Sdf.AssetPath) and value.path and not Path(value.path).is_absolute():attr.Set(Sdf.AssetPath(str(scene_file.parent/value.path)))
 target=UsdGeom.Xform(stage.GetPrimAtPath('/World/VisibilityTarget'));target.ClearXformOpOrder();target.AddTranslateOp().Set(Gf.Vec3d(*manifest['fixed_target_position_world_m']));UsdGeom.Imageable(target.GetPrim()).MakeVisible()
 assert not stage.GetPrimAtPath('/World/Mission') and not stage.GetPrimAtPath('/World/Maze')
 # Only the authored visibility target is static. Robot remains native articulation.
 for prim in stage.Traverse():
  if not str(prim.GetPath()).startswith('/World/Spawn/R12') and prim.HasAPI(UsdPhysics.RigidBodyAPI):UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Set(False)
 contact_events=[]
 for prim in stage.Traverse():
  if prim.HasAPI(UsdPhysics.RigidBodyAPI) or prim.HasAPI(UsdPhysics.CollisionAPI):PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
 def oncontact(headers,data):
  for h in headers:
   paths=[str(PhysicsSchemaTools.intToSdfPath(x))for x in [h.collider0,h.collider1]]
   pts=[dict(position=[float(data[j].position.x),float(data[j].position.y),float(data[j].position.z)],separation=float(data[j].separation),impulse=[float(data[j].impulse.x),float(data[j].impulse.y),float(data[j].impulse.z)])for j in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data)]
   contact_events.append(dict(paths=paths,points=pts))
 sub=omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(oncontact)
 trace.mark('before_world_reset',stage)
 robot=world.scene.add(SingleArticulation('/World/Spawn/R12/base_link',name='arm'));world.reset();view=robot._articulation_view;names=robot.dof_names;n=len(names);arm=[names.index('arm_joint'+str(i))for i in range(1,6)];fingers=[names.index('gripper_'+x+'_joint')for x in ['left','right']];ids=arm+fingers
 q=robot.get_joint_positions().copy();q[arm]=manifest['q'];q[fingers]=[geom['gripper_open_positions'][names[j]]for j in fingers];robot.set_joint_positions(q);robot.set_joint_velocities(np.zeros(n));view.set_gains(kps=np.zeros((1,n)),kds=np.zeros((1,n)));caps=np.array([2.]*5+[10.]*2);eff=np.zeros((1,n));eff[0,ids]=caps;view.set_max_efforts(eff);ref=q.copy();M=np.asarray(view._physics_view.get_generalized_mass_matrices())[0];policy,meta=design(M,ids,n,bool(view._physics_view.shared_metatype.fixed_base));save('lqr_model.json',meta)
 trace.mark('after_world_reset',stage,world.current_time_step_index)
 retained={k:carb.settings.get_settings().get(k)for k in ['/rtx/rendermode','/rtx/post/dlss/execMode']}
 carb.settings.get_settings().set_bool('/rtx-transient/post/aa/limitedOps',False)
 carb.settings.get_settings().set_int('/rtx/post/aa/op',0)
 trace.mark('after_aa_off',stage,world.current_time_step_index)
 save('initialization.json',dict(q=q.tolist(),names=names,scope='Explicit initial fixture state at CPU checked pose; no traversal executed',fixed_base=True,physics_hz=960,torque_caps=caps.tolist(),initialization_setters_once=True))
 def fk_measured(qdict):
  rt=transforms(root,qdict);return {name:(B@rt[name]).tolist()for name in schema.config['cameras']}
 issuer=RendererEpochIssuer(run_id=os.environ['FLEET_RUN_ID'],clock_epoch=os.environ['FLEET_RUN_ID']+'_bench',cameras=list(rgb_ann),joint_names=[f'arm_joint{i}'for i in range(1,6)],physics_hz=960,model_sha256=hashlib.sha256((root/'models/r12/r12_arm.urdf').read_bytes()).hexdigest(),source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),fk=fk_measured,depth_camera_map={k:v['rgb_camera']for k,v in schema.config['depth'].items()},history_size=2048,camera_schema=schema)
 producer=NativeMissionCapture(issuer=issuer,rgb=rgb_ann,depth=depth_ann,rgb_reference=rgb_ref,depth_reference=depth_ref,read_epoch=lambda:(int(world.current_time_step_index),float(world.current_time)),read_joint_positions=robot.get_joint_positions,arm_indices=arm,joint_names=[f'arm_joint{i}'for i in range(1,6)],render=world.render)
 ticks=[]
 def tick():
  before=robot.get_joint_positions().copy();qd=robot.get_joint_velocities().copy();grav=np.asarray(view._physics_view.get_gravity_compensation_forces())[0][np.array(ids)+meta['offset']];command=policy.command(np.r_[before[ids]-ref[ids],qd[ids]],equilibrium_input=grav,lower=-caps,upper=caps);effort=np.zeros(n);effort[ids]=command;robot.apply_action(ArticulationAction(joint_efforts=effort));world.step(render=False);after=robot.get_joint_positions().copy();vel=robot.get_joint_velocities().copy();fd=(after-before)*960
  ticks.append(dict(step=int(world.current_time_step_index),time=float(world.current_time),q=after.tolist(),qd=vel.tolist(),fd=fd.tolist(),command=command.tolist(),target=ref.tolist()))
  producer.record_physics_step(native_qd=vel[arm].tolist(),same_step_fd=fd[arm].tolist())
 inventory=[dict(path=str(p.GetPath()),type=p.GetTypeName(),enabled=bool(UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get()))for p in stage.Traverse()if p.HasAPI(UsdPhysics.CollisionAPI)]
 save('active_collider_inventory.json',dict(scope='Standalone fixed-base bench; no mission scene or ground plane imported',colliders=inventory))
 for i in range(500):
  tick()
  save('physics_ticks.json',ticks);save('first_step_contacts.json',contact_events)
  bad_contacts=[x for x in contact_events if any(pt['separation']<-.00001 for pt in x['points'])]
  if bad_contacts:raise RuntimeError('Native initial/contact penetration; reject bench setup')
  if max(abs(np.array(ticks[-1]['qd'])[arm]))>=.02 or max(abs(np.array(ticks[-1]['fd'])[arm]))>=.02:raise RuntimeError('Immediate native/FD .02 velocity abort')
  if i==0:save('one_step_admission.json',dict(pass_native_velocity=True,no_recorded_penetration=True,scope='One tick only, not hold acceptance'))
  if i%48==0:
   trace.mark('before_hold_render_'+str(i),stage,world.current_time_step_index)
   world.render()
   trace.mark('after_hold_render_'+str(i),stage,world.current_time_step_index)
 save('physics_ticks.json',ticks)
 save('first_step_contacts.json',contact_events)
 strict=len(ticks)>=480 and all(max(abs(np.array(t['qd'])[arm]))<.02 and max(abs(np.array(t['fd'])[arm]))<.02 for t in ticks[-480:])
 save('settle_gate.json',dict(last480_all_strict=strict,window_s=.5,threshold_rad_s=.02,qd_all_max=float(max(max(abs(np.array(t['qd'])[arm]))for t in ticks)),fd_all_max=float(max(max(abs(np.array(t['fd'])[arm]))for t in ticks)),torque_arm_max=float(max(max(abs(np.array(t['command'])[:5]))for t in ticks)),motion_authorized=False))
 if not strict:raise RuntimeError('Native readback and same-tick FD strict .5s gate failed')
 rows=[]
 for seq in range(3):
  if seq:
   for _ in range(48):
    tick()
    if max(abs(np.array(ticks[-1]['qd'])[arm]))>=.02 or max(abs(np.array(ticks[-1]['fd'])[arm]))>=.02:raise RuntimeError('Capture interval velocity violation')
  measured=robot.get_joint_positions().copy();rt=transforms(root,{name:float(measured[j])for j,name in enumerate(names)})
  for name,cam in cams.items():cam.GetOrderedXformOps()[0].Set(Gf.Matrix4d(*(B@rt[name]@O).T.flatten().tolist()))
  before_q=robot.get_joint_positions().copy();before_step=int(world.current_time_step_index);result=producer.acquire()
  assert int(world.current_time_step_index)==before_step and np.array_equal(before_q,robot.get_joint_positions())
  evidence={k:v for k,v in result.items()if k not in ('pixels','depths')};evidence.update(frame=seq,native_step=before_step,native_q_unchanged=True)
  if result['captured']:
   arrays={**result['pixels'],**result['depths']};np.savez_compressed(out/f'rgbd_{seq:02d}.npz',**arrays)
   evidence['verification']=issuer.verify(result['certificate'],pixels=result['pixels'],depths=result['depths'],now_ns=time.monotonic_ns(),expected_current_run=os.environ['FLEET_RUN_ID'])
  rows.append(evidence);save('producer_capture_results.json',rows);save('encoder_history.json',dict(issuer.history));save('physics_ticks.json',ticks);save('first_step_contacts.json',contact_events)
  trace.mark('after_capture_'+str(seq),stage,world.current_time_step_index)
  assert carb.settings.get_settings().get('/rtx/post/aa/op')==0
 save('render_result.json',dict(rows=rows,source_hashes=source_hashes,scope='8RGB4depth producer/schema connection in independent anchored bench; no mission or calibration authority',motion_authorized=False,calibration_accepted=False))
 world.stop()
except BaseException:
 save('render_error.json',dict(traceback=traceback.format_exc()));raise
finally:app.close()
