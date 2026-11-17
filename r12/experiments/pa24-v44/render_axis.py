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
 for name in manifest['cameras']:
  T=B@rig[manifest['camera_optical_keys'][name]];cam=UsdGeom.Camera.Define(stage,'/World/AuditSensors/'+name)
  cam.AddTransformOp().Set(Gf.Matrix4d(*(T@O).T.flatten().tolist()));cam.CreateHorizontalApertureAttr(20.955);cam.CreateVerticalApertureAttr(15.71625);cam.CreateFocalLengthAttr(13.096875);cam.CreateClippingRangeAttr(Gf.Vec2f(.01,30));cams[name]=cam
  rp=rep.create.render_product(str(cam.GetPath()),(1280,960));products.append(rp);rgb=rep.AnnotatorRegistry.get_annotator('rgb');rgb.attach(rp);dep=rep.AnnotatorRegistry.get_annotator('distance_to_image_plane');dep.attach(rp);ann[name]=(rgb,dep)
  ref_time=rep.annotators.get('ReferenceTime');ref_time.attach(rp);reference_ann[name]=ref_time

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
 ticks=[]
 def tick():
  before=robot.get_joint_positions().copy();qd=robot.get_joint_velocities().copy();grav=np.asarray(view._physics_view.get_gravity_compensation_forces())[0][np.array(ids)+meta['offset']];command=policy.command(np.r_[before[ids]-ref[ids],qd[ids]],equilibrium_input=grav,lower=-caps,upper=caps);effort=np.zeros(n);effort[ids]=command;robot.apply_action(ArticulationAction(joint_efforts=effort));world.step(render=False);after=robot.get_joint_positions().copy();vel=robot.get_joint_velocities().copy();fd=(after-before)*960
  ticks.append(dict(step=int(world.current_time_step_index),time=float(world.current_time),q=after.tolist(),qd=vel.tolist(),fd=fd.tolist(),command=command.tolist(),target=ref.tolist()))
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
 reference_records=[];ready_reports=[]
 def ref_read():
  result={}
  for key,a in reference_ann.items():
   value=a.get_data()
   result[key]=json.loads(json.dumps(value,default=lambda x:x.tolist()if hasattr(x,'tolist')else str(x)))
  return result
 from PIL import Image
 for seq in range(20):
  if seq:
   for _ in range(48):
    tick()
    if max(abs(np.array(ticks[-1]['qd'])[arm]))>=.02 or max(abs(np.array(ticks[-1]['fd'])[arm]))>=.02:raise RuntimeError('Capture interval immediate .02 velocity abort')
  strict_window=ticks[-480:]
  epoch_gate=len(strict_window)==480 and all(max(abs(np.array(t['qd'])[arm]))<.02 and max(abs(np.array(t['fd'])[arm]))<.02 for t in strict_window)
  if not epoch_gate:raise RuntimeError('Per-capture .5s strict window failed')
  if any(pt['separation']<-.00001 for e in contact_events for pt in e['points']):raise RuntimeError('Capture interval penetration')
  save('physics_ticks.json',ticks);save('first_step_contacts.json',contact_events)
  ready_start_ns=time.monotonic_ns();measured=robot.get_joint_positions().copy();capture_step=int(world.current_time_step_index);capture_time=float(world.current_time);values={name:float(measured[j])for j,name in enumerate(names)};rig=transforms(root,values)
  for name,cam in cams.items():
   cam.GetOrderedXformOps()[0].Set(Gf.Matrix4d(*(B@rig[manifest['camera_optical_keys'][name]]@O).T.flatten().tolist()))
  trace.mark('before_capture_'+str(seq),stage,world.current_time_step_index)
  reference_before=ref_read()
  begin=time.monotonic_ns();attempt_records=[];epoch_ready=False
  from fractions import Fraction
  for attempt in range(4):
   if time.monotonic_ns()-ready_start_ns>200_000_000:break
   before_attempt=ref_read();world.render();after_attempt=ref_read()
   attempt_hashes={name:dict(rgb=hashlib.sha256(np.asarray(a[0].get_data()).tobytes()).hexdigest(),depth=hashlib.sha256(np.asarray(a[1].get_data()).tobytes()).hexdigest())for name,a in ann.items()}
   after_read=ref_read();now=time.monotonic_ns();q_after=robot.get_joint_positions().copy()
   aligned=[]
   for value in after_attempt.values():
    nref=value.get('referenceTimeNumerator');dref=value.get('referenceTimeDenominator')
    aligned.append(type(nref)is int and type(dref)is int and dref>0 and abs(Fraction(nref,dref)-Fraction(capture_step,960))<=Fraction(1,1_000_000_000))
   epoch_ready=all(aligned)and len(aligned)==5 and after_read==after_attempt and int(world.current_time_step_index)==capture_step and np.array_equal(measured,q_after)and now-ready_start_ns<=200_000_000
   attempt_records.append(dict(attempt=attempt+1,host_end_ns=now,wall_wait_ms=(now-ready_start_ns)/1e6,before_render=before_attempt,after_render=after_attempt,after_rgbd_hash_read=after_read,physics_step=int(world.current_time_step_index),q_unchanged=bool(np.array_equal(measured,q_after)),rgbd_hashes=attempt_hashes,bench_exact_epoch_ready=epoch_ready))
   if epoch_ready:break
  end=time.monotonic_ns();data={};per={}
  ready_reports.append(dict(frame=seq,capture_step=capture_step,encoder_read_start_ns=ready_start_ns,attempts=attempt_records,bench_exact_epoch_ready=epoch_ready,scope='5-camera bench diagnostic only, not mission 8RGB4depth ready certificate',motion_authorized=False))
  save('native_ready_diagnostics.json',ready_reports)
  trace.mark('after_capture_'+str(seq),stage,world.current_time_step_index)
  reference_after=ref_read()
  settings=carb.settings.get_settings()
  assert settings.get('/rtx/post/aa/op')==0 and settings.get('/rtx-transient/post/aa/limitedOps')==False
  assert all(settings.get(k)==v for k,v in retained.items())
  for prim in stage.Traverse():
   if prim.GetTypeName()=='RenderProduct':
    assert prim.GetAttribute('omni:rtx:post:aa:limitedOps').Get()==False
    assert str(prim.GetAttribute('omni:rtx:post:aa:op').Get()).lower()in ('off','none','disabled','0')
  for name,(rgb,dep)in ann.items():
   im=np.asarray(rgb.get_data())[...,:3].copy();z=np.asarray(dep.get_data()).copy()
   assert im.shape==(960,1280,3) and z.shape==(960,1280)
   data[name+'_rgb']=im;data[name+'_depth']=z
   Image.fromarray(im).save(out/f'{name}_{seq:02d}.png')
   per[name]=dict(rgb_sha256=hashlib.sha256(im.tobytes()).hexdigest(),depth_sha256=hashlib.sha256(z.tobytes()).hexdigest(),valid_depth_pixels=int((np.isfinite(z)&(z>0)).sum()),resolution=[1280,960],K=[800,800,639.5,479.5],T_world_camera=(B@rig[manifest['camera_optical_keys'][name]]).tolist())
  np.savez_compressed(out/f'rgbd_{seq:02d}.npz',**data)
  reference_records.append(dict(frame=seq,capture_step=capture_step,physics_time=capture_time,before_render=reference_before,after_render=reference_after,after_rgbd_read=ref_read(),rgbd_sha256=hashlib.sha256((out/f'rgbd_{seq:02d}.npz').read_bytes()).hexdigest(),scope='Renderer rational reference time only; not exposure time or wall clock'))
  save('reference_time_records.json',reference_records)
  rows.append(dict(frame=seq,render_interval_ns=[begin,end],physics_time=float(world.current_time),per_camera=per,fixture_initial_q=manifest['q'],encoder_measured=True,measured_q=measured.tolist(),capture_step=capture_step,capture_sim_time=capture_time,strict_window=dict(passed=epoch_gate,start_step=strict_window[0]['step'],end_step=strict_window[-1]['step'],ticks=480,threshold=.02),capture_FK_link5=forward_transform(root/'models/r12/r12_arm.urdf','arm_link5',values).tolist(),motion_authorized=False))
  save('job_status.json',dict(updated_at=time.time(),stage='static_five_camera_capture',progress=dict(completed=seq+1,total=20),training=None,issue=None))
 save('render_result.json',dict(rows=rows,source_hashes=source_hashes,scope='Native advancing fixed-base observation epochs with measured encoder FK and per-epoch strict window. Exact initialized equilibrium; no calibration or trajectory admission.',motion_authorized=False,calibration_accepted=False))
 world.stop()
except BaseException:
 save('render_error.json',dict(traceback=traceback.format_exc()));raise
finally:app.close()
