#!/usr/bin/env python3
"""Actual R12 mounted stereo + 2 physics IMUs + encoder packet source."""
import uuid
import argparse,json,socket,sys,time,os,subprocess,atexit,numpy as np
from scipy.spatial.transform import Rotation
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--frames',type=int,default=200);p.add_argument('--port',type=int,default=19612);args=p.parse_args()
def atomic_json(path,data):
 temporary=path.with_suffix(path.suffix+'.tmp');temporary.write_text(json.dumps(data,separators=(',',':')));temporary.replace(path)
capture_run_id=uuid.uuid4().hex
root=Path(__file__).resolve().parents[1];out=Path(os.environ['FLEET_RUN_DIR']) if os.getenv('R12_NATIVE_PRODUCER_STATIONARY','0')=='1' else root/'reports/local'/os.getenv('R12_SENSOR_RUN','sensors');out.mkdir(parents=True,exist_ok=True);sys.path.insert(0,str(root/'src'))
frame_file_path=Path(os.getenv('R12_FRAME_FILE',str(out/'frame.npz')))
from arm_practice.isaac_pixel_geometry import archive_geometry_fields
from arm_practice.perception_transport import JsonPipeReader, require_response, fresh_response, stop_worker
from arm_practice.motion_envelope import WheelSlew
from arm_practice.latest_policy import allocate,dc_effort
from arm_practice.stop_brake import stop_yaw_targets
from arm_practice.wheel_integral import WheelIntegral
from arm_practice.encoder_velocity import wrapped_velocity
from arm_practice.result_journal import ResultJournal
result_journal=ResultJournal(out/'sim_results.json') if os.getenv('R12_INCREMENTAL_RESULT_LOGS','0')=='1' else None
archive_clearance_depth=os.getenv('R12_ARCHIVE_CLEARANCE_DEPTH','0')=='1'
if archive_clearance_depth and os.getenv('R12_GPU_ACQUISITION','0')=='1':
 raise ValueError('Unmasked depth archives currently require CPU sensor readback')
from arm_practice.steering_transition import brake_before_large_turn,project_current_heading,coordinate_turn_with_drive
from isaacsim import SimulationApp
stationary_capture=os.getenv('R12_STATIONARY_CAPTURE','0')=='1'
native_producer_enabled=os.getenv('R12_NATIVE_PRODUCER_STATIONARY','0')=='1'
if native_producer_enabled:
 if not stationary_capture or os.getenv('R12_MISSION_PERCEPTION','1')!='0' or os.getenv('R12_GUI','0')!='0':raise ValueError('native producer currently requires stationary headless without legacy perception')
 if any(os.getenv(k,'0')=='1' for k in ('R12_CAPTURE_JOINT_EPOCH','R12_GPU_ACQUISITION','R12_RECORD_VIDEO','R12_SHARED_FRAME_DIAGNOSTIC','R12_SIM_COMMAND_DELAY_NS')):raise ValueError('native producer forbids mixed acquisition/legacy epoch paths')
 capture_run_id=Path(os.environ['FLEET_RUN_DIR']).name

# Diagnostic integration is deliberately limited to zero-command stationary trials.
command_delay=None;command_delay_rows=[]
if 'R12_SIM_COMMAND_DELAY_NS' in os.environ:
 if not stationary_capture:raise ValueError('Command-delay integration currently requires stationary capture')
 from arm_practice.sim_command_delay import SimCommandDelay
 command_delay=SimCommandDelay(capture_run_id,delay_ns=int(os.environ['R12_SIM_COMMAND_DELAY_NS']),max_age_ns=int(os.environ['R12_SIM_COMMAND_MAX_AGE_NS']))
 delay_submit_frames=int(os.getenv('R12_SIM_DELAY_SUBMIT_FRAMES','-1'))

if stationary_capture and any(os.getenv(k,'0')=='1' for k in ('R12_MISSION','R12_SENSOR_MOTION')):raise ValueError('Stationary capture forbids mission/motion')
gui=os.getenv('R12_GUI','0')=='1'
residual_scale=float(os.getenv('R12_PPO_RESIDUAL_SCALE','1'))
if not np.isfinite(residual_scale) or not 0<=residual_scale<=1:raise ValueError('Invalid PPO residual scale')
low_speed_residual_fraction=None if 'R12_PPO_LOW_SPEED_FRACTION' not in os.environ else float(os.environ['R12_PPO_LOW_SPEED_FRACTION'])
if low_speed_residual_fraction is not None and (not np.isfinite(low_speed_residual_fraction) or not 0<=low_speed_residual_fraction<1):raise ValueError('Invalid low-speed PPO residual fraction')
small_viewport=os.getenv('R12_SMALL_VIEWPORT_DIAGNOSTIC','0')=='1'
if small_viewport and not (stationary_capture and gui):raise ValueError('Small viewport requires stationary GUI diagnostic')
viewport_size=(640,360) if small_viewport else (1280,720)
from arm_practice.native_rgbd.renderer_startup_order import startup_options,validate as validate_startup_order
app=SimulationApp({**startup_options(native_producer_enabled and os.getenv('R12_NATIVE_STARTUP_ORDER')=='1'),'headless':not gui,'renderer':'RaytracedLighting','anti_aliasing':0,'width':viewport_size[0],'height':viewport_size[1],**({'active_gpu':0,'physics_gpu':0,'multi_gpu':False,'fast_shutdown':True} if native_producer_enabled else {})})
if native_producer_enabled and os.getenv('R12_NATIVE_STARTUP_ORDER')=='1':
 import carb
 from omni.syntheticdata import SyntheticData
 atomic_json(out/'renderer_startup_order.json',validate_startup_order(carb.settings.get_settings(),SyntheticData))
if small_viewport:
 import carb
 atomic_json(out/'viewport_configuration.json',{'requested':list(viewport_size),'renderer_width':carb.settings.get_settings().get('/app/renderer/resolution/width'),'renderer_height':carb.settings.get_settings().get('/app/renderer/resolution/height'),'sensor_resolution':[640,480],'gui':gui,'scope':'Viewport-only stationary diagnostic; camera render products remain explicitly 640x480'})
# Explicit renderer A/B option; does not change scene geometry or sensor count.
force_aa_off=os.getenv('R12_FORCE_AA_OFF','0')=='1'
def apply_renderer_override(phase):
 if not force_aa_off:return
 import carb
 settings=carb.settings.get_settings()
 before=settings.get('/rtx/post/aa/op')
 settings.set_int('/rtx/post/aa/op',0)
 with (out/'renderer_override.jsonl').open('a') as f:f.write(json.dumps({'phase':phase,'before_aa_op':before,'after_aa_op':settings.get('/rtx/post/aa/op'),'monotonic_ns':time.monotonic_ns()})+'\n')
apply_renderer_override('app_initialized')
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.extensions import enable_extension
enable_extension('isaacsim.sensors.physics');app.update()
hydra_profile_rows=[];hydra_stats=None
render_settings_audit=os.getenv('R12_RENDER_SETTINGS_AUDIT','0')=='1'
if render_settings_audit and not stationary_capture:raise ValueError('Render settings audit requires stationary capture')
def audit_render_settings(phase):
 if not render_settings_audit:return
 import carb
 settings=carb.settings.get_settings()
 keys=['/rtx/rendermode','/rtx/post/aa/op','/rtx/post/dlss/execMode','/rtx-defaults/rendermode','/rtx-defaults/post/aa/op','/rtx-transient/dlssg/enabled','/app/renderer/resolution/width','/app/renderer/resolution/height']
 with (out/'effective_render_settings.jsonl').open('a') as f:f.write(json.dumps({'phase':phase,'monotonic_ns':time.monotonic_ns(),'values':{k:settings.get(k) for k in keys}})+'\n')
audit_render_settings('app_initialized')

if os.getenv('R12_HYDRA_PROFILE','0')=='1':
 if not (stationary_capture and gui):raise ValueError('Hydra profiling requires stationary GUI')
 enable_extension('omni.hydra.engine.stats');app.update()
 from omni.hydra.engine.stats import HydraEngineStats
 import carb
 profiler_settings=carb.settings.get_settings()
 profiler_before=profiler_settings.get('/profiler/enabled')
 hydra_delayed=os.getenv('R12_HYDRA_PROFILE_AFTER_WARMUP','0')=='1'
 if hydra_delayed:profiler_settings.set_bool('/profiler/enabled',False)
 elif os.getenv('R12_HYDRA_PROFILE_ENABLE','0')=='1':profiler_settings.set_bool('/profiler/enabled',True)
 atomic_json(out/'hydra_profile_configuration.json',{'enabled_before':profiler_before,'enabled_after':profiler_settings.get('/profiler/enabled'),'explicit_enable':os.getenv('R12_HYDRA_PROFILE_ENABLE','0')=='1'})
 hydra_stats=HydraEngineStats()

import isaacsim.sensors.physics as physics_sensors
from isaacsim.sensors.physics.bindings import _sensor
physics_sensors._sensor=_sensor
from isaacsim.sensors.physics.impl.imu_sensor import IMUSensor
from pxr import UsdGeom,UsdLux,UsdShade,Gf,UsdPhysics,PhysxSchema,PhysicsSchemaTools
import omni.usd,omni.replicator.core as rep
from arm_practice.kinematics import forward_transform
from arm_practice.swerve import SwervePolicy,actuate
import torch
net=SwervePolicy();net.load_state_dict(torch.load(root/'models/swerve/policy.pt',weights_only=True));net.eval();torch.set_num_threads(2)
physics_hz=int(os.getenv('R12_PHYSICS_HZ','120'))
if physics_hz not in [120,240,480,960,1920]:raise ValueError('Unsupported physics frequency')
imu_stride=physics_hz//120
world=World(stage_units_in_meters=1.,physics_dt=1/physics_hz,rendering_dt=1/20)
add_reference_to_stage(str(root/'models/navigation/mission_scene.usda'),'/World/Mission');world.scene.add_default_ground_plane()
stage=omni.usd.get_context().get_stage();parent=UsdGeom.Xform.Define(stage,'/World/Spawn');parent.AddTranslateOp().Set(Gf.Vec3d(.734,.950,.008));parent.AddRotateZOp().Set(180.)
explicit_base_root=os.getenv('R12_EXPLICIT_BASE_ROOT','0')=='1'
robot_asset='r12_mobile_rl.usda' if explicit_base_root else 'r12_mobile_gripper.usda'
articulation_path='/World/Spawn/R12/base_link' if explicit_base_root else '/World/Spawn/R12'
add_reference_to_stage(str(root/'models/r12'/robot_asset),'/World/Spawn/R12')
# Initialization-only world support, no runtime pose projection or gain changes.
anchor=UsdPhysics.FixedJoint.Define(stage,'/World/v123_external_anchor')
anchor.CreateBody1Rel().SetTargets(['/World/Spawn/R12/base_link'])
anchor_world=UsdGeom.Xformable(stage.GetPrimAtPath('/World/Spawn/R12/base_link')).ComputeLocalToWorldTransform(0)
anchor_rotation=anchor_world.ExtractRotationQuat()
anchor.CreateLocalPos0Attr().Set(Gf.Vec3f(anchor_world.ExtractTranslation()))
anchor.CreateLocalRot0Attr().Set(Gf.Quatf(anchor_rotation.GetReal(),Gf.Vec3f(anchor_rotation.GetImaginary())))
assert not anchor.GetBody0Rel().GetTargets() and len(anchor.GetBody1Rel().GetTargets())==1
atomic_json(out/'fixed_base_fixture.json',dict(path=str(anchor.GetPath()),body0=[],body1=[str(x)for x in anchor.GetBody1Rel().GetTargets()],world_position=list(anchor_world.ExtractTranslation()),scope='Initialization-only external support; changes base/contact context; not mission configuration',motion_authorized=False))

diagnostic_inertia_path=os.getenv('R12_DIAGNOSTIC_INERTIA_PROFILE','')
diagnostic_inertia=None
if diagnostic_inertia_path:
 diagnostic_inertia=json.loads(Path(diagnostic_inertia_path).read_text())
 if set(diagnostic_inertia['wheels'])!={n+'_wheel' for n in ['lh1','rh1','lh2','rh2']}:raise ValueError('Expected four wheel inertials')
 for name,data in diagnostic_inertia['wheels'].items():
  prim=stage.GetPrimAtPath('/World/Spawn/R12/'+name);tensor=np.asarray(data['inertia_tensor'],float);center=np.asarray(data['center_of_mass'],float)
  if not prim.IsValid() or tensor.shape!=(3,3) or center.shape!=(3,) or not np.isfinite(tensor).all() or not np.isfinite(center).all() or not np.isfinite(data['mass']) or data['mass']<=0 or not np.allclose(tensor,tensor.T):raise ValueError('Invalid diagnostic inertial')
  vals,axes=np.linalg.eigh(tensor)
  if min(vals)<=0:raise ValueError('Nonpositive inertia')
  if np.linalg.det(axes)<0:axes[:,0]*=-1
  quat=Rotation.from_matrix(axes).as_quat();mass=UsdPhysics.MassAPI(prim);mass.GetMassAttr().Set(data['mass']);mass.GetCenterOfMassAttr().Set(Gf.Vec3f(*center));mass.GetDiagonalInertiaAttr().Set(Gf.Vec3f(*vals));mass.GetPrincipalAxesAttr().Set(Gf.Quatf(float(quat[3]),Gf.Vec3f(*quat[:3])))
atomic_json(out/'diagnostic_dynamics.json',{'physics_hz':physics_hz,'inertia_profile':diagnostic_inertia,'scope':'Simulation diagnostic assumptions only; not hardware certification'})
UsdLux.DomeLight.Define(stage,'/World/Light').CreateIntensityAttr(1800.)
robot=world.scene.add(SingleArticulation(articulation_path,name='robot'))
imus=[]
for i,x in enumerate([.057,-.057]):
 imus.append(world.scene.add(IMUSensor(f'/World/Spawn/R12/base_link/opencr_{i+1}',name=f'imu{i}',frequency=120,translation=np.array([x,0,.16597]),orientation=np.array([1,0,0,0]) if i==0 else np.array([0,0,0,1]))))
sys.path.insert(0,str(root/'scripts/sensor_runtime'))
annotator_no_copy=os.getenv('R12_ANNOTATOR_NO_COPY','0')=='1'
annotator_device=os.getenv('R12_ANNOTATOR_DEVICE','cpu')
if annotator_device!='cpu' and not stationary_capture:raise ValueError('CUDA annotators require stationary diagnostic')
if annotator_no_copy and not stationary_capture:raise ValueError('No-copy annotator experiment requires stationary capture')
from camera_rig import transforms as camera_transforms,PreparedCameraRig
prepared_camera_rig=PreparedCameraRig(root) if os.getenv('R12_PREPARED_CAMERA_RIG','0')=='1' else None
from imu_window import reduce_window
rig_home=camera_transforms(root,{f'arm_joint{i}':0 for i in range(1,6)})
from arm_practice.native_rgbd.puck_fixture import build as build_puck_fixture
atomic_json(out/'fixture_setup_evaluator_only.json',build_puck_fixture(stage,rig_home['arm_camera_left_optical']))
sys.path.insert(0,str(root))
from evaluator_capture import EvaluatorCapture
evaluator_only=EvaluatorCapture(world,stage,robot,out)
# Preserve the complete calibration used by this run; legacy extrinsics.json
# contains only camera1. Wrist home transforms are not live encoder evidence.
import hashlib
atomic_json(out/'camera_rig_calibration.json',{
 'scope':'run calibration only; not live image/encoder association',
 'home_joint_positions':{f'arm_joint{i}':0. for i in range(1,6)},
 'home_transforms':{name:T.tolist() for name,T in rig_home.items()},
 'fixed_upper_camera_names':[name for name in rig_home if name.startswith('r07_')],
 'source_sha256':{str(path.relative_to(root)):hashlib.sha256(path.read_bytes()).hexdigest()
  for path in [root/'models/r12/r12_arm.urdf',root/'configs/sensors/camera_rig.json',root/'scripts/sensor_runtime/camera_rig.py']},
 'motion_authorized':False})
if native_producer_enabled:
 from arm_practice.native_rgbd.mixed_camera_schema import MixedCameraSchema
 native_schema=MixedCameraSchema.from_file(root/'configs/mixed_camera_schema.json')
 native_rgb_refs={};native_depth_refs={};native_optics={}
def native_attach(frame,camera,rp,is_left):
 if not native_producer_enabled:return
 ref=rep.AnnotatorRegistry.get_annotator('ReferenceTime');ref.attach(rp);native_rgb_refs[frame]=ref
 if is_left:
  ref=rep.AnnotatorRegistry.get_annotator('ReferenceTime');ref.attach(rp);native_depth_refs[frame.replace('_left_optical','_depth')]=ref
 native_optics[frame]={'resolution_wh':[int(x) for x in stage.GetPrimAtPath(rp.path).GetAttribute('resolution').Get()], 'clipping_range_m':[float(x) for x in camera.GetClippingRangeAttr().Get()], 'focal_length_mm':float(camera.GetFocalLengthAttr().Get()), 'horizontal_aperture_mm':float(camera.GetHorizontalApertureAttr().Get()), 'vertical_aperture_mm':float(camera.GetVerticalApertureAttr().Get())}
anns=[];segments=[];extrinsics={}
for side in ['left','right']:
 frame=f'r07_camera1_{side}_optical';T=forward_transform(root/'models/r12/r12_arm.urdf',frame,{f'arm_joint{i}':0 for i in range(1,6)});extrinsics[side]=T.tolist()
 optical_to_usd=np.diag([1.,-1.,-1.,1.]);camera=UsdGeom.Camera.Define(stage,f'/World/Spawn/R12/base_link/sensor_{side}')
 camera.AddTransformOp().Set(Gf.Matrix4d(*(T@optical_to_usd).T.flatten().tolist()));camera.CreateHorizontalApertureAttr(20.955);camera.CreateVerticalApertureAttr(20.955*3/4);camera.CreateFocalLengthAttr(20.955*.625);camera.CreateClippingRangeAttr(Gf.Vec2f(.01,30.))
 rp=rep.create.render_product(str(camera.GetPath()),(640,480));rgb=rep.AnnotatorRegistry.get_annotator('rgb',do_array_copy=not annotator_no_copy,device=annotator_device);rgb.attach(rp);anns.append(rgb)
 native_attach(frame,camera,rp,side=='left')
 if not native_producer_enabled:
  seg=rep.AnnotatorRegistry.get_annotator('instance_id_segmentation_fast',device=annotator_device);seg.attach(rp);segments.append(seg)
 if side=='left':depth=rep.AnnotatorRegistry.get_annotator('distance_to_image_plane',do_array_copy=not annotator_no_copy,device=annotator_device);depth.attach(rp)
extra_cameras={};extra_depth={};extra_segments={}
for name in ['r07_camera2','r07_camera3','arm_camera']:
 for side in ['left','right']:
  optical=name+'_'+side+'_optical';T=rig_home[optical]
  parent_path='/World/Spawn/R12/'+('arm_link5' if name=='arm_camera' else 'base_link')
  if name=='arm_camera':T=np.linalg.inv(forward_transform(root/'models/r12/r12_arm.urdf','arm_link5',{f'arm_joint{i}':0 for i in range(1,6)}))@T
  cam=UsdGeom.Camera.Define(stage,parent_path+'/'+optical);cam.AddTransformOp().Set(Gf.Matrix4d(*(T@np.diag([1.,-1.,-1.,1.])).T.flatten().tolist()));cam.CreateHorizontalApertureAttr(20.955);cam.CreateVerticalApertureAttr(20.955*.75);cam.CreateFocalLengthAttr(20.955*.625);cam.CreateClippingRangeAttr(Gf.Vec2f(.01,30))
  resolution=tuple(native_schema.config['cameras'][optical]['resolution_wh']) if native_producer_enabled else (640,480)
  rp=rep.create.render_product(str(cam.GetPath()),resolution);a=rep.AnnotatorRegistry.get_annotator('rgb',do_array_copy=not annotator_no_copy,device=annotator_device);a.attach(rp);extra_cameras[optical]=a
  native_attach(optical,cam,rp,side=='left')
  if not native_producer_enabled:
   seg=rep.AnnotatorRegistry.get_annotator('instance_id_segmentation_fast',device=annotator_device);seg.attach(rp);extra_segments[optical]=seg
  if side=='left':d=rep.AnnotatorRegistry.get_annotator('distance_to_image_plane',do_array_copy=not annotator_no_copy,device=annotator_device);d.attach(rp);extra_depth[name]=d
if native_producer_enabled:
 native_schema.check_actual_optics(native_optics)
 atomic_json(out/'native_actual_optics.json',dict(verified=True,readback=native_optics,camera_schema_sha256=native_schema.sha256,segmentation_annotators=0,raw_unmasked_only=True))
lidar_names=['front_right','rear_left','low_front_left','low_rear_right']
lidar_T={n:forward_transform(root/'models/r12/r12_arm.urdf','lidar_'+n,{f'arm_joint{i}':0 for i in range(1,6)}) for n in lidar_names}
(out/'lidar_extrinsics.json').write_text(json.dumps({n:T.tolist() for n,T in lidar_T.items()}))
import omni.physx
query=omni.physx.get_physx_scene_query_interface()
contacts=[];diagnostic_arm_contacts=[];contact_frame=[-1]
for prim in stage.Traverse():
 if '/R12/' in str(prim.GetPath()) and prim.HasAPI(UsdPhysics.RigidBodyAPI):
  PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
def contact_event(headers,data):
 for h in headers:
  paths=[str(PhysicsSchemaTools.intToSdfPath(v)) for v in [h.collider0,h.collider1]]
  if any('/R12/arm_' in v for v in paths) and h.num_contact_data:
   diagnostic_arm_contacts.append(dict(step=int(world.current_time_step_index),colliders=paths,count=int(h.num_contact_data),minimum_separation=min(float(data[i].separation) for i in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data))))
  if any('/R12/' in v for v in paths) and any('/Mission/' in v for v in paths) and h.num_contact_data:
   contacts.append({'frame':contact_frame[0],'colliders':paths,'min_separation':min(float(data[i].separation) for i in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data))})
contact_subscription=omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(contact_event)
if gui:
 from isaacsim.core.utils.viewports import set_camera_view
 set_camera_view(eye=np.array([7.,-5.,9.]),target=np.array([2.2,3.2,0.]))
# Single factor official TGS steady-state discrepancy diagnostic.
solver_scenes=[p for p in stage.Traverse() if p.HasAPI(PhysxSchema.PhysxSceneAPI)]
assert len(solver_scenes)==1
solver_scene=solver_scenes[0];solver_scene_api=PhysxSchema.PhysxSceneAPI(solver_scene)
external_attr=solver_scene_api.GetEnableExternalForcesEveryIterationAttr()
external_before=external_attr.Get();assert external_before is False
assert solver_scene_api.GetSolverTypeAttr().Get()=='TGS'
solver_scene_api.CreateEnableExternalForcesEveryIterationAttr(True)
assert external_attr.Get() is True
atomic_json(out/'solver_configuration.json',dict(scene=str(solver_scene.GetPath()),solver_type=solver_scene_api.GetSolverTypeAttr().Get(),external_forces_every_iteration_before=external_before,external_forces_every_iteration_after=external_attr.Get(),articulation_solver_attributes_unchanged=True,baseline='v108 unauthored articulation defaults position32 velocity1, no API Apply or authored iteration change',reference='https://nvidia-omniverse.github.io/PhysX/physx/5.8.0/docs/Simulation.html',limitation='Articulation Coriolis exception remains; no convergence claim'))
(out/'extrinsics.json').write_text(json.dumps(extrinsics,indent=2));apply_renderer_override('before_world_reset');world.reset();names=robot.dof_names
steer=[names.index(n+'_steer_joint') for n in ['lh1','rh1','lh2','rh2']];wheel=[names.index(n+'_wheel_joint') for n in ['lh1','rh1','lh2','rh2']];arm=[names.index(f'arm_joint{i}') for i in range(1,6)]
kp=np.zeros((1,len(names)));kd=np.full_like(kp,2.);kp[0,steer]=40;kd[0,steer]=4;kp[0,arm]=80;kd[0,arm]=8;robot._articulation_view.set_gains(kps=kp,kds=kd)
grip=[names.index('gripper_'+side+'_joint') for side in ['left','right']];grip_cfg=json.loads((root/'configs/r12_gripper.json').read_text());grip_target=grip_cfg['simulation_finger_displacement_m'][1];grip_width=.075
semantic_joint_indices={'steer':steer,'wheel':wheel,'arm':arm,'gripper':grip}
semantic_flat=[i for indices in semantic_joint_indices.values() for i in indices]
if len(names)!=len(set(names)) or len(semantic_flat)!=len(set(semantic_flat)) or set(semantic_flat)!=set(range(len(names))):raise ValueError('Ambiguous or unexpected R12 articulation joint mapping')
atomic_json(out/'articulation_mapping.json',{'explicit_base_root':explicit_base_root,'asset':robot_asset,'articulation_path':articulation_path,'dof_names':list(names),'semantic_indices':semantic_joint_indices,'root_api_paths':[str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)],'scope':'Initialization mapping evidence only; torque signs and sensor axes require separate verification'})
kp[0,grip]=1000;kd[0,grip]=20;robot._articulation_view.set_gains(kps=kp,kds=kd)
eff=np.ones_like(kp);eff[0,grip]=10;eff[0,arm]=2;robot._articulation_view.set_max_efforts(eff)
latest_mode=os.getenv('R12_USE_LATEST_PPO')=='1'
steer_reference_rate=float(os.getenv('R12_STEER_REFERENCE_RATE','7.330382858'))
if not np.isfinite(steer_reference_rate) or not 0<steer_reference_rate<=7.330382858:raise ValueError('Invalid steering reference rate')
wheel_kd=float(os.getenv('R12_WHEEL_KD','.02'));wheel_accel=float(os.getenv('R12_WHEEL_ACCEL','0'))
if not np.isfinite([wheel_kd,wheel_accel]).all() or not 0< wheel_kd<=.08 or not 0<=wheel_accel<=24.45:raise ValueError('Invalid experimental wheel control gains')
if latest_mode:
 kp[0,steer]=0;kp[0,wheel]=0;kd[0,steer]=0;kd[0,wheel]=0;robot._articulation_view.set_gains(kps=kp,kds=kd);eff[0,steer]=.54;eff[0,wheel]=.108;robot._articulation_view.set_max_efforts(eff)
rolling_projection=os.getenv('R12_ROLLING_PROJECTION','0')=='1'
if rolling_projection and (residual_scale!=0 or not latest_mode):raise ValueError('Diagnostic rolling projection requires residual-free latest mode')
coordinate_drive=os.getenv('R12_COORDINATE_DRIVE','0')=='1'
if coordinate_drive and (residual_scale!=0 or not latest_mode or rolling_projection):raise ValueError('Coordination requires residual-free latest mode without projection override')
encoder_feedback120=os.getenv('R12_ENCODER_FEEDBACK120','0')=='1'
if encoder_feedback120 and not latest_mode:raise ValueError('Encoder feedback requires latest actuator mode')
wheel_action=np.zeros(4)
wheel_integral=WheelIntegral();encoder_wheel_velocity=np.zeros(4);integral_enabled=os.getenv('R12_WHEEL_INTEGRAL','0')=='1'
stationary_hold_torque=np.zeros(4)
stop_brake_valid=False;stop_brake_gyro_z=0.;stop_brake_enabled=os.getenv("R12_STOP_YAW_BRAKE","0")=="1"
q=np.zeros(len(names));vel=np.zeros(len(names));command=np.zeros(3);wheel_scale=1.;previous_target=None;wheel_slew=WheelSlew();emergency=False
from arm_practice.native_rgbd.observe_setup import TARGET as observation_target,settled as observation_settled
if not stationary_capture:raise ValueError('Observation setup requires stationary diagnostic')
setup_q=robot.get_joint_positions().copy();setup_q[arm]=observation_target;setup_q[grip]=[.02768613,-.02768613]
# Initial state placement only, before warmup and before issuing sensor receipts.
robot.set_joint_positions(setup_q);robot.set_joint_velocities(np.zeros(len(names)));robot.apply_action(ArticulationAction(joint_positions=setup_q))
atomic_json(out/'observer_initial_setup.json',dict(initial_arm_target=observation_target.tolist(),initial_gripper_target=setup_q[grip].tolist(),scope='Simulator initial state only; folded path CPU audited but NOT physically executed',motion_authorized=False))
apply_renderer_override('before_warmup')
for warmup_step in range(physics_hz):world.step(render=(warmup_step+1)%imu_stride==0)
settle_samples=[];consecutive=0
for settle_step in range(4*physics_hz):
 world.step(render=(settle_step+1)%imu_stride==0)
 if (settle_step+1)%imu_stride:continue
 measured=robot.get_joint_positions();measured_velocity=robot.get_joint_velocities();ok=observation_settled(measured[arm],measured_velocity[arm],measured[grip],measured_velocity[grip]);consecutive=consecutive+1 if ok else 0
 settle_samples.append(dict(physics_step=int(world.current_time_step_index),q=measured[arm].tolist(),qd=measured_velocity[arm].tolist(),gripper_q=measured[grip].tolist(),gripper_qd=measured_velocity[grip].tolist(),within_gate=ok))
 if consecutive>=30:break
atomic_json(out/'observer_actual_settling.json',dict(samples=settle_samples,consecutive=consecutive,settled=consecutive>=30,scope='Actual joint encoder and native velocity; no synthetic zero velocity',motion_authorized=False))
if consecutive<30:raise ValueError('Observer setup did not settle within4s')
audit_render_settings('scene_warmup_complete')
if hydra_stats is not None and hydra_delayed:
 profiler_settings.set_bool('/profiler/enabled',True)
 atomic_json(out/'hydra_profile_after_warmup.json',{'enabled':profiler_settings.get('/profiler/enabled'),'warmup_steps':physics_hz,'monotonic_ns':time.monotonic_ns()})

if stationary_capture:
 from arm_practice.native_rgbd.observe_setup import preserve_arm_reference
 measured_hold=robot.get_joint_positions().copy();q=preserve_arm_reference(measured_hold,setup_q,arm);stationary_reference=q.copy();grip_target=float(q[grip[0]]);grip_width=float(grip_cfg['cad_zero_aperture_m']+q[grip[0]]-q[grip[1]])
sub_previous=np.rint(robot.get_joint_positions()[wheel]/4*4096/(2*np.pi))*2*np.pi/4096*4;sub_encoder=np.zeros(4);wheel_demand=np.zeros(4)
if native_producer_enabled:
 from arm_practice.native_rgbd.mission_entrypoint import MissionNativeProducer
 if not (latest_mode and encoder_feedback120 and physics_hz==960 and wheel_kd==.04 and residual_scale==0 and prepared_camera_rig is not None):raise ValueError('native producer fixed actuator/sensor contract mismatch')
 native_rgb={f'r07_camera1_{side}_optical':anns[i] for i,side in enumerate(['left','right'])};native_rgb.update(extra_cameras)
 native_depth={'r07_camera1_depth':depth,**{k+'_depth':v for k,v in extra_depth.items()}}
 if os.getenv('R12_NATIVE_STARTUP_ORDER')=='1':
  import carb
  from omni.syntheticdata import SyntheticData
  atomic_json(out/'renderer_startup_order_active.json',validate_startup_order(carb.settings.get_settings(),SyntheticData))
 sys.path.insert(0,str(root));from torque_parity import TorqueParity
 native_torque_parity=TorqueParity();atomic_json(out/'torque_parity.json',native_torque_parity.result())
# The worker receives only rendered camera frames; fixture labels stay in this process.
perception_enabled=os.getenv('R12_MISSION_PERCEPTION','1')=='1'
perception_interval=int(os.getenv('R12_PERCEPTION_INTERVAL_FRAMES','5'))
if not 1<=perception_interval<=20:raise ValueError('Invalid perception interval')
fixture=json.loads((root/'models/navigation/scene_fixture.json').read_text())
signal_start=None;signal_ready=False;perception={};request={};panel_bbox=None
if perception_enabled:
 worker=subprocess.Popen(['/home/seongmin/ros2_ws/src/turtlebot-swerve-cuvslam/.venv-perception-live/bin/python',str(root/'scripts/r12_perception_worker.py')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=(out/'perception_worker.log').open('w'),text=True)
 atexit.register(stop_worker,worker)
 perception_reader=JsonPipeReader(worker.stdout)
 ready=perception_reader.receive(lambda obj:obj.get('ready') is True,float(os.getenv('R12_PERCEPTION_STARTUP_TIMEOUT_S','120')))
 atomic_json(out/'perception_models.json',ready)
 perception_timeout_s=float(os.getenv('R12_PERCEPTION_TIMEOUT_S','30'))
 perception_max_age_ns=int(float(os.getenv('R12_PERCEPTION_MAX_AGE_S','0.5'))*1e9)
 if perception_max_age_ns<=0:raise ValueError('Invalid perception maximum age')
sock=socket.create_connection(('127.0.0.1',args.port),timeout=30);reader=sock.makefile('r');logs=[];previous=None
from arm_practice.terminal_observation import TerminalObservation
terminal_gate=TerminalObservation() if os.getenv('R12_TERMINAL_OBSERVATION','0')=='1' else None
terminal_captured=False
video=None
if os.getenv('R12_RECORD_VIDEO','0')=='1':
 from mission_video import MissionVideo
 video=MissionVideo(out/'mission.mp4');atexit.register(video.close)
imu_latest_sampling=os.getenv('R12_IMU_LATEST_SAMPLING','0')=='1'
imu_last_times=[float(imu.get_current_frame(read_gravity=True)['time']) for imu in imus]
shared_frame_enabled=os.getenv('R12_SHARED_FRAME_DIAGNOSTIC','0')=='1'
if shared_frame_enabled:
 if not stationary_capture or perception_enabled:raise ValueError('Shared frames require stationary capture without perception')
 from arm_practice.shared_frame_prototype import SharedFrameSlot
 shared_frame_slot=SharedFrameSlot(directory=os.environ['R12_SHARED_FRAME_DIRECTORY']);atexit.register(shared_frame_slot.close)
ownership_audit=os.getenv('R12_OWNERSHIP_AUDIT','0')=='1'
if ownership_audit and not stationary_capture:raise ValueError('Ownership audit requires stationary capture')
import hashlib
ownership_rows=[];source_views=[];prior_snapshot=None;prior_hash=None
gpu_probe_enabled=os.getenv('R12_GPU_CAMERA_PROBE','0')=='1';gpu_probe_retained=None
gpu_acquisition=os.getenv('R12_GPU_ACQUISITION','0')=='1';gpu_validate=os.getenv('R12_GPU_ACQUISITION_VALIDATE','0')=='1'
gpu_validation_rows=[];gpu_prior=None;gpu_prior_hash=None
if gpu_acquisition and (not stationary_capture or annotator_device!='cuda'):raise ValueError('GPU acquisition requires stationary CUDA diagnostic')
if gpu_probe_enabled and not stationary_capture:raise ValueError('GPU probe requires stationary capture')
all_image_annotators=anns+[depth]+list(extra_cameras.values())+list(extra_depth.values())
if ownership_audit or annotator_no_copy:
 copy_flags=[bool(a.get_node().get_attribute('outputs:__do_array_copy').get()) for a in all_image_annotators]
 atomic_json(out/'annotator_copy_flags.json',{'requested_copy':not annotator_no_copy,'actual_flags':copy_flags})
 if any(flag!=(not annotator_no_copy) for flag in copy_flags):raise ValueError('Annotator copy configuration mismatch')
def read_image_annotation(ann):
 raw=ann.get_data(device='cpu')
 if ownership_audit:source_views.append((raw,hashlib.sha256(raw.tobytes()).hexdigest()))
 return raw
def snapshot_hash(arrays):
 return {k:hashlib.sha256(np.asarray(v).tobytes()).hexdigest() for k,v in arrays.items()}
# Diagnostic-only native force state; never used by controller or capture issuer.
dynamics_support={}
def dynamics_read(name,**kwargs):
 if dynamics_support.get(name,{}).get('supported') is False:return None
 try:
  method=getattr(robot._articulation_view._physics_view,'get_gravity_compensation_forces') if name=='get_generalized_gravity_forces' else getattr(robot._articulation_view,name);value=method(**kwargs)
  if value is None:raise ValueError('getter returned None')
  if isinstance(value,tuple):
   parts=[np.asarray(v)for v in value]
   if not all(np.isfinite(v).all()for v in parts):raise ValueError('nonfinite tuple getter result')
   dynamics_support[name]={'supported':True,'tuple_shapes':[list(v.shape)for v in parts]}
   return [v.tolist()for v in parts]
  value=np.asarray(value)
  if not np.isfinite(value).all():raise ValueError('nonfinite getter result')
  dynamics_support[name]={'supported':True,'shape':list(value.shape)}
  return value.tolist()
 except Exception as error:
  dynamics_support[name]={'supported':False,'reason':type(error).__name__+': '+str(error)}
  return None
dynamics_static={name:dynamics_read(name)for name in ['get_body_masses','get_body_coms','get_body_inertias','get_dof_limits','get_mass_matrices','get_generalized_gravity_forces','get_body_disable_gravity']}
dynamics_static['dof_names']=list(names);dynamics_static['arm_indices']=[int(i)for i in arm];dynamics_static['joint_wrench_frame']='child local incoming joint frame; joint_names maps internal index+1 per installed API'
dynamics_static['usd_joint_attributes']=[]
for diagnostic_prim in stage.Traverse():
 if diagnostic_prim.GetPath().pathString.startswith('/World/Spawn/R12') and diagnostic_prim.IsA(UsdPhysics.Joint):
  dynamics_static['usd_joint_attributes'].append({'path':str(diagnostic_prim.GetPath()),'attributes':{a.GetName():str(a.Get())for a in diagnostic_prim.GetAttributes() if any(x in a.GetName().lower()for x in ['axis','limit','localrot','localpos','drive'])}})
dynamics_static['gravity_mapping']='raw PhysX get_gravity_compensation_forces, floating base first6 then DOFs; wrapper default selected only num_dof15 and truncated21 vector in v120'
dynamics_static['active_constraint_limit_impulse']='not directly instrumented: positional proximity is diagnostic only, not solver active-set evidence'
atomic_json(out/'dynamics_static.json',dynamics_static)
def dynamics_tick():
 return dict(wrench=dynamics_read('get_measured_joint_forces',joint_names=[names[i]for i in arm]),gravity=dynamics_read('get_generalized_gravity_forces'),measured_effort=dynamics_read('get_measured_joint_efforts',joint_names=[names[i]for i in arm]),contact_events_seen=len(diagnostic_arm_contacts),diagnostic_only=True)

import gc
step_gc_events=[]
def diagnostic_gc_callback(phase,info):step_gc_events.append(dict(phase=phase,monotonic_ns=time.monotonic_ns(),generation=info.get('generation'),collected=info.get('collected')))
gc.callbacks.append(diagnostic_gc_callback)
atexit.register(lambda:atomic_json(out/'step_gc_events.json',step_gc_events))
simulate_timings=[]
original_sim_interface=world._physics_context._physics_sim_interface
class DiagnosticSimulationProxy:
 def __getattr__(self,name):return getattr(original_sim_interface,name)
 def simulate(self,*args,**kwargs):
  wall=time.monotonic_ns();cpu=time.thread_time_ns()
  try:return original_sim_interface.simulate(*args,**kwargs)
  finally:simulate_timings.append(dict(step=int(world.current_time_step_index),start_ns=wall,end_ns=time.monotonic_ns(),thread_cpu_ns=time.thread_time_ns()-cpu))
world._physics_context._physics_sim_interface=DiagnosticSimulationProxy()
atomic_json(out/'installed_step_path.json',dict(path='World.step -> SimulationContext.step(render=False) -> PhysicsContext._step -> _physics_sim_interface.simulate(dt,current_time)',separate_fetch_call=False,app_update_called_in_physics_only_step=False,fabric_update=False,scope='Installed Isaac6.1 source inspection; internal native simulate scheduling opaque',call_order_unchanged=True))
try:
 hold_timeline=[]
 hold_phase="bounded_post_setup_settle"
 def apply_controls_and_step(sub):
  global sub_previous,sub_encoder,wheel_demand
  if latest_mode:
   position=robot.get_joint_positions();velocity_now=robot.get_joint_velocities();torque=np.zeros(len(names));torque[steer]=dc_effort(q[steer],position[steer],velocity_now[steer],True)
   if encoder_feedback120:
    if sub%imu_stride==0:
     sub_measured=np.rint(position[wheel]/4*4096/(2*np.pi))*2*np.pi/4096*4;sub_encoder=wrapped_velocity(sub_measured,sub_previous,1/120);sub_previous=sub_measured.copy()
     wheel_demand=dc_effort(vel[wheel],position[wheel],sub_encoder,wheel_kd=wheel_kd,wheel_feedforward=integral_torque)
    # Plant back-EMF uses physical shaft velocity, never exposed to the encoder controller.
    upper=np.clip(.54*(1-velocity_now[wheel]/29.321531434),-.108,.108);lower=np.clip(.54*(-1-velocity_now[wheel]/29.321531434),-.108,.108);torque[wheel]=np.clip(wheel_demand,lower,upper)
   else:torque[wheel]=dc_effort(vel[wheel],position[wheel],velocity_now[wheel],wheel_kd=wheel_kd,wheel_feedforward=integral_torque)
   drive_substeps.append({'native_wheel_velocity_rad_s':velocity_now[wheel].tolist(),'encoder_feedback_rad_s':sub_encoder.tolist() if encoder_feedback120 else None,'applied_wheel_torque_Nm':torque[wheel].tolist()});robot.apply_action(ArticulationAction(joint_positions=q,joint_efforts=torque))
  else:robot.apply_action(ArticulationAction(joint_positions=q,joint_velocities=vel))
  # Bounded read-only diagnostic at EVERY original physics step.
  if True:
   diag_begin=time.monotonic_ns();actual_action=robot.get_applied_action()
   diag=dict(phase=hold_phase,before_step=int(world.current_time_step_index),before_sim_time_s=float(world.current_time),read_start_ns=diag_begin,q=robot.get_joint_positions()[arm].tolist(),qd=robot.get_joint_velocities()[arm].tolist(),requested_target=q[arm].tolist(),applied_target=np.asarray(actual_action.joint_positions)[arm].tolist(),applied_feedforward_effort=robot.get_applied_joint_efforts(joint_indices=np.array(arm)).tolist(),measured_joint_effort=robot.get_measured_joint_efforts(joint_indices=np.array(arm)).tolist(),kp=kp[0,arm].tolist(),kd=kd[0,arm].tolist(),max_effort=eff[0,arm].tolist(),step_start_ns=time.monotonic_ns())
  diag['phase_times']={'backend_pre_start_ns':time.monotonic_ns()}
  diag['backend_before_q']=np.asarray(robot._articulation_view._physics_view.get_dof_positions())[0,arm].tolist()
  diag['backend_before_qd']=np.asarray(robot._articulation_view._physics_view.get_dof_velocities())[0,arm].tolist()
  diag['phase_times']['backend_pre_end_ns']=time.monotonic_ns();diag['phase_times']['dynamics_pre_start_ns']=time.monotonic_ns()
  diag['dynamics_before']=dynamics_tick()
  diag['phase_times']['dynamics_pre_end_ns']=time.monotonic_ns();diag['phase_times']['world_step_start_ns']=time.monotonic_ns()
  world.step(render=False)
  diag['phase_times']['world_step_end_ns']=time.monotonic_ns();diag['phase_times']['dynamics_post_start_ns']=time.monotonic_ns()
  diag['dynamics_after']=dynamics_tick()
  diag['phase_times']['dynamics_post_end_ns']=time.monotonic_ns();diag['phase_times']['backend_post_start_ns']=time.monotonic_ns()
  diag['backend_after_q']=np.asarray(robot._articulation_view._physics_view.get_dof_positions())[0,arm].tolist()
  diag['backend_after_qd']=np.asarray(robot._articulation_view._physics_view.get_dof_velocities())[0,arm].tolist()
  diag['phase_times']['backend_post_end_ns']=time.monotonic_ns()
  if True:
   diag.update(step_end_ns=time.monotonic_ns(),after_step=int(world.current_time_step_index),after_sim_time_s=float(world.current_time),after_q=robot.get_joint_positions()[arm].tolist(),after_qd=robot.get_joint_velocities()[arm].tolist());hold_timeline.append(diag)
   if native_producer_enabled and 'native_producer' in globals():
    epoch_start=time.monotonic_ns();epoch_q=robot.get_joint_positions()[arm].copy();epoch_end=time.monotonic_ns()
    if int(world.current_time_step_index)!=diag['after_step']:raise ValueError('physics advanced while recording settling epoch')
    actual_dt=diag['after_sim_time_s']-diag['before_sim_time_s']
    same_step_fd=((np.asarray(diag['after_q'])-np.asarray(diag['q']))/actual_dt).tolist()
    conservative_native=np.maximum(np.abs(diag['qd']),np.abs(diag['after_qd'])).tolist()
    epoch_encoder=(np.rint(epoch_q*4096/(2*np.pi))*2*np.pi/4096).tolist()
    native_producer.issuer.record_encoder_epoch(physics_step=diag['after_step'],sim_time_s=diag['after_sim_time_s'],read_start_ns=epoch_start,read_end_ns=epoch_end,measured_encoder=epoch_encoder,native_qd=conservative_native,same_step_fd=same_step_fd)
 
 
 # Fixed before execution: additional2s simulated hold; never early-pass on30 samples.
 # Diagnostic-only: collect even on velocity-gate failure; no admitted observation.
 additional_hold_steps=2*physics_hz
 atomic_json(out/'continuous_settle_plan.json',dict(additional_hold_steps=additional_hold_steps,physics_hz=physics_hz,additional_sim_seconds=2.,early_exit=False,native_velocity_gate_rad_s=.02,scope='Diagnostic observation, not admission; original capture gates unchanged',motion_authorized=False))
 integral_torque=np.zeros(4);drive_substeps=[]
 for hold_sub in range(additional_hold_steps):apply_controls_and_step(hold_sub)
 atomic_json(out/'continuous_settle_prefix.json',hold_timeline)
 atomic_json(out/'diagnostic_arm_contacts.json',diagnostic_arm_contacts)
 # Begin capture epoch only after diagnostic hold; no IMU interval is skipped.
 if native_producer_enabled:
  native_producer=MissionNativeProducer(requested_epochs=args.frames,root=root,out=out,run_id=capture_run_id,schema=native_schema,robot=robot,world=world,arm=arm,joint_names=[names[i]for i in arm],rig=prepared_camera_rig,rgb=native_rgb,depth=native_depth,rgb_refs=native_rgb_refs,depth_refs=native_depth_refs)
 hold_phase='capture_and_current'
 packet_previous_native_time=None
 for frame in range(args.frames):
  frame_loop_started_ns=time.monotonic_ns()
  if command_delay is not None:
   # Packet0 stamps the end of the first50ms step. At the next iteration's
   # start, simulator time equals the previous packet stamp, not wall time.
   delay_sample=command_delay.sample(950_000_000+frame*50_000_000)
   command=np.asarray(delay_sample['velocity'],float)
   command_delay_rows.append(dict(frame=frame,application_sim_ns=950_000_000+frame*50_000_000,**delay_sample))
  contact_frame[0]=frame
  if not stationary_capture:
   grip_goal=(grip_width-grip_cfg['cad_zero_aperture_m'])/2;grip_target+=float(np.clip(grip_goal-grip_target,-.04*.05,.04*.05));q[grip]=[grip_target,-grip_target]
  if signal_ready and signal_start is None:signal_start=frame
  elapsed=-1 if signal_start is None else (frame-signal_start)*.05
  active=fixture['sequence'][int(elapsed//2)] if 0<=elapsed<24 and elapsed%2<1 else None
  for color,rgb_light in [('R',(1.,0.,0.)),('Y',(1.,1.,0.)),('G',(0.,1.,0.))]:
   shader=UsdShade.Shader(stage.GetPrimAtPath('/World/Mission/FixtureMaterials/Lamp'+color+'/Shader'))
   shader.GetInput('emissiveColor').Set(Gf.Vec3f(*rgb_light) if active==color else Gf.Vec3f(0))
   shader.GetInput('diffuseColor').Set(Gf.Vec3f(*rgb_light) if active==color else Gf.Vec3f(.015))
  coordination_status=None
  allocator_input=None
  transition_braking=np.zeros(4,dtype=bool)
  if stationary_capture:
   q=stationary_reference.copy();vel[:]=0.;a=q[steer].copy();w=np.zeros(4);previous_target=a.copy()
  elif latest_mode:
   encoder_steer=np.rint(robot.get_joint_positions()[steer]*4096/(2*np.pi))*2*np.pi/4096
   allocator_input={'command':command.tolist(),'steering_rad':robot.get_joint_positions()[steer].tolist(),'previous_target_rad':None if previous_target is None else previous_target.tolist(),'scaled_action':wheel_action.tolist(),'encoder_wheel_rad_s':encoder_wheel_velocity.tolist()}
   a,w=allocate(command,robot.get_joint_positions()[steer],previous_target,wheel_action,steer_rate=steer_reference_rate,low_speed_residual_fraction=low_speed_residual_fraction)
   if rolling_projection:w=project_current_heading(command,robot.get_joint_positions()[steer])
   if stop_brake_enabled and stop_brake_valid and not emergency and np.linalg.norm(command)<1e-6:
    w=stop_yaw_targets(robot.get_joint_positions()[steer],stop_brake_gyro_z,vel[wheel],.05)
   coordinating=coordinate_drive and not emergency and np.linalg.norm(command)>1e-6
   if coordinating:
    a,w,coordination_status=coordinate_turn_with_drive(command,encoder_steer,a,encoder_wheel_velocity,return_status=True)
   if not coordinating and os.getenv('R12_STEERING_TRANSITION_BRAKE','0')=='1':
    a,w,transition_braking=brake_before_large_turn(a,robot.get_joint_positions()[steer],w,encoder_wheel_velocity)
   if wheel_accel>0:
    limited=vel[wheel]+np.clip(w-vel[wheel],-wheel_accel*.05,wheel_accel*.05)
    if coordinating:
     changed=np.abs(limited-w)>1e-9
     a=np.where(changed,encoder_steer,a)
     coordination_status['held_for_previous_target_slew']=changed.tolist()
     coordination_status['final_encoder_delta_rad_s']=(limited-encoder_wheel_velocity).tolist()
     coordination_status['input_encoder_steer']=encoder_steer.tolist()
     coordination_status['input_encoder_wheels']=encoder_wheel_velocity.tolist()
    w=limited
   previous_target=a;q[steer]=a;vel[wheel]=np.zeros(4) if emergency else w
  else:
   a,w=actuate(net,command,robot.get_joint_positions()[steer],.05,previous_target);previous_target=a;q[steer]=a;vel[wheel]=wheel_slew.step(w*wheel_scale,interlock=emergency or (np.linalg.norm(command)>1e-6 and np.max(np.abs(w))<1e-9))
  integral_torque=wheel_integral.step(vel[wheel],encoder_wheel_velocity,.05,wheel_kd,enabled=not stationary_capture and integral_enabled and not emergency and np.linalg.norm(command)>1e-6)
  if not stationary_capture and not emergency and np.linalg.norm(command)<1e-6:integral_torque+=stationary_hold_torque
  drive_substeps=[]
  imu_samples=[[],[]]
  if native_producer_enabled:
   from arm_practice.native_rgbd.stationary_gc import collect_stationary
   if not np.array_equal(q,stationary_reference)or np.any(vel!=0):raise ValueError('GC_barrier_target_changed')
   def gc_encoder_state():return (int(world.current_time_step_index),float(world.current_time),robot.get_joint_positions()[arm].tolist(),robot.get_joint_velocities()[arm].tolist())
   barrier=collect_stationary(stationary=stationary_capture,capture_pending=native_producer.pending is not None,rpc_pending=native_producer.decoder_bridge.pending is not None,command=list(map(float,command)),read_state=gc_encoder_state)
   atomic_json(out/('gc_barrier_%02d.json'%frame),dict(frame=frame,**barrier))
  physics_started_ns=time.monotonic_ns()
  for sub in range(physics_hz//20):
   apply_controls_and_step(sub)
   if native_producer_enabled and (sub+1)%imu_stride==0:native_producer.sample_control_encoder()
   for i,imu in enumerate(imus):
    if imu_latest_sampling:
     if (sub+1)%imu_stride:continue
     native=imu._imu_sensor_interface.get_sensor_reading(imu.prim_path,use_latest_data=True,read_gravity=True)
     sample={'time':native.time,'is_valid':native.is_valid,'ang_vel':[native.ang_vel_x,native.ang_vel_y,native.ang_vel_z],'lin_acc':[native.lin_acc_x,native.lin_acc_y,native.lin_acc_z]}
    else:sample=imu.get_current_frame(read_gravity=True)
    sensor_time=float(sample['time'])
    if sensor_time>imu_last_times[i]:
     imu_samples[i].append({'physics_step':int(world.current_time_step_index),'physics_time_s':float(world.current_time),'time':sensor_time,'valid':bool(sample['is_valid']),'gyro':np.asarray(sample['ang_vel']).copy().tolist(),'accel':np.asarray(sample['lin_acc']).copy().tolist()});imu_last_times[i]=sensor_time
  # Conservative acquisition start includes rendering and sensor readback latency.
  acquisition_timing_enabled=os.getenv("R12_ACQUISITION_TIMING","0")=="1"
  acquisition_stages={}
  readback_parts={"rgb_depth_ns":0,"segmentation_ns":0,"mask_ns":0,"diagnostic_write_ns":0}
  if native_producer_enabled:
   capture_joint_epoch_enabled=False
   # Read-only native articulation diagnostic; no controller/feedback substitution.
   native_pose_read_start=time.monotonic_ns()
   native_pose_q=robot.get_joint_positions()[arm].copy()
   native_pose_qd=robot.get_joint_velocities()[arm].copy()
   native_pose_read_end=time.monotonic_ns()
   atomic_json(out/('capture_pose_diagnostic_%02d.json'%frame),dict(frame=frame,physics_step=int(world.current_time_step_index),physics_time_s=float(world.current_time),read_start_ns=native_pose_read_start,read_end_ns=native_pose_read_end,q=native_pose_q.tolist(),native_qd=native_pose_qd.tolist(),native_qd_below_startup_threshold=bool(np.max(abs(native_pose_qd))<=.02),scope='Articulation getter before acquire; compare actual postcapture encoder FD separately; no tracking certificate',motion_authorized=False))
   native_result=native_producer.acquire(frame,command=command,substeps=len(drive_substeps),imu_samples=imu_samples)
   evaluator_only.record(frame)
   if not native_result['captured']:
    native_producer.finish_frame(frame)
    continue
   # Native-ready has completed. Only now may new controlled physics samples run.
   def stationary_post_step(sub):
    if np.any(command!=0) or not np.array_equal(q,stationary_reference) or np.any(vel!=0):raise ValueError('Postcapture requires unchanged stationary targets')
    apply_controls_and_step(sub)
   def read_post_imus():
    samples=[]
    for i,imu in enumerate(imus):
     native=imu._imu_sensor_interface.get_sensor_reading(imu.prim_path,use_latest_data=True,read_gravity=True)
     samples.append({'time':float(native.time),'valid':bool(native.is_valid),'gyro':[float(native.ang_vel_x),float(native.ang_vel_y),float(native.ang_vel_z)],'accel':[float(native.lin_acc_x),float(native.lin_acc_y),float(native.lin_acc_z)]})
     imu_last_times[i]=float(native.time)
    return samples
   native_producer.refresh_after_capture(stationary_post_step,read_post_imus)
   raw=native_result['pixels'];raw_depth=native_result['depths']
   rgb=np.stack([raw['r07_camera1_'+side+'_optical']for side in ('left','right')]);dep=raw_depth['r07_camera1_depth']
   extras={k:v for k,v in raw.items() if not k.startswith('r07_camera1_')};extras.update({k:v for k,v in raw_depth.items() if k!='r07_camera1_depth'})
   frame_capture_monotonic_ns=native_result['certificate']['body']['capture_monotonic_lower_bound_ns']
   frame_arrays={}  # no native pixels under synthetic_packet_20hz metadata
  else:
   capture_joint_epoch_enabled=os.getenv('R12_CAPTURE_JOINT_EPOCH','0')=='1'
   if capture_joint_epoch_enabled:
    capture_physics_step=int(world.current_time_step_index)
    frame_capture_monotonic_ns=time.monotonic_ns()
    capture_encoder_raw=robot.get_joint_positions().copy()
    capture_encoder_read_end_ns=time.monotonic_ns()
   else:frame_capture_monotonic_ns=time.monotonic_ns()
   if acquisition_timing_enabled:acquisition_stages.update(frame_loop_start=frame_loop_started_ns,physics_start=physics_started_ns,capture_start=frame_capture_monotonic_ns)
   capture_render_start_ns=time.monotonic_ns()
   world.render()
   render_return_ns=time.monotonic_ns()
   if frame in (0,args.frames-1):audit_render_settings('after_render_'+str(frame))
   if hydra_stats is not None:
    profile_start=time.monotonic_ns()
    try:profile_sample={'devices':hydra_stats.get_gpu_profiler_result()}
    except Exception as error:profile_sample={'error':str(error)}
    hydra_profile_rows.append({'frame':frame,'render_start_ns':frame_capture_monotonic_ns,'render_return_ns':render_return_ns,'query_start_ns':profile_start,'query_end_ns':time.monotonic_ns(),**profile_sample})
   if gpu_probe_enabled:
    from gpu_camera_probe import probe,verify_retained
    if frame==0:
     try:
      probe_cameras=[('r07_camera1_'+side,anns[i],segments[i],depth if i==0 else None) for i,side in enumerate(('left','right'))]
      probe_cameras += [(key,ann,extra_segments[key],extra_depth[key[:-len('_left_optical')]] if key.endswith('_left_optical') else None) for key,ann in extra_cameras.items()]
      gpu_probe_report,gpu_probe_retained=probe(probe_cameras,out/'gpu_camera_devices.json');gpu_probe_report['supported']=True
     except Exception as error:
      gpu_probe_report={'supported':False,'error_type':type(error).__name__,'error':str(error)}
     atomic_json(out/'gpu_camera_probe.json',gpu_probe_report)
    elif frame==1 and gpu_probe_retained is not None:
     gpu_probe_report['owned_snapshots_survive_next_render']=verify_retained(gpu_probe_retained)
     atomic_json(out/'gpu_camera_probe.json',gpu_probe_report);gpu_probe_retained=None
   if ownership_audit:
    if prior_snapshot is not None:
     if snapshot_hash(prior_snapshot)!=prior_hash:raise ValueError('Owned snapshot changed after next render')
     ownership_rows[-1]['survived_next_render']=True
    source_views=[]
   if acquisition_timing_enabled:acquisition_stages["render_done"]=time.monotonic_ns()
   if gpu_acquisition:
    from gpu_camera_acquisition import acquire
    if gpu_validate and gpu_prior is not None:
     if snapshot_hash(gpu_prior)!=gpu_prior_hash:raise ValueError('GPU host snapshot changed after next render')
     gpu_validation_rows[-1]['survived_next_render']=True
    gpu_cameras=[('r07_camera1_'+side+'_optical',anns[i],segments[i],depth if i==0 else None) for i,side in enumerate(('left','right'))]
    gpu_cameras += [(key,ann,extra_segments[key],extra_depth[key[:-len('_left_optical')]] if key.endswith('_left_optical') else None) for key,ann in extra_cameras.items()]
    gpu_arrays,gpu_info=acquire(gpu_cameras,verify_sources=gpu_validate,profile=os.getenv('R12_GPU_ACQUISITION_PROFILE','0')=='1',stream_probe=os.getenv('R12_GPU_STREAM_PROBE','0')=='1')
    if gpu_info.get('stream_probe') is not None:
     with (out/'gpu_stream_probe.jsonl').open('a') as stream_file:stream_file.write(json.dumps({'frame':frame,**gpu_info['stream_probe']})+'\n')
    if gpu_info.get('profile') is not None:
     with (out/'gpu_acquisition_profile.jsonl').open('a') as profile_file:profile_file.write(json.dumps({'frame':frame,**gpu_info['profile']})+'\n')
    if frame==0:atomic_json(out/'gpu_acquisition_layout.json',gpu_info)
   clearance_depth_arrays={}
   if not gpu_acquisition or gpu_validate:
    part_start=time.monotonic_ns()
    if os.getenv('R12_SINGLE_RGB_STACK_COPY','0')=='1':
     # Stack owns the RGB buffer; no preliminary per-eye copy is needed.
     rgb=np.stack([read_image_annotation(a)[:,:,:3] for a in anns])
    else:rgb=np.stack([read_image_annotation(a)[:,:,:3].copy() for a in anns])
    dep=read_image_annotation(depth).copy()
    if archive_clearance_depth:clearance_depth_arrays['r07_camera1_depth_unmasked']=dep.copy()
    readback_parts["rgb_depth_ns"]+=time.monotonic_ns()-part_start
    # Exclude only the robot's own calibrated geometry from camera observations.
    # Instance IDs are simulator self-masking instrumentation, not object labels for navigation.
    for i,seg in enumerate(segments):
     part_start=time.monotonic_ns();result=seg.get_data(device='cpu')
     readback_parts['segmentation_ns']+=time.monotonic_ns()-part_start;part_start=time.monotonic_ns()
     labels=result['info']['idToLabels'];ids=[int(k) for k,v in labels.items() if '/R12/' in str(v)]
     mask=np.isin(result['data'],ids);rgb[i][mask]=0
     if i==0:dep[mask]=0
     readback_parts['mask_ns']+=time.monotonic_ns()-part_start;part_start=time.monotonic_ns()
     if frame==0:(out/f'self_mask_{i}.json').write_text(json.dumps({'labels':labels,'masked_pixels':int(mask.sum())},indent=2))
     readback_parts['diagnostic_write_ns']+=time.monotonic_ns()-part_start
    extras={}
    for optical,ann in extra_cameras.items():
     part_start=time.monotonic_ns();pixels=read_image_annotation(ann)[:,:,:3].copy()
     readback_parts['rgb_depth_ns']+=time.monotonic_ns()-part_start;part_start=time.monotonic_ns()
     seg=extra_segments[optical].get_data(device='cpu')
     readback_parts['segmentation_ns']+=time.monotonic_ns()-part_start;part_start=time.monotonic_ns()
     ids=[int(k) for k,v in seg['info']['idToLabels'].items() if '/R12/' in str(v)];mask=np.isin(seg['data'],ids);pixels[mask]=0;extras[optical]=pixels
     readback_parts['mask_ns']+=time.monotonic_ns()-part_start
     if optical.endswith('left_optical'):
      part_start=time.monotonic_ns();name=optical[:-len('_left_optical')];dd=read_image_annotation(extra_depth[name]).copy()
      if archive_clearance_depth:clearance_depth_arrays[name+'_depth_unmasked']=dd.copy()
      readback_parts['rgb_depth_ns']+=time.monotonic_ns()-part_start;part_start=time.monotonic_ns()
      dd[mask]=0;extras[name+'_depth']=dd
      readback_parts['mask_ns']+=time.monotonic_ns()-part_start
   if gpu_acquisition:
    if gpu_validate:
     baseline={'r07_camera1_left_optical':rgb[0],'r07_camera1_right_optical':rgb[1],'r07_camera1_depth':dep,**extras}
     if baseline.keys()!=gpu_arrays.keys() or any(baseline[k].tobytes()!=gpu_arrays[k].tobytes() for k in baseline):raise ValueError('GPU/CPU masked bytes mismatch')
     gpu_validation_rows.append(dict(frame=frame,arrays=len(gpu_arrays),bytes_equal=True,source_unchanged=gpu_info['source_unchanged'],source_arrays_checked=gpu_info['source_arrays_checked'],survived_next_render=None))
     gpu_prior=gpu_arrays;gpu_prior_hash=snapshot_hash(gpu_arrays)
     atomic_json(out/'gpu_acquisition_validation.json',gpu_validation_rows)
    rgb=np.stack([gpu_arrays['r07_camera1_'+side+'_optical'] for side in ('left','right')]);dep=gpu_arrays['r07_camera1_depth']
    extras={k:v for k,v in gpu_arrays.items() if not k.startswith('r07_camera1_')}
   if acquisition_timing_enabled:acquisition_stages["readback_mask_done"]=time.monotonic_ns()
   # Synthetic packet time is separate from the acquisition monotonic clock.
   frame_arrays=dict(run_id=np.array(capture_run_id),capture_monotonic_ns=np.int64(frame_capture_monotonic_ns),rgb=rgb,depth=dep,frame_index=np.int64(frame),stamp_ns=np.int64(1_000_000_000+frame*50_000_000),clock_domain=np.array('synthetic_packet_20hz'),**extras)
   # These are rendered depth samples, including the robot's visible surface.
   # A self-masked zero never becomes free space; queries must stop at raw depth.
   # Existing perception/ROS consumers continue using their masked depth keys.
   frame_arrays.update(archive_geometry_fields())
   frame_arrays.update(clearance_depth_arrays)
   if capture_joint_epoch_enabled:
    capture_readback_return_ns=time.monotonic_ns()
    capture_encoder_after=robot.get_joint_positions()[arm].copy()
    if not np.array_equal(capture_encoder_raw[arm],capture_encoder_after):raise ValueError("Native arm encoder changed during capture")
    frame_arrays.update(encoder_read_start_monotonic_ns=np.int64(frame_capture_monotonic_ns),encoder_read_end_monotonic_ns=np.int64(capture_encoder_read_end_ns),arm_feedback_capture_monotonic_ns=np.int64((frame_capture_monotonic_ns+capture_encoder_read_end_ns)//2),render_start_monotonic_ns=np.int64(capture_render_start_ns),render_return_monotonic_ns=np.int64(render_return_ns),readback_return_monotonic_ns=np.int64(capture_readback_return_ns),physics_step_before=np.int64(capture_physics_step),physics_step_after=np.int64(world.current_time_step_index),arm_encoder=np.rint(capture_encoder_raw[arm]*4096/(2*np.pi))*2*np.pi/4096,arm_encoder_after_render=np.rint(capture_encoder_after*4096/(2*np.pi))*2*np.pi/4096)
   if ownership_audit:
    source_unchanged=all(hashlib.sha256(raw.tobytes()).hexdigest()==digest for raw,digest in source_views)
    if not source_unchanged:raise ValueError('Masking changed annotator source')
    prior_snapshot=frame_arrays;prior_hash=snapshot_hash(frame_arrays)
    ownership_rows.append({'frame':frame,'source_arrays':len(source_views),'source_unchanged':source_unchanged,'survived_next_render':None})
    atomic_json(out/'ownership_audit.json',ownership_rows)
   if shared_frame_enabled:
    shared_frame_descriptor=shared_frame_slot.publish(frame_arrays)
   else:
    with (frame_file_path.with_suffix('.npz.tmp')).open('wb') as frame_file:np.savez(frame_file,**frame_arrays)
    os.replace(frame_file_path.with_suffix('.npz.tmp'),frame_file_path)
   if acquisition_timing_enabled:acquisition_stages["npz_done"]=time.monotonic_ns()
  pending_perception_audit=None;pending_perception_record=None
  if perception_enabled and frame%perception_interval==0:
   request={'frame':str(frame_file_path),'run_id':capture_run_id,'frame_index':frame,'clock_domain':'synthetic_packet_20hz','capture_monotonic_ns':frame_capture_monotonic_ns,'stamp_ns':1_000_000_000+frame*50_000_000,'max_age_ns':perception_max_age_ns,'detect':frame%20==0,'camera':['r07_camera1','r07_camera2','r07_camera3','arm_camera'][(frame//20)%4]}
   if os.getenv('R12_WRIST_INSPECTION','0')=='1' and frame>0:
    from arm_practice.wrist_perception_request import wrist_schedule_feedback
    from arm_practice.perception_schedule import select_perception_work
    work=select_perception_work(frame,**wrist_schedule_feedback(response,run_id=capture_run_id,stamp_ns=request['stamp_ns'],now_ns=time.monotonic_ns()))
    request.update(camera=work['camera'],detect=work['detect'])
   worker.stdin.write(json.dumps(request)+'\n');worker.stdin.flush()
   perception=perception_reader.receive(lambda obj:'stamp_ns' in obj,perception_timeout_s)
   # Preserve timing of rejected responses without using them as control observations.
   response_received_ns=time.monotonic_ns()
   try:perception['provenance_check']=require_response(request,perception,perception_max_age_ns)
   except ValueError as error:
    atomic_json(out/'rejected_perception.json',{'error':str(error),'frame_index':frame,'capture_monotonic_ns':frame_capture_monotonic_ns,'response_received_monotonic_ns':response_received_ns,'validation_failed_monotonic_ns':time.monotonic_ns(),'max_age_ns':perception_max_age_ns,'acquisition_stages_monotonic_ns':acquisition_stages,'readback_parts_ns':readback_parts,'response':perception})
    raise
 
   pending_perception_audit={'frame_index':frame,'capture_monotonic_ns':frame_capture_monotonic_ns,'response_received_monotonic_ns':response_received_ns,'age_ns':response_received_ns-frame_capture_monotonic_ns,'max_age_ns':perception_max_age_ns,'request_received_monotonic_ns':perception.get('request_received_monotonic_ns'),'inference_started_monotonic_ns':perception.get('inference_started_monotonic_ns'),'inference_completed_monotonic_ns':perception.get('inference_completed_monotonic_ns')}
   pending_perception_record=dict(perception)
  if perception_enabled:
   perception=fresh_response(request,perception,perception_max_age_ns)
  panel_bbox=perception.get('barcode_boxes',[None])[0] if perception.get('barcode_boxes') else None
  import cv2
  signal_color='UNKNOWN'
  if panel_bbox is not None:
   x0,y0,x1,y1=panel_bbox;h=max(1,y1-y0)
   crop=rgb[0][max(0,int(y0-3*h)):max(0,int(y0)),max(0,int(x0)):min(640,int(x1))]
   if crop.size:
    hsv=cv2.cvtColor(crop,cv2.COLOR_RGB2HSV);bright=(hsv[:,:,1]>150)&(hsv[:,:,2]>180)
    votes={c:int(np.count_nonzero(bright&mask)) for c,mask in [('R',(hsv[:,:,0]<10)|(hsv[:,:,0]>170)),('Y',(hsv[:,:,0]>=20)&(hsv[:,:,0]<=40)),('G',(hsv[:,:,0]>=45)&(hsv[:,:,0]<=90))]}
    lit=[c for c,n in votes.items() if n>=12];signal_color=lit[0] if len(lit)==1 else ('OFF' if not lit else 'UNKNOWN')
  if native_producer_enabled:native_current_feedback=native_producer.current_feedback()
  encoder_packet_raw=capture_encoder_raw if capture_joint_epoch_enabled else robot.get_joint_positions()
  measured=np.rint(encoder_packet_raw*4096/(2*np.pi))*2*np.pi/4096
  measured[grip]=np.rint(encoder_packet_raw[grip]/1e-5)*1e-5  # provisional 10um linear readout, not motor angular ticks
  if latest_mode:measured[wheel]=np.rint(encoder_packet_raw[wheel]/4*4096/(2*np.pi))*2*np.pi/4096*4
  # Continuous wheel joints may wrap; shortest angular difference per 50 ms is unambiguous at <=10 rad/s.
  packet_velocity_valid=previous is not None if native_producer_enabled else frame>0
  packet_dt=float(world.current_time)-packet_previous_native_time if native_producer_enabled and packet_previous_native_time is not None else .05
  if packet_dt<=0:raise ValueError('Invalid native packet encoder interval')
  velocity=np.zeros_like(measured) if previous is None else ((measured-previous+np.pi)%(2*np.pi)-np.pi)/packet_dt;previous=measured.copy()
  if native_producer_enabled:packet_previous_native_time=float(world.current_time)
  encoder_wheel_velocity=velocity[wheel].copy()
  readings=[]
  for samples in imu_samples:
   try:readings.append(reduce_window(samples))
   except ValueError as error:readings.append({'valid':False,'gyro':[0.,0.,0.],'accel':[0.,0.,0.],'error':str(error),'native_times':[s['time'] for s in samples]})
  packet_parts={}
  packet_part_start=time.monotonic_ns()
  base_matrix=np.array(UsdGeom.Xformable(stage.GetPrimAtPath('/World/Spawn/R12/base_link')).ComputeLocalToWorldTransform(0)).T
  scans={}
  for name,local in lidar_T.items():
   transform=base_matrix@local;values=[]
   for angle in np.linspace(-np.pi/2,np.pi/2,90):
    direction=transform[:3,:3]@np.array([np.cos(angle),np.sin(angle),0]);hits=[]
    def hit(h):
     if '/R12/' not in str(h.rigid_body):hits.append(float(h.distance))
     return True
    query.raycast_all(tuple(transform[:3,3]),tuple(direction),6.,hit);values.append(min(hits) if hits else 6.)
   scans[name]=values
  packet_parts['lidar_ns']=time.monotonic_ns()-packet_part_start
  packet={'capture_run_id':capture_run_id,'frame_capture_monotonic_ns':frame_capture_monotonic_ns,'scans':scans,'stamp_ns':1_000_000_000+frame*50_000_000,'imu':readings,'names':names,'position':measured.tolist(),'velocity':velocity.tolist(),'velocity_valid':packet_velocity_valid}
  packet_part_start=time.monotonic_ns()
  camera_pose=prepared_camera_rig.transforms(dict(zip(names,measured))) if prepared_camera_rig is not None else camera_transforms(root,dict(zip(names,measured)))
  packet_parts['camera_fk_ns']=time.monotonic_ns()-packet_part_start
  packet['camera_transforms']={k:v.tolist() for k,v in camera_pose.items()};packet['gripper_aperture_m']=float(grip_cfg['cad_zero_aperture_m']+robot.get_joint_positions()[grip[0]]-robot.get_joint_positions()[grip[1]])
  packet['perception']=perception;packet['signal_color']=signal_color
  if native_producer_enabled:
   native_timing=native_producer.native_packet_timing()
   native_producer.publish_native_packet(native_timing,native_current_feedback)
   packet.update(stamp_ns=native_timing['stamp_ns'],clock_domain=native_timing['clock_domain'],imu=native_timing['imu'],native_packet_timeline=native_timing)
   packet['native_capture_diagnostic']=native_producer.packet_summary(native_current_feedback)
   packet['legacy_image_publication_disabled']=True
   packet['perception_capabilities']={'decoder_started':False,'barcode_started':False,'object_detection_started':False,'pose_started':False}
 
  if capture_joint_epoch_enabled:
   from arm_practice.capture_joint_epoch import EPOCH_FIELDS
   packet['arm_capture_epoch']={key:np.asarray(frame_arrays[key]).tolist() for key in EPOCH_FIELDS}
   packet['arm_capture_epoch'].update(run_id=capture_run_id,stamp_ns=packet['stamp_ns'])
  packet['signal_observation']={'run_id':capture_run_id,'stamp_ns':packet['stamp_ns'],'capture_monotonic_ns':frame_capture_monotonic_ns,'camera':'r07_camera1','clock_domain':'synthetic_packet_20hz','color':signal_color,'roi_source':{'run_id':perception.get('run_id'),'stamp_ns':perception.get('stamp_ns'),'capture_monotonic_ns':perception.get('capture_monotonic_ns'),'camera':'r07_camera1'} if panel_bbox is not None else None}
  if native_producer_enabled:
   packet['signal_observation']={'run_id':capture_run_id,'stamp_ns':native_timing['capture_stamp_ns'],'capture_monotonic_ns':frame_capture_monotonic_ns,'camera':'r07_camera1','clock_domain':native_timing['clock_domain'],'color':'UNKNOWN','valid':False,'reason':'native_producer_diagnostic_no_signal_detector'}
  packet['fault']=None
  if os.getenv('R12_FAULT_AUDIT','0')=='1':
   for start,kind in [(200,'imu0'),(230,'imu1'),(260,'encoder'),(290,'camera'),(320,'lidar')]:
    if start<=frame<start+10:packet['fault']=kind
   if packet['fault'] in ['imu0','imu1']:packet['imu'][int(packet['fault'][-1])]['valid']=False
   if packet['fault']=='encoder':packet['velocity_valid']=False
   if packet['fault']=='lidar':packet['scans'].pop('rear_left')
  if acquisition_timing_enabled:
   acquisition_stages["packet_ready"]=time.monotonic_ns();packet["acquisition_stages_monotonic_ns"]=acquisition_stages;packet["readback_parts_ns"]=readback_parts;packet["packet_parts_ns"]=packet_parts
  if shared_frame_enabled:packet['shared_frame_descriptor']=shared_frame_descriptor
  sock.sendall((json.dumps(packet)+'\n').encode());response=json.loads(reader.readline());command=np.array(response.get('command',[0,0,0]));wheel_action=np.array(response.get('wheel_action',[0,0,0,0]))*residual_scale;wheel_scale=float(response.get('wheel_scale',1.));emergency=response.get('motion_envelope',{}).get('reason') in ['invalid_sensor_or_time','tilt_stop','await_gravity','invalid_visual_attitude','attitude_disagreement'] or response.get('lidar_guard')=='invalid_sensor'
  if response.get('cancel_previous_arm_target') is True:
   # pa28 v47 demonstrated a torque discontinuity with this implicit drive.
   # Stop the simulation before another physics step; do not continue the old
   # trajectory or claim measured-q replacement is a validated holding action.
   atomic_json(out/'fault_hold_rejected.json',{'reason':'implicit_hold_transition_unvalidated','frame':frame,'physics_step':int(world.current_time_step_index),'next_physics_step_authorized':False})
   raise RuntimeError('Implicit fault-hold transition unvalidated; simulation stopped before next physics step')
  response['applied_allocator_input']=allocator_input
  if acquisition_timing_enabled:response['sim_response_received_monotonic_ns']=time.monotonic_ns()
  if shared_frame_enabled:shared_frame_slot.acknowledge(response.get('shared_frame_ack'))
  if command_delay is not None:
   if np.any(command!=0):raise RuntimeError('Nonzero response rejected by stationary delay probe')
   if delay_submit_frames<0 or frame<delay_submit_frames:
    accepted=command_delay.submit(run_id=capture_run_id,sequence=frame,capture_ns=packet['stamp_ns'],now_ns=packet['stamp_ns'],velocity=command)
    if not accepted:raise RuntimeError('Delay queue rejected response: '+str(command_delay.fault))
  if stationary_capture:command=np.zeros(3);wheel_action=np.zeros(4);wheel_scale=1.;response.update(stationary_wheel_hold_torque=[0.]*4,stop_brake_valid=False)
  stationary_hold_torque=np.asarray(response.get('stationary_wheel_hold_torque',[0.]*4),float)
  if stationary_hold_torque.shape!=(4,) or not np.isfinite(stationary_hold_torque).all():stationary_hold_torque=np.zeros(4)
  stationary_hold_torque=np.clip(stationary_hold_torque,-.04,.04)
  stop_brake_valid=bool(response.get('stop_brake_valid',False));stop_brake_gyro_z=response.get('stop_brake_gyro_z')
  if not stationary_capture and response.get('gripper_width_m') is not None:grip_width=float(np.clip(response['gripper_width_m'],.020,.075))
  signal_ready=bool(response.get('signal_ready',False))
  # Diagnostic disk writes occur after the control response, not before observation consumption.
  if native_producer_enabled:native_producer.finish_frame(frame)
  if pending_perception_audit is not None:
   with (out/'perception_transport_audit.jsonl').open('a') as f:f.write(json.dumps(pending_perception_audit)+'\n')
   with (out/'perception.jsonl').open('a') as f:f.write(json.dumps(pending_perception_record)+'\n')
  if not stationary_capture and response.get('arm_target') is not None:q[arm]=response['arm_target']
  # Actual base pose is evaluator-only, never transmitted to the ROS estimator/controller.
  T=UsdGeom.Xformable(stage.GetPrimAtPath('/World/Spawn/R12/base_link')).ComputeLocalToWorldTransform(0);logs.append({'frame':frame,'actual_base':list(T.ExtractTranslation()),'actual_rpy_deg':Rotation.from_matrix(np.array(T)[:3,:3].T.copy()).as_euler('xyz',degrees=True).tolist(),'contact_count':len(contacts),'imu_window':readings,'diagnostic_lidar_scans':scans if frame%10==0 else None,'drive_telemetry':{'physics_hz':physics_hz,'imu_hz':120,'encoder_feedback120':encoder_feedback120,'rolling_projection':rolling_projection,'coordination':coordination_status,'ppo_residual_scale':residual_scale,'low_speed_residual_fraction':low_speed_residual_fraction,'transition_braking':transition_braking.tolist() if latest_mode else [],'integral_torque_Nm':integral_torque.tolist(),'substeps':drive_substeps,'steer_position_rad':measured[steer].tolist(),'steer_target_rad':q[steer].tolist(),'wheel_velocity_rad_s':velocity[wheel].tolist(),'wheel_target_rad_s':vel[wheel].tolist()},**response})
  history_stride=int(os.getenv('R12_OBSERVATION_HISTORY_STRIDE','0'))
  if history_stride>0 and frame%history_stride==0 and response.get('arm_feedback_valid',False):
   from arm_practice.observed_frame_archive import archive_observed_frame
   archive_observed_frame(frame_file_path,out/'observation_history',response,frame)
  if terminal_gate is not None:
   terminal_status=terminal_gate.update(response,time.monotonic_ns())
   logs[-1]['terminal_observation']=terminal_status
   if terminal_status['settled']:
    from arm_practice.observed_frame_archive import archive_observed_frame
    archive_observed_frame(frame_file_path,out/'terminal_observation',response,frame)
    terminal_captured=True
  if video is not None:video.write(rgb,extras,frame,response)
  if frame%100==0:
   print('SENSOR_FRAME',frame,response,flush=True)
   if result_journal is not None:result_journal.checkpoint(logs)
   else:atomic_json(out/'sim_results.json',logs)
   atomic_json(out/'contacts.json',contacts)
  if (response.get('finished') and (terminal_gate is None or terminal_captured)) or response.get('diagnostic_stop') or (out/'STOP').exists():break
 if native_producer_enabled:native_producer.close()
 if command_delay is not None:atomic_json(out/'command_delay_audit.json',command_delay_rows)
 if video is not None:
  video.close();atomic_json(out/'video_manifest.json',{'frames':video.frames,'fps':20,'file':'mission.mp4','views':'top3+arm1','scope':'inventory survey, not pick/place'})
 if gpu_acquisition and gpu_validate:atomic_json(out/'gpu_acquisition_validation.json',gpu_validation_rows)
 if ownership_audit:atomic_json(out/'ownership_audit.json',ownership_rows)
 if shared_frame_enabled:
  with (frame_file_path.with_suffix('.npz.tmp')).open('wb') as frame_file:np.savez(frame_file,**frame_arrays)
  os.replace(frame_file_path.with_suffix('.npz.tmp'),frame_file_path);shared_frame_slot.close()
 if hydra_stats is not None:atomic_json(out/'hydra_profile.json',{'scope':'Raw Hydra profiler snapshot after render; frame correspondence and nested-duration additivity not established','samples':hydra_profile_rows})
 if native_producer_enabled and not logs:raise RuntimeError('No current native RGBD epoch acquired in bounded mission entrypoint trial')
 np.save(out/'last_rgb.npy',rgb);np.save(out/'last_depth.npy',dep)
 if result_journal is not None:result_journal.checkpoint(logs)
 atomic_json(out/'sim_results.json',logs)
 if result_journal is not None:result_journal.close()
 atomic_json(out/'dynamics_support.json',dynamics_support);atomic_json(out/'capture_stage_timings.json',native_producer.capture.stage_timings);atomic_json(out/'issuer_stage_timings.json',native_producer.issuer.diagnostic_timings);atomic_json(out/'hold_control_timeline.json',hold_timeline);atomic_json(out/'contacts.json',contacts);(out/'completion.json').write_text(json.dumps({'terminal_observation_requested':terminal_gate is not None,'terminal_observation_captured':terminal_captured,'frames':len(logs),'finished':bool(response.get('finished')),'stopped_by_file':(out/'STOP').exists()}));print('R12_SENSOR_SIM_COMPLETE',flush=True);atomic_json(out/'step_gc_events.json',step_gc_events);atomic_json(out/'simulate_timings.json',simulate_timings);world.stop();app.close()
finally:
 atomic_json(out/'step_gc_events.json',step_gc_events)
 atomic_json(out/'simulate_timings.json',simulate_timings)
 world._physics_context._physics_sim_interface=original_sim_interface
