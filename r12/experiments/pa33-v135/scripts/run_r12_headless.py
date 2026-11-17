"""Longer drive/perception diagnostic with staged timings and immutable sensor archives."""
import argparse,ast,fcntl,os,json,time,subprocess,signal,socket,tempfile,threading
from pathlib import Path
import rclpy
parser=argparse.ArgumentParser(description="Isolated headless R12 drive/perception trial; cameras remain rendered")
parser.add_argument('--run',required=True)
parser.add_argument('--domain',type=int,required=True)
parser.add_argument('--port',type=int,required=True)
parser.add_argument('--frames',type=int,default=12000)
parser.add_argument('--stationary-capture',action='store_true',help='Zero-command sensor diagnostic; disables mission and motion at both bridge and simulator')
parser.add_argument('--cruise-speed',type=float,default=.12,help='Requested translation ceiling in m/s; obstacle, bend, acceleration and stopping limits still apply')
parser.add_argument('--wrist-inspection',action='store_true',help='S1 stationary lower/view/refold simulation diagnostic; no grasp')
parser.add_argument('--low-speed-residual-fraction',type=float,default=None)
parser.add_argument('--residual-scale',type=float,default=1.,help='Simulation A/B scale for compatible PPO wheel residual; checkpoint remains unchanged')
parser.add_argument('--wheel-kd',type=float,default=.02,help='Simulation wheel speed feedback gain; existing torque and speed limits unchanged')
parser.add_argument('--imu-latest-sampling',action='store_true',help='Read native physics IMU on the exact120Hz substep boundary')
parser.add_argument('--explicit-base-root',action='store_true',help='Opt-in root-only RL overlay; separate physics compatibility diagnostic')
parser.add_argument('--oriented-live-path',action='store_true',help='Unvalidated opt-in live Nav2/nvBlox heading adapter; fail closed without matching fresh maps')
parser.add_argument('--continuous-path',action='store_true',help='Experimental continuous holonomic reference with filtered-motion preview')
parser.add_argument('--planner-stop-hold',action='store_true',help='Opt-in encoder hold during healthy zero-command planner stops; fixed torque limits retained')
parser.add_argument('--approach-radius',type=float,default=.10,help='Coarse navigation region for continuous path mode; does not certify arm reach')
parser.add_argument('--goal-stop-hold',action='store_true')
parser.add_argument('--goal-capture-radius',type=float,default=.02)
parser.add_argument('--async-path-smoothing',action='store_true')
parser.add_argument('--prepared-camera-rig',action='store_true')
parser.add_argument('--incremental-result-logs',action='store_true')
parser.add_argument('--single-station',choices=['S1'],help='Stop at S1; do not continue the coverage route')
parser.add_argument('--clearance-depth',action='store_true',help='Archive sensor depth before robot self masking; no clearance authorization')
parser.add_argument('--dds-profile',type=Path)
parser.add_argument('--rviz',action='store_true',help='Show RViz2 only; Isaac Sim remains headless')
parser.add_argument('--record-video',action='store_true',help='Record the existing four RGB cameras locally at simulation-time20fps')
parser.add_argument('--rviz-after-tracking',action='store_true',help='Delay GUI subscribers until cuVSLAM has produced valid tracking')
parser.add_argument('--rviz-domain',type=int,help='Isolate RViz using a one-way sensor relay to this domain')
args=parser.parse_args()
if not args.run.replace('_','').replace('-','').isalnum():parser.error('run must be a simple directory name')
if not 0<=args.domain<=101:parser.error('domain must be 0..101 for bounded UDP port allocation')
if args.rviz_domain is not None and (not args.rviz or not 0<=args.rviz_domain<=101 or args.rviz_domain==args.domain):parser.error('RViz domain must be distinct,0..101,and requires --rviz')
if not 1024<=args.port<=65535 or not 1<=args.frames<=12000:parser.error('invalid port or frames')
if not .01<=args.cruise_speed<=.899795667:parser.error('cruise speed must be finite and within0.01..0.899795667m/s')
if args.low_speed_residual_fraction is not None and not 0<=args.low_speed_residual_fraction<1:parser.error('low-speed residual fraction must be in [0,1)')
if not 0<=args.residual_scale<=1:parser.error('residual scale must be finite and within [0,1]')
if not .01<=args.wheel_kd<=.04:parser.error('wheel feedback gain must be within tested comparison range0.01..0.04')
if not .05<=args.approach_radius<=.20:parser.error('coarse approach radius must be5..20cm')
if not 0<args.goal_capture_radius<=.02:parser.error('capture radius must be within the unchanged2cm arrival radius')
if args.goal_capture_radius!=.02 and not args.goal_stop_hold:parser.error('capture radius requires goal stop hold')
if args.oriented_live_path and not (args.continuous_path and args.planner_stop_hold):parser.error('oriented live paths require --continuous-path and --planner-stop-hold')
if args.stationary_capture and (args.wrist_inspection or args.single_station or args.oriented_live_path):parser.error('Stationary capture cannot request wrist inspection, station travel or live paths')
if args.wrist_inspection and args.single_station!='S1':parser.error('Wrist inspection currently requires single-station S1')
repo=Path(__file__).resolve().parents[1];out=repo.parents[1]/'outputs/r12'/args.run;out.mkdir(exist_ok=False)
lock_dir=out.parent/'runtime_locks';lock_dir.mkdir(exist_ok=True)
locks=[]
for resource in [f'domain_{args.domain}',f'port_{args.port}']+([f'domain_{args.rviz_domain}'] if args.rviz_domain is not None else []):
 lock=(lock_dir/(resource+'.lock')).open('a')
 try:fcntl.flock(lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
 except BlockingIOError:raise SystemExit(f'Another headless R12 launcher owns {resource}')
 locks.append(lock)  # Retain the locks until this launcher exits.
from r12_resource_admission import check,snapshot
from r12_owned_memory import sample_owned
from r12_owned_session import OwnedSession
import shutil
admission=check(out/'resource_admission.json',minimum_available_gib=4)
if not admission['admitted']:raise SystemExit('Resource admission deferred: '+str(admission['reasons']))
if args.dds_profile is not None:
 profile=out/'transport.xml';profile.write_bytes(args.dds_profile.read_bytes())
 os.environ['FASTDDS_DEFAULT_PROFILES_FILE']=str(profile)
 # Preserve the last working Isaac ROS participant configuration. This installed
 # rmw build still reports using the legacy variable for participant defaults.
 os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']=str(profile)
 os.environ.pop('FASTDDS_BUILTIN_TRANSPORTS',None)
env=os.environ.copy()
for key in list(env):
 if key.startswith('R12_'):env.pop(key)
env.update(ROS_DOMAIN_ID=str(args.domain),R12_SENSOR_PORT=str(args.port),R12_SENSOR_RUN=str(out),R12_CLOUD_PUBLICATION_AUDIT='1',R12_STATIONARY_CAPTURE='0',R12_MISSION='1',R12_SENSOR_MOTION='1',R12_MISSION_PERCEPTION='1',R12_PERCEPTION_BACKEND='tensorrt_fp16',R12_TRT_MODEL_DIR=str(repo.parents[1]/'outputs/r12/v401_tensorrt'),R12_GUI='0',R12_FORCE_AA_OFF='1',R12_PERCEPTION_INTERVAL_FRAMES='1',R12_BOUND_VISUAL_WAIT_TO_FRESHNESS='1',R12_IMAGE_ARRAY_BUFFER='1',R12_SIGNAL_AT_RECEIVE='1',R12_FRAME_MEMBER_CACHE='1',R12_SINGLE_RGB_STACK_COPY='1',R12_FAST_SLICE_COUNT='1',R12_USE_LATEST_PPO='1',R12_USE_PPO='0',R12_CRUISE_SPEED='0.12',R12_WHEEL_INTEGRAL='1',R12_ENCODER_FEEDBACK120='1',R12_GOAL_OBSERVATION_DWELL_S='4.5',R12_ACQUISITION_TIMING='1',R12_PUBLICATION_OPERATION_TIMING='1',R12_OBSERVATION_HISTORY_STRIDE='100',R12_RECORD_VIDEO='0',R12_PHYSICS_HZ='960',R12_SIM_SENSOR_WAIT_S='1.0')
if args.stationary_capture:
 env.update(R12_STATIONARY_CAPTURE='1',R12_MISSION='0',R12_SENSOR_MOTION='0')
env.update(CUDA_VISIBLE_DEVICES='0',__NV_PRIME_RENDER_OFFLOAD='1',__GLX_VENDOR_LIBRARY_NAME='nvidia',__VK_LAYER_NV_optimus='NVIDIA_only')
env['R12_RECORD_VIDEO']='1' if args.record_video else '0'
env['R12_CRUISE_SPEED']=str(args.cruise_speed)
env['R12_WRIST_INSPECTION']='1' if args.wrist_inspection else '0'
env['R12_PPO_RESIDUAL_SCALE']=str(args.residual_scale)
env['R12_WHEEL_KD']=str(args.wheel_kd)
env['R12_IMU_LATEST_SAMPLING']='1' if args.imu_latest_sampling else '0'
env['R12_CONTINUOUS_PATH']='1' if args.continuous_path else '0'
env['R12_EXPLICIT_BASE_ROOT']='1' if args.explicit_base_root else '0'
env['R12_ORIENTED_LIVE_PATH']='1' if args.oriented_live_path else '0'
env['R12_PLANNER_STOP_HOLD']='1' if args.planner_stop_hold else '0'
env['R12_APPROACH_RADIUS_M']=str(args.approach_radius)
if args.low_speed_residual_fraction is not None:env['R12_PPO_LOW_SPEED_FRACTION']=str(args.low_speed_residual_fraction)
if args.goal_stop_hold:env['R12_GOAL_STOP_HOLD']='1'
env['R12_GOAL_CAPTURE_RADIUS_M']=str(args.goal_capture_radius)
env['R12_CALLBACK_DURATION_TIMING']='1'
if args.async_path_smoothing:env['R12_ASYNC_PATH_SMOOTHING']='1'
if args.prepared_camera_rig:env['R12_PREPARED_CAMERA_RIG']='1'
if args.incremental_result_logs:env['R12_INCREMENTAL_RESULT_LOGS']='1'
if args.clearance_depth:env['R12_ARCHIVE_CLEARANCE_DEPTH']='1'
if args.single_station:env['R12_TERMINAL_OBSERVATION']='1'
os.environ['ROS_DOMAIN_ID']=str(args.domain);rclpy.init();preflight_name=f'headless_preflight_{os.getpid()}';n=rclpy.create_node(preflight_name,enable_rosout=False)
try:
 for _ in range(10):rclpy.spin_once(n,timeout_sec=.1)
 assert all(name==preflight_name for name,ns in n.get_node_names_and_namespaces()),'domain occupied'
finally:n.destroy_node();rclpy.shutdown()
if args.rviz_domain is not None:
 os.environ['ROS_DOMAIN_ID']=str(args.rviz_domain);rclpy.init();n=rclpy.create_node(preflight_name,enable_rosout=False)
 try:
  for _ in range(10):rclpy.spin_once(n,timeout_sec=.1)
  assert all(name==preflight_name for name,ns in n.get_node_names_and_namespaces()),'RViz domain occupied'
 finally:n.destroy_node();rclpy.shutdown();os.environ['ROS_DOMAIN_ID']=str(args.domain)
with socket.socket() as sock:sock.bind(('127.0.0.1',args.port))
shared_directory=Path(tempfile.mkdtemp(prefix='r12-frame-',dir='/dev/shm'))
env['R12_FRAME_FILE']=str(shared_directory/'frame.npz')
(out/'launch_configuration.json').write_text(json.dumps({'run':args.run,'domain':args.domain,'port':args.port,'gui':False,'rviz':args.rviz,'record_video':args.record_video,'frame_file':env['R12_FRAME_FILE'],'environment':{k:v for k,v in env.items() if k.startswith('R12_') or k in ('ROS_DOMAIN_ID','ROS_AUTOMATIC_DISCOVERY_RANGE','FASTDDS_BUILTIN_TRANSPORTS','FASTDDS_DEFAULT_PROFILES_FILE','FASTRTPS_DEFAULT_PROFILES_FILE','CUDA_VISIBLE_DEVICES','__NV_PRIME_RENDER_OFFLOAD','__GLX_VENDOR_LIBRARY_NAME','__VK_LAYER_NV_optimus')}},indent=2))
 # RAM-backed NPZ retains the existing synchronous frame identity protocol.
route=json.loads((repo/'models/navigation/routes.json').read_text());route.pop('observation_posture',None)
if args.single_station:
 route['steps']=[{'station':args.single_station,'phase':'wrist_clearance_sensor_capture'}]
 route['routes']=[]
 route['scope']='One stationary S1 sensor archive for wrist clearance; no arm unfolding or pickup yet'
if args.wrist_inspection:
 route['observation_posture']={'arm_q':[0.,-.33,-.175,-.33,.30],'cycle':'live_surface_guarded_inspection','exposure_seconds':5.}
 route['scope']='S1 drive/stop/lower wrist camera/inspect/refold; observed-surface guard, unobserved clearance not certified; no pick/place'
(out/'survey_route.json').write_text(json.dumps(route));env['R12_ROUTE_FILE']=str(out/'survey_route.json')
children=[];child_labels={};owned_sessions=[];logs=[];report={'scope':'Simulation mission perception inventory and navigation survey, not pick/place; headless, no RViz; cameras and point clouds retained; speed ceiling0.12m/s; compatible v2 policy performance NOT accepted','frames_requested':args.frames,'gui':False,'rviz':False,'ros_domain_id':args.domain,'port':args.port,'launcher_pid':os.getpid(),'run_completed_normally':False,'control_comparison':{'bounded_wheel_integral':True,'quantized_encoder_feedback_hz':120,'policy_file_unchanged':True,'speed_ceiling_m_s':0.12,'baseline':'v410_mission'},'launcher_executes_unit_tests':False,'dds_profile':env.get('FASTDDS_DEFAULT_PROFILES_FILE')}
report['control_comparison'].update(low_speed_residual_fraction=args.low_speed_residual_fraction,baseline='v412_headless' if args.low_speed_residual_fraction is not None else 'v410_mission')
if args.goal_stop_hold:report['control_comparison'].update(goal_stop_hold=True,baseline='v413_headless_lowspeed_bound')
report['control_comparison'].update(goal_capture_radius_m=args.goal_capture_radius,arrival_radius_m=.02,async_path_smoothing=args.async_path_smoothing)
report['prepared_camera_rig']=args.prepared_camera_rig
report['incremental_result_logs']=args.incremental_result_logs
report.update(single_station=args.single_station,unmasked_sensor_depth_archive=args.clearance_depth)
report.update(rviz=args.rviz,record_video=args.record_video,scope='Isaac Sim headless driving and perception inspection; RViz2 optional GUI; top3+arm1 sensor video; no pick/place or gripper closing; wrist inspection cycle not validated; speed ceiling0.12m/s; policy performance NOT accepted')
report['rviz_domain']=args.rviz_domain if args.rviz_domain is not None else args.domain
report['control_comparison']['speed_ceiling_m_s']=args.cruise_speed
report['scope']=f'Isaac Sim headless driving and perception inspection; RViz2 optional GUI; top3+arm1 sensor video; no pick/place or gripper closing; wrist inspection cycle not validated; requested speed ceiling {args.cruise_speed:g}m/s, subject to existing sensor and motion limits; policy performance NOT accepted'
report['stationary_capture']=args.stationary_capture
if args.stationary_capture:report['scope']='Zero-command stationary sensor diagnostic; startup dynamics retained; no navigation or manipulation success claimed'
report['wrist_inspection_requested']=args.wrist_inspection
report['control_comparison'].update(residual_scale=args.residual_scale,continuous_path=args.continuous_path,imu_latest_sampling=args.imu_latest_sampling)
report['control_comparison']['wheel_kd']=args.wheel_kd
report['control_comparison']['planner_stop_hold']=args.planner_stop_hold
report['control_comparison']['explicit_base_root']=args.explicit_base_root
report['control_comparison']['oriented_live_path']=args.oriented_live_path
if args.continuous_path:report['control_comparison'].update(arrival_radius_m=args.approach_radius,capture_radius_m=args.approach_radius,arrival_scope='Coarse navigation region, not arm reach or manipulation acceptance')
last_owned_refresh=0.
def refresh_owned(force=False):
 global last_owned_refresh
 if force or time.monotonic()-last_owned_refresh>=.5:
  for session in owned_sessions:session.refresh()
  last_owned_refresh=time.monotonic()
def spawn(cmd,name,e=env):
 f=(out/(name+'.log')).open('x');logs.append(f)
 p=subprocess.Popen(cmd,cwd=repo,env=e,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);children.append(p);child_labels[p.pid]=name
 try:owned_sessions.append(OwnedSession(p))
 except BaseException:
  if p.poll() is None:
   p.terminate()
   try:p.wait(timeout=5)
   except subprocess.TimeoutExpired:p.kill();p.wait(timeout=5)
  raise
 refresh_owned(force=True)
 return p
def shm_inventory():
 result={}
 for path in Path('/dev/shm').iterdir():
  try:
   stat=path.stat()
   if path.is_file():result[path.name]={'inode':stat.st_ino,'uid':stat.st_uid,'allocated_bytes':stat.st_blocks*512}
  except OSError:pass
 return result
shm_before=shm_inventory()
(out/'shm_before.json').write_text(json.dumps(shm_before))
telemetry_stop=threading.Event()
def record_resources():
 with (out/'resource_samples.jsonl').open('x') as file:
  while not telemetry_stop.is_set():
   try:
    row=snapshot();row['owned_processes']=sample_owned([p.pid for p in children if p.poll() is None])
    gpu=subprocess.run(['nvidia-smi','--query-gpu=memory.used,memory.total,utilization.gpu','--format=csv,noheader,nounits'],capture_output=True,text=True,timeout=3)
    row['gpu_csv_mib_percent']=gpu.stdout.strip() if gpu.returncode==0 else None
   except Exception as error:row={'error':str(error),'monotonic_ns':time.monotonic_ns()}
   file.write(json.dumps(row)+'\n');file.flush()
   telemetry_stop.wait(2)
telemetry_thread=threading.Thread(target=record_resources,daemon=True);telemetry_thread.start()
try:
 slam=spawn(['ros2','launch','configs/sensors/cuvslam.launch.py'],'cuvslam')
 mapping_process=spawn(['ros2','launch','configs/sensors/mapping.launch.py'],'mapping')
 bridge=spawn([str(repo/'.venv-r12/bin/python'),'scripts/r12_sensor_ros.py'],'bridge')
 deadline=time.monotonic()+30
 while time.monotonic()<deadline:
  refresh_owned()
  if bridge.poll() is not None:raise RuntimeError('bridge exited')
  if 'SENSOR_ROS_READY' in (out/'bridge.log').read_text():break
  time.sleep(.25)
 else:raise RuntimeError('bridge readiness timeout')
 rclpy.init();check=rclpy.create_node('headless_readiness',enable_rosout=False)
 try:
  deadline=time.monotonic()+30
  while time.monotonic()<deadline:
   refresh_owned()
   if mapping_process.poll() is not None:raise RuntimeError('mapping exited before readiness')
   rclpy.spin_once(check,timeout_sec=.1)
   names={name for name,ns in check.get_node_names_and_namespaces()}
   if {'ekf_filter_node','nvblox_node','r12_robot_state_publisher'}<=names:break
  else:raise RuntimeError('mapping node discovery timeout')
  report['mapping_nodes_discovered']=sorted(names)
 finally:check.destroy_node();rclpy.shutdown()
 observer=spawn([str(repo/'.venv-r12/bin/python'),'scripts/record_r12_clouds_v336.py',str(out/'cloud_delivery.json')],'cloud_observer')
 if args.rviz:
  import yaml
  rviz_config=yaml.safe_load((repo/'configs/sensors/r12_mission.rviz').read_text())
  rviz_config['Visualization Manager']['Global Options']['Frame Rate']=10
  for display in rviz_config['Visualization Manager']['Displays']:
   if display['Class']=='rviz_default_plugins/TF':display['Enabled']=True
   if display['Class'] in ('rviz_default_plugins/PointCloud2','rviz_default_plugins/LaserScan','rviz_default_plugins/Image'):
    display['Queue Size']=1
    display.setdefault('Topic',{}).update({'Reliability Policy':'Best Effort','History Policy':'Keep Last','Depth':1})
  rviz_path=out/'inspection.rviz';rviz_path.write_text(yaml.safe_dump(rviz_config,sort_keys=False))
  def start_rviz():
   rvizenv=env.copy()
   if args.rviz_domain is not None:
    spawn([str(repo/'.venv-r12/bin/python'),'scripts/r12_rviz_relay.py','--source-domain',str(args.domain),'--display-domain',str(args.rviz_domain),'--report',str(out/'rviz_relay.json')],'rviz_relay')
    rvizenv['ROS_DOMAIN_ID']=str(args.rviz_domain)
   spawn([str(repo/'.venv-r12/bin/python'),'scripts/r12_rviz_map.py'],'rviz_map',rvizenv)
   spawn(['rviz2','-d',str(rviz_path),'--ros-args','-p','use_sim_time:=true'],'rviz',rvizenv)
   report['rviz_started_monotonic_ns']=time.monotonic_ns()
  if not args.rviz_after_tracking:start_rviz()
 report['gui']=False;report['rviz']=args.rviz
 simenv=env.copy()
 for k in ['ROS_DISTRO','AMENT_PREFIX_PATH','PYTHONPATH','LD_LIBRARY_PATH']:simenv.pop(k,None)
 simenv['OMNI_KIT_ACCEPT_EULA']='YES'
 sim=spawn(['bash','scripts/r12_runtime.sh','python','scripts/r12_sensor_sim.py','--frames',str(args.frames),'--port',str(args.port)],'sim',simenv)
 report['pids']=[p.pid for p in children];(out/'running.json').write_text(json.dumps(report,indent=2))
 deadline=time.monotonic()+7200;pressure_strikes=0
 while sim.poll() is None:
  refresh_owned()
  if args.rviz and args.rviz_after_tracking and 'rviz_started_monotonic_ns' not in report:
   progress=out/'sim_results_progress.json'
   latest=json.loads(progress.read_text())['latest'] if progress.exists() else {}
   if latest.get('odom_received') and latest.get('tracking')==1:
    start_rviz();report['rviz_start_frame']=latest['frame'];report['pids']=[p.pid for p in children]
    (out/'running.json').write_text(json.dumps(report,indent=2))
  if mapping_process.poll() is not None or slam.poll() is not None:raise RuntimeError("Required mission process exited; stopping owned simulation")
  if bridge.poll() is not None and not (bridge.returncode==0 and (out/'completion.json').exists()):raise RuntimeError("Sensor bridge exited without simulator completion; stopping owned simulation")
  resource=snapshot()
  pressured=resource['mem_available_bytes']<2*1024**3 or resource['pressure']['memory']['full']['avg10']>10
  pressure_strikes=pressure_strikes+1 if pressured else 0
  if pressure_strikes>=3:
   report['resource_abort']=resource
   raise RuntimeError('Owned stationary trial stopped after three resource-pressure samples')
  if time.monotonic()>deadline:raise TimeoutError('Bounded simulator timeout7200s')
  time.sleep(2)
 bridge.wait(timeout=30)
 report['sim_exit']=sim.returncode
 report['completion_marker']='R12_SENSOR_SIM_COMPLETE' in (out/'sim.log').read_text()
 report['completion_file']=(out/'completion.json').exists()
 report['run_completed_normally']=bool(sim.returncode==0 and report['completion_marker'] and report['completion_file'])
 report['perception_rejected']=(out/'rejected_perception.json').exists()
except BaseException as error:
 report['launcher_error']={'type':type(error).__name__,'message':str(error)}
 raise
finally:
 refresh_owned(force=True)
 report['owned_session_cleanup']=[session.stop() for session in reversed(owned_sessions)]
 report['all_tracked_session_processes_exited']=len(owned_sessions)==len(children) and all(row['all_tracked_exited'] for row in report['owned_session_cleanup'])
 for session in owned_sessions:session.close()
 for f in logs:f.close()
 telemetry_stop.set();telemetry_thread.join(timeout=5)
 shm_after=shm_inventory()
 (out/'shm_after.json').write_text(json.dumps(shm_after))
 report['new_shm_names_observed']=[name for name,value in shm_after.items() if name not in shm_before or shm_before[name]['inode']!=value['inode']]
 report['shm_attribution_note']='Temporal difference only, not ownership proof; no DDS files deleted.'
 # Remove frame storage only after tracked same-session descendants have exited.
 frame_archive=shared_directory/'frame.npz'
 if frame_archive.exists():
  import shutil
  shutil.copy2(frame_archive,out/'frame.npz')
 if report['all_tracked_session_processes_exited']:
  shutil.rmtree(shared_directory)
 else:
  report['shared_directory_preserved_for_live_processes']=str(shared_directory)
 report['shared_directory_removed']=not shared_directory.exists()
 report.update(owned_children_reaped=all(p.poll() is not None for p in children),child_exit_codes=[p.returncode for p in children],execution_authorized=True)
 report['child_results']=[{'name':child_labels[p.pid],'pid':p.pid,'exit_code':p.returncode,'log':str(out/(child_labels[p.pid]+'.log'))} for p in children]
 (out/'execution.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))

if not report.get('run_completed_normally',False):
 raise RuntimeError('Simulator completion missing or failed; execution.json preserves diagnostics')
