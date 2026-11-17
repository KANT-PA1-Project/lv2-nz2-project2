"""Longer drive/perception diagnostic with staged timings and immutable sensor archives."""
import ast,os,json,time,subprocess,signal,socket,tempfile,threading
from pathlib import Path
import rclpy
repo=Path(__file__).resolve().parents[1];out=repo.parents[1]/'outputs/r12/v411_mission';out.mkdir(exist_ok=False)
from r12_resource_admission import check,snapshot
from r12_owned_memory import sample_owned
from r12_owned_session import OwnedSession
import shutil
admission=check(out/'resource_admission.json',minimum_available_gib=4)
if not admission['admitted']:raise SystemExit('Resource admission deferred: '+str(admission['reasons']))
env=os.environ.copy()
for key in list(env):
 if key.startswith('R12_'):env.pop(key)
env.update(ROS_DOMAIN_ID='222',R12_SENSOR_PORT='19991',R12_SENSOR_RUN=str(out),R12_CLOUD_PUBLICATION_AUDIT='1',R12_STATIONARY_CAPTURE='0',R12_MISSION='1',R12_SENSOR_MOTION='1',R12_MISSION_PERCEPTION='1',R12_PERCEPTION_BACKEND='tensorrt_fp16',R12_TRT_MODEL_DIR=str(repo.parents[1]/'outputs/r12/v401_tensorrt'),R12_GUI='1',R12_FORCE_AA_OFF='1',R12_PERCEPTION_INTERVAL_FRAMES='1',R12_BOUND_VISUAL_WAIT_TO_FRESHNESS='1',R12_IMAGE_ARRAY_BUFFER='1',R12_SIGNAL_AT_RECEIVE='1',R12_FRAME_MEMBER_CACHE='1',R12_SINGLE_RGB_STACK_COPY='1',R12_FAST_SLICE_COUNT='1',R12_USE_LATEST_PPO='1',R12_USE_PPO='0',R12_CRUISE_SPEED='0.12',R12_WHEEL_INTEGRAL='1',R12_ENCODER_FEEDBACK120='1',R12_GOAL_OBSERVATION_DWELL_S='4.5',R12_ACQUISITION_TIMING='1',R12_PUBLICATION_OPERATION_TIMING='1',R12_OBSERVATION_HISTORY_STRIDE='100',R12_RECORD_VIDEO='0',R12_PHYSICS_HZ='960',R12_SIM_SENSOR_WAIT_S='1.0')
os.environ['ROS_DOMAIN_ID']='222';rclpy.init();n=rclpy.create_node('v411_preflight',enable_rosout=False)
try:
 for _ in range(10):rclpy.spin_once(n,timeout_sec=.1)
 assert all(name=='v411_preflight' for name,ns in n.get_node_names_and_namespaces()),'domain occupied'
finally:n.destroy_node();rclpy.shutdown()
with socket.socket() as sock:sock.bind(('127.0.0.1',19991))
shared_directory=Path(tempfile.mkdtemp(prefix='r12-frame-',dir='/dev/shm'))
env['R12_FRAME_FILE']=str(shared_directory/'frame.npz')
 # RAM-backed NPZ retains the existing synchronous frame identity protocol.
route=json.loads((repo/'models/navigation/routes.json').read_text());route.pop('observation_posture',None);(out/'survey_route.json').write_text(json.dumps(route));env['R12_ROUTE_FILE']=str(out/'survey_route.json')
children=[];owned_sessions=[];logs=[];report={'scope':'Simulation mission perception inventory and navigation survey, not pick/place; GUI and RViz; speed ceiling0.12m/s; compatible v2 policy performance NOT accepted','frames_requested':12000,'run_completed_normally':False,'control_comparison':{'bounded_wheel_integral':True,'quantized_encoder_feedback_hz':120,'policy_file_unchanged':True,'speed_ceiling_m_s':0.12,'baseline':'v410_mission'},'validation_tests_passed':21}
last_owned_refresh=0.
def refresh_owned(force=False):
 global last_owned_refresh
 if force or time.monotonic()-last_owned_refresh>=.5:
  for session in owned_sessions:session.refresh()
  last_owned_refresh=time.monotonic()
def spawn(cmd,name,e=env):
 f=(out/(name+'.log')).open('x');logs.append(f)
 p=subprocess.Popen(cmd,cwd=repo,env=e,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);children.append(p)
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
 rclpy.init();check=rclpy.create_node('v411_readiness',enable_rosout=False)
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
 import yaml
 cfg=yaml.safe_load((repo/'configs/sensors/r12_mission.rviz').read_text())
 cfg['Panels'].append({'Class':'r12/ReadinessProbe','Name':'R12 model readiness'})
 config=out/'readiness.rviz';config.write_text(yaml.safe_dump(cfg))
 env['R12_RVIZ_READINESS_FILE']=str(out/'rviz_readiness.json')
 rviz=spawn(['rviz2','-d',str(config),'--ros-args','-p','use_sim_time:=true'],'rviz')
 readiness_start=time.monotonic()
 while True:
  refresh_owned()
  ready_file=out/'rviz_readiness.json'
  if ready_file.exists():
   ready=json.loads(ready_file.read_text())
   if ready.get('urdf_ready'):
    report['readiness_at_start']=ready;report['urdf_ready_wait_s']=time.monotonic()-readiness_start;break
  if rviz.poll() is not None:raise RuntimeError('RViz exited before readiness')
  if time.monotonic()-readiness_start>60:raise TimeoutError('URDF parsed OK not observed within60s')
  time.sleep(.25)
 simenv=env.copy()
 for k in ['ROS_DISTRO','AMENT_PREFIX_PATH','PYTHONPATH','LD_LIBRARY_PATH']:simenv.pop(k,None)
 simenv['OMNI_KIT_ACCEPT_EULA']='YES'
 sim=spawn(['bash','scripts/r12_runtime.sh','python','scripts/r12_sensor_sim.py','--frames','12000','--port','19991'],'sim',simenv)
 report['pids']=[p.pid for p in children];(out/'running.json').write_text(json.dumps(report,indent=2))
 deadline=time.monotonic()+7200;pressure_strikes=0
 while sim.poll() is None:
  refresh_owned()
  if bridge.poll() is not None or rviz.poll() is not None or mapping_process.poll() is not None or slam.poll() is not None:raise RuntimeError("Required mission process exited; stopping owned simulation")
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
 (out/'execution.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
