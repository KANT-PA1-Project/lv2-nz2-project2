"""Bounded owned-process stationary GUI capture, isolated domain and outputs."""
import ast,os,json,time,subprocess,signal,socket,tempfile,threading
from pathlib import Path
import rclpy
repo=Path(__file__).resolve().parents[1];out=repo.parents[1]/'outputs/r12/v393_stationary';out.mkdir(exist_ok=False)
from r12_resource_admission import check,snapshot
from r12_owned_memory import sample_owned
admission=check(out/'resource_admission.json',minimum_available_gib=4)
if not admission['admitted']:raise SystemExit('Resource admission deferred: '+str(admission['reasons']))
env=os.environ.copy();env.update(ROS_DOMAIN_ID='232',R12_SENSOR_PORT='19980',R12_SENSOR_RUN=str(out),R12_CLOUD_PUBLICATION_AUDIT='1',R12_STATIONARY_CAPTURE='1',R12_MISSION='0',R12_SENSOR_MOTION='0',R12_MISSION_PERCEPTION='0',R12_GUI='1',R12_BUFFER_PUBLICATION_DIAGNOSTICS='1',R12_PUBLICATION_DIAGNOSTICS='1',R12_CALLBACK_AUDIT='1',R12_FAST_SLICE_COUNT='1',R12_SLICE_CALLBACK_TIMING='1',R12_PUBLICATION_OPERATION_TIMING='1',R12_ACQUISITION_TIMING='1',R12_FRAME_MEMBER_CACHE='1',R12_SINGLE_RGB_STACK_COPY='1',R12_SHARED_FRAME_DIAGNOSTIC='1',R12_PREPARED_CAMERA_RIG='1',R12_ANNOTATOR_NO_COPY='1',R12_OWNERSHIP_AUDIT='0',R12_SMALL_VIEWPORT_DIAGNOSTIC='0',R12_HYDRA_PROFILE='1',R12_HYDRA_PROFILE_ENABLE='1',R12_HYDRA_PROFILE_AFTER_WARMUP='1',R12_RENDER_SETTINGS_AUDIT='1',R12_ANNOTATOR_DEVICE='cpu',R12_GPU_ACQUISITION='0',R12_GPU_CAMERA_PROBE='0',R12_USE_LATEST_PPO='0',R12_USE_PPO='0',R12_RECORD_VIDEO='0',R12_PHYSICS_HZ='960',R12_SIM_SENSOR_WAIT_S='1.0')
os.environ['ROS_DOMAIN_ID']='232';rclpy.init();n=rclpy.create_node('v393_preflight',enable_rosout=False)
try:
 for _ in range(10):rclpy.spin_once(n,timeout_sec=.1)
 assert all(name=='v393_preflight' for name,ns in n.get_node_names_and_namespaces()),'domain occupied'
finally:n.destroy_node();rclpy.shutdown()
with socket.socket() as sock:sock.bind(('127.0.0.1',19980))
shared_directory=tempfile.TemporaryDirectory(prefix='r12-frame-',dir='/dev/shm')
env['R12_SHARED_FRAME_DIRECTORY']=shared_directory.name
children=[];logs=[];report={'scope':'stationary IsaacSim GUI sensor capture; non-debugger RViz URDF-status readiness; no hardware','frames_requested':20}
def spawn(cmd,name,e=env):
 f=(out/(name+'.log')).open('x');logs.append(f);p=subprocess.Popen(cmd,cwd=repo,env=e,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);children.append(p);return p
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
 # Only state publication, EKF and nvblox; do not start planners for stationary capture.
 mapping_source=(repo/'configs/sensors/mapping.launch.py').read_text()
 mapping_source=mapping_source.replace("root=Path(__file__).resolve().parent",'root=Path('+repr(str(repo/'configs/sensors'))+')')
 tree=ast.parse(mapping_source)
 for item in ast.walk(tree):
  if isinstance(item,ast.List):
   item.elts=[e for e in item.elts if not (isinstance(e,ast.Call) and any(k.arg=='package' and isinstance(k.value,ast.Constant) and str(k.value.value).startswith('nav2_') for k in e.keywords))]
 mapping_source=ast.unparse(ast.fix_missing_locations(tree));compile(mapping_source,'stationary_mapping','exec')
 mapping=out/'stationary_mapping.launch.py';mapping.write_text(mapping_source+'\n')
 mapping_process=spawn(['ros2','launch',str(mapping)],'mapping')
 bridge=spawn([str(repo/'.venv-r12/bin/python'),'scripts/r12_sensor_ros.py'],'bridge')
 deadline=time.monotonic()+30
 while time.monotonic()<deadline:
  if bridge.poll() is not None:raise RuntimeError('bridge exited')
  if 'SENSOR_ROS_READY' in (out/'bridge.log').read_text():break
  time.sleep(.25)
 else:raise RuntimeError('bridge readiness timeout')
 rclpy.init();check=rclpy.create_node('v393_readiness',enable_rosout=False)
 try:
  deadline=time.monotonic()+30
  while time.monotonic()<deadline:
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
 sim=spawn(['bash','scripts/r12_runtime.sh','python','scripts/r12_sensor_sim.py','--frames','20','--port','19980'],'sim',simenv)
 report['pids']=[p.pid for p in children];(out/'running.json').write_text(json.dumps(report,indent=2))
 deadline=time.monotonic()+240;pressure_strikes=0
 while sim.poll() is None:
  resource=snapshot()
  pressured=resource['mem_available_bytes']<2*1024**3 or resource['pressure']['memory']['full']['avg10']>10
  pressure_strikes=pressure_strikes+1 if pressured else 0
  if pressure_strikes>=3:
   report['resource_abort']=resource
   raise RuntimeError('Owned stationary trial stopped after three resource-pressure samples')
  if time.monotonic()>deadline:raise TimeoutError('Bounded simulator timeout240s')
  time.sleep(2)
 bridge.wait(timeout=30)
 report['sim_exit']=sim.returncode
 report['completion_marker']='R12_SENSOR_SIM_COMPLETE' in (out/'sim.log').read_text()
 report['completion_file']=(out/'completion.json').exists()
finally:
 for p in reversed(children):
  if p.poll() is None:
   os.killpg(p.pid,signal.SIGTERM)
   try:p.wait(timeout=5)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=5)
 for f in logs:f.close()
 telemetry_stop.set();telemetry_thread.join(timeout=5)
 shm_after=shm_inventory()
 (out/'shm_after.json').write_text(json.dumps(shm_after))
 report['new_shm_names_observed']=[name for name,value in shm_after.items() if name not in shm_before or shm_before[name]['inode']!=value['inode']]
 report['shm_attribution_note']='Temporal difference only, not ownership proof; no DDS files deleted.'
 # All owned child processes are reaped before the supervisor removes its slot.
 shared_directory.cleanup()
 report['shared_directory_removed']=not Path(shared_directory.name).exists()
 report.update(owned_children_reaped=all(p.poll() is not None for p in children),child_exit_codes=[p.returncode for p in children],execution_authorized=False)
 (out/'execution.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
