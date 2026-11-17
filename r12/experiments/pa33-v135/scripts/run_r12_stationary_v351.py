"""Bounded owned-process stationary GUI capture, isolated domain and outputs."""
import ast,os,json,time,subprocess,signal,socket,tempfile
from pathlib import Path
import rclpy
repo=Path(__file__).resolve().parents[1];out=repo.parents[1]/'outputs/r12/v351_stationary';out.mkdir(exist_ok=False)
env=os.environ.copy();env.update(ROS_DOMAIN_ID='233',R12_SENSOR_PORT='19951',R12_SENSOR_RUN=str(out),R12_STATIONARY_CAPTURE='1',R12_MISSION='0',R12_SENSOR_MOTION='0',R12_MISSION_PERCEPTION='0',R12_GUI='1',R12_PUBLICATION_DIAGNOSTICS='1',R12_CALLBACK_AUDIT='1',R12_ACQUISITION_TIMING='1',R12_FRAME_MEMBER_CACHE='1',R12_SINGLE_RGB_STACK_COPY='1',R12_SHARED_FRAME_DIAGNOSTIC='1',R12_PREPARED_CAMERA_RIG='1',R12_ANNOTATOR_NO_COPY='1',R12_OWNERSHIP_AUDIT='0',R12_GPU_CAMERA_PROBE='0',R12_GPU_ACQUISITION='1',R12_GPU_ACQUISITION_VALIDATE='0',R12_ANNOTATOR_DEVICE='cuda',R12_USE_LATEST_PPO='0',R12_USE_PPO='0',R12_RECORD_VIDEO='0',R12_PHYSICS_HZ='960',R12_SIM_SENSOR_WAIT_S='1.0')
os.environ['ROS_DOMAIN_ID']='233';rclpy.init();n=rclpy.create_node('v351_preflight',enable_rosout=False)
try:
 for _ in range(10):rclpy.spin_once(n,timeout_sec=.1)
 assert all(name=='v351_preflight' for name,ns in n.get_node_names_and_namespaces()),'domain occupied'
finally:n.destroy_node();rclpy.shutdown()
with socket.socket() as sock:sock.bind(('127.0.0.1',19951))
shared_directory=tempfile.TemporaryDirectory(prefix='r12-frame-',dir='/dev/shm')
env['R12_SHARED_FRAME_DIRECTORY']=shared_directory.name
children=[];logs=[];report={'scope':'stationary GUI sensor capture,not mission or grasp','frames_requested':20}
def spawn(cmd,name,e=env):
 f=(out/(name+'.log')).open('x');logs.append(f);p=subprocess.Popen(cmd,cwd=repo,env=e,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);children.append(p);return p
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
 rclpy.init();check=rclpy.create_node('v351_readiness',enable_rosout=False)
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
 spawn(['rviz2','-d','configs/sensors/r12_mission.rviz','--ros-args','-p','use_sim_time:=true'],'rviz')
 simenv=env.copy()
 for k in ['ROS_DISTRO','AMENT_PREFIX_PATH','PYTHONPATH','LD_LIBRARY_PATH']:simenv.pop(k,None)
 simenv['OMNI_KIT_ACCEPT_EULA']='YES'
 sim=spawn(['bash','scripts/r12_runtime.sh','python','scripts/r12_sensor_sim.py','--frames','20','--port','19951'],'sim',simenv)
 report['pids']=[p.pid for p in children];(out/'running.json').write_text(json.dumps(report,indent=2))
 report['sim_exit']=sim.wait(timeout=240)
 report['completion_marker']='R12_SENSOR_SIM_COMPLETE' in (out/'sim.log').read_text()
 report['completion_file']=(out/'completion.json').exists()
finally:
 for p in reversed(children):
  if p.poll() is None:
   os.killpg(p.pid,signal.SIGTERM)
   try:p.wait(timeout=5)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=5)
 for f in logs:f.close()
 # All owned child processes are reaped before the supervisor removes its slot.
 shared_directory.cleanup()
 report['shared_directory_removed']=not Path(shared_directory.name).exists()
 report.update(owned_children_reaped=all(p.poll() is not None for p in children),child_exit_codes=[p.returncode for p in children],execution_authorized=False)
 (out/'execution.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
