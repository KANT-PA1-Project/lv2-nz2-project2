from pathlib import Path
import os,json,subprocess,signal,time,hashlib,sys,ast
from resource_tree import snapshot
root=Path(__file__).resolve().parent;out=Path(os.environ['FLEET_RUN_DIR']);manifest=root/'native_entrypoint_manifest.json'
def atomic(n,d):
 p=out/(n+'.tmp');p.write_text(json.dumps(d,indent=2));p.replace(out/n)
def verify():
 m=json.loads(manifest.read_text())['sha256'];assert all(hashlib.sha256((root/k).read_bytes()).hexdigest()==v for k,v in m.items());return len(m)
def gpu():
 a=subprocess.check_output(['nvidia-smi','--query-gpu=memory.used,memory.free,utilization.gpu','--format=csv,noheader,nounits'],text=True).splitlines()[0];u,f,c=map(int,a.split(','));return dict(used_MiB=u,free_MiB=f,compute_percent=c)
files=verify();g=gpu();assert g['free_MiB']>6500 and snapshot(-1)['mem_available_kib']>8*1024*1024
python=Path.home()/'pa-course/.venv-isaacsim/bin/python'
atomic('job_status.json',dict(updated_at=time.time(),stage='CPU actual entrypoint contracts',progress=dict(current=0,total=10,unit='native control epochs'),training=None,completed=False,performance_accepted=False,issue=None))
with(out/'CPU_contract_tests.log').open('w')as log:
 p=subprocess.run([str(python),'-c',"import sys;sys.path.append('/usr/lib/python3/dist-packages');import pytest,glob;raise SystemExit(pytest.main(['-q']+sorted(glob.glob('tests/test_native*.py'))))"],cwd=root,env=dict(os.environ,PYTHONPATH=str(root/'src')+':'+str(root/'tests')),stdout=log,stderr=subprocess.STDOUT,timeout=60)
 if p.returncode:raise RuntimeError('CPU actual entrypoint contracts failed')
atomic('preflight.json',dict(gpu=g,source_files=files,entrypoint='scripts/r12_sensor_sim.py',scope='Actual stationary main loop: postcapture2x120Hz samples after native-ready, original image epoch retained; no decoder/ROS/motion'))
sys.path.insert(0,str(root/'src'))
from arm_practice.native_rgbd.warmup_provenance import plan,digest
phase_plan=plan(2,8);phase_sha=digest(phase_plan)
atomic('precommitted_phase_plan.json',dict(plan=phase_plan,plan_sha256=phase_sha,committed_monotonic_ns=time.monotonic_ns(),scope='Fixed before child launch; preserve all cold failures'))
env=os.environ.copy()
env['R12_PHASE_PLAN_SHA256']=phase_sha
for k in ['PYTHONPATH','LD_LIBRARY_PATH','AMENT_PREFIX_PATH','COLCON_PREFIX_PATH','ROS_DISTRO']:env.pop(k,None)
env.update(CUDA_VISIBLE_DEVICES='0',OMNI_KIT_ACCEPT_EULA='YES',R12_GUI='0',R12_NATIVE_PRODUCER_STATIONARY='1',R12_NATIVE_ROS_PUBLICATION='1',R12_PRIORITY_FEEDBACK='1',R12_NATIVE_STARTUP_ORDER='1',R12_STATIONARY_CAPTURE='1',R12_USE_LATEST_PPO='1',R12_WHEEL_KD='.04',R12_ENCODER_FEEDBACK120='1',R12_PHYSICS_HZ='960',R12_PPO_RESIDUAL_SCALE='0',R12_PREPARED_CAMERA_RIG='1',R12_EXPLICIT_BASE_ROOT='1',R12_MISSION='0',R12_SENSOR_MOTION='0',R12_MISSION_PERCEPTION='0',R12_ISAAC_PIXEL_CENTER_V1='1',R12_IMU_LATEST_SAMPLING='1')
f=(out/'entrypoint.log').open('w');peerlog=(out/'sink.log').open('w');peer=subprocess.Popen(['/usr/bin/python3',str(root/'stationary_packet_sink.py')],cwd=root,env=env,stdout=peerlog,stderr=subprocess.STDOUT,start_new_session=True)
for _ in range(100):
 if (out/'sink_ready.json').exists():break
 if peer.poll()is not None:raise RuntimeError('Stationary sink startup failed')
 time.sleep(.01)
else:raise RuntimeError('Stationary sink startup timeout')
p=subprocess.Popen([str(python),str(root/'scripts/r12_sensor_sim.py'),'--frames','10','--port','20033'],cwd=root,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
atomic('owned_processes.json',dict(isaac_pid=p.pid,sink_pid=peer.pid,started_at=time.time()));start=time.time();resources=[];tree=[];failure=None;last_status=0
try:
 while p.poll()is None:
  try:p.wait(timeout=.5)
  except subprocess.TimeoutExpired:pass
  g=gpu();r=snapshot(p.pid);g['elapsed_s']=time.time()-start;resources.append(g);tree.append(r)
  if g['free_MiB']<1024:raise RuntimeError('VRAM reserve below1024MiB')
  if r['mem_available_kib']<2*1024*1024:raise RuntimeError('RAM reserve below2GiB')
  if time.time()-start>240:raise RuntimeError('Bounded240s wall limit')
  if time.time()-last_status>5:
   count=len(json.loads((out/'native_producer_frames.json').read_text()))if(out/'native_producer_frames.json').exists()else 0
   atomic('job_status.json',dict(updated_at=time.time(),stage='Actual r12_sensor_sim stationary native producer',progress=dict(current=count,total=10,unit='native control epochs'),training=None,completed=False,performance_accepted=False,issue='Producer integration only, perception/mission not started'));last_status=time.time()
except Exception as e:failure=str(e)
finally:
 for child in [p,peer]:
  if child.poll()is None:
   try:child.wait(timeout=2 if child is peer else .01)
   except subprocess.TimeoutExpired:
    os.killpg(child.pid,signal.SIGTERM)
    try:child.wait(timeout=8)
    except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
 f.close();peerlog.close();unchanged=verify()==files
 c=json.loads((out/'native_producer_completion.json').read_text())if(out/'native_producer_completion.json').exists()else {}
 ros_results=json.loads((out/'ROS_publication_results.json').read_text())if(out/'ROS_publication_results.json').exists()else[]
 child_exits={name:json.loads((out/name).read_text())if(out/name).exists()else None for name in ['priority_feedback_exit.json','ROS_publisher_exit.json','decoder_exit.json']}
 ok=all(value is not None and value['exit_code']==0 for value in child_exits.values())and bool(ros_results)and all(x['all_received']for x in ros_results)and p.returncode==0 and peer.returncode==0 and c.get('measurement_complete',False)and failure is None and unchanged
 atomic('resource_samples.json',resources);atomic('CPU_resource_samples.json',tree)
 result=dict(measurement_complete=bool(c.get('measurement_complete',False) and failure is None),capture_pass=bool(c.get('capture_pass',False) and failure is None),mission_success=False,runtime_exit=p.returncode,sink_exit=peer.returncode,completed=ok,clean_process_exit=p.returncode==0,clean_exit=ok,supervisor_failure=failure,source_unchanged=unchanged,native_completion=c,performance_accepted=False,decoder_started=(out/'decoder_ready.json').exists(),ROS_started=(out/'ROS_trusted_session.json').exists(),ROS_capture_count=len(ros_results),child_exits=child_exits)
 atomic('shutdown_exit.json',result);atomic('job_status.json',dict(updated_at=time.time(),stage='Actual native producer integration complete'if ok else'Actual native producer integration failed',progress=dict(current=c.get('measured_control_epochs',0),total=10,unit='native control epochs'),training=None,completed=ok,performance_accepted=False,issue=failure or'No perception/mission admission; inspect native readiness/current velocity freshness'))
 print(json.dumps(result));sys.exit(0 if ok else 1)
