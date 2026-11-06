"""Archived-frame worker replay: measures transport/inference, NEVER live freshness."""
import json, sys, time, subprocess, tempfile, uuid
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'src'))
from arm_practice.perception_transport import JsonPipeReader,stop_worker,require_response
out=root.parents[1]/'outputs/r12/v239_worker_replay'
out.mkdir(exist_ok=True)
source=root/'reports/local/observed_history_v159/frame.npz'
with np.load(source,allow_pickle=False) as f:data={k:f[k] for k in f.files}
report={'source':str(source),'scope':'archived pixels replay; timestamp is replay start, NOT original capture','live_freshness_verified':False,'motion_executed':False,'samples':[]}
def save():
 (out/'results.json').write_text(json.dumps(report,indent=2)+'\n')
worker=None
try:
 with (out/'worker.log').open('w') as log,tempfile.TemporaryDirectory(prefix='r12-v239-',dir='/dev/shm') as temp:
  start=time.monotonic()
  worker=subprocess.Popen(['/home/seongmin/ros2_ws/src/turtlebot-swerve-cuvslam/.venv-perception-live/bin/python',str(root/'scripts/r12_perception_worker.py')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True)
  reader=JsonPipeReader(worker.stdout)
  report['ready']=reader.receive(lambda x:x.get('ready') is True,120)
  report['startup_seconds']=time.monotonic()-start
  run='offline-replay-'+uuid.uuid4().hex
  for index,(camera,detect) in enumerate([('r07_camera1',False),('r07_camera1',True),('arm_camera',False),('arm_camera',True)]):
   capture=time.monotonic_ns();stamp=1000000000+index*50000000
   req=dict(run_id=run,clock_domain='synthetic_packet_20hz',frame_index=index,stamp_ns=stamp,capture_monotonic_ns=capture,camera=camera,detect=detect,max_age_ns=60_000_000_000,frame=str(Path(temp)/'frame.npz'))
   data.update({k:np.array(req[k]) for k in ['run_id','clock_domain','frame_index','stamp_ns','capture_monotonic_ns']})
   np.savez(req['frame'],**data)
   worker.stdin.write(json.dumps(req)+'\n');worker.stdin.flush()
   res=reader.receive(lambda x:'stamp_ns' in x,60)
   verdict=require_response(req,res,req['max_age_ns'])
   elapsed=(time.monotonic_ns()-capture)/1e9
   row=dict(camera=camera,detect=detect,replay_roundtrip_seconds=elapsed,inference_seconds=res['inference_seconds'],detections=len(res['detections']),tags=len(res['tags']),within_candidate_half_second=elapsed<=.5,identity_matched=verdict['matched'])
   report['samples'].append(row);save();print(json.dumps(row),flush=True)
  report['completed']=True
except Exception as error:
 report['error']=repr(error);report['completed']=False
 raise
finally:
 if worker is not None:stop_worker(worker)
 save()
