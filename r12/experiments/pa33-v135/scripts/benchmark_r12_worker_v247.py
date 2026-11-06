"""Archived-frame worker replay: measures transport/inference, NEVER live freshness."""
import json, sys, time, subprocess, tempfile, uuid
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'src'))
from arm_practice.perception_transport import JsonPipeReader,stop_worker,require_response
out=root.parents[1]/'outputs/r12/v247_worker_replay'
out.mkdir(exist_ok=True)
sources=[root/'reports/local'/name/'frame.npz' for name in ['observed_history_v159','mission_low_speed_drive_v21','diagnostic_frame_provenance_v150','terminal_observation_v166']]
frames=[]
for source in sources:
 with np.load(source,allow_pickle=False) as f:
  data={k:f[k] for k in f.files}
 for camera in ['r07_camera2','r07_camera3','arm_camera']:
  assert camera+'_left_optical' in data and camera+'_depth' in data, (source,camera)
 frames.append(data)
source=sources[0]
report={'sources':[str(p) for p in sources],'scope':'archived pixels replay; timestamp is replay start, NOT original capture','live_freshness_verified':False,'motion_executed':False,'samples':[]}
def save():
 (out/'results.json').write_text(json.dumps(report,indent=2)+'\n')
worker=None
try:
 with (out/'worker.log').open('w') as log,tempfile.TemporaryDirectory(prefix='r12-v247-',dir='/dev/shm') as temp:
  start=time.monotonic()
  worker=subprocess.Popen(['/home/seongmin/ros2_ws/src/turtlebot-swerve-cuvslam/.venv-perception-live/bin/python',str(root/'scripts/r12_perception_worker_v247_diagnostic.py')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True)
  reader=JsonPipeReader(worker.stdout)
  report['ready']=reader.receive(lambda x:x.get('ready') is True,120)
  report['startup_seconds']=time.monotonic()-start
  run='offline-replay-'+uuid.uuid4().hex
  jobs=[(scene,camera,detect,repeat) for repeat in range(3) for scene in range(4) for camera in ['r07_camera1','r07_camera2','r07_camera3','arm_camera'] for detect in [True]]
  for index,(scene,camera,detect,repeat) in enumerate(jobs):
   data=frames[scene].copy()
   capture=time.monotonic_ns();stamp=1000000000+index*50000000
   req=dict(run_id=run,clock_domain='synthetic_packet_20hz',frame_index=index,stamp_ns=stamp,capture_monotonic_ns=capture,camera=camera,detect=detect,max_age_ns=60_000_000_000,frame=str(Path(temp)/'frame.npz'))
   data.update({k:np.array(req[k]) for k in ['run_id','clock_domain','frame_index','stamp_ns','capture_monotonic_ns']})
   np.savez(req['frame'],**data)
   worker.stdin.write(json.dumps(req)+'\n');worker.stdin.flush()
   res=reader.receive(lambda x:'stamp_ns' in x,60)
   verdict=require_response(req,res,req['max_age_ns'])
   elapsed=(time.monotonic_ns()-capture)/1e9
   row=dict(scene=str(sources[scene]),repeat=repeat,camera=camera,detect=detect,pose_outputs=sum('pose' in d for d in res['detections']),valid_translation_outputs=sum(bool(d.get('pose',{}).get('translation_valid')) for d in res['detections']),depth_consistent_outputs=sum(bool(d.get('depth_consistent')) for d in res['detections']),replay_roundtrip_seconds=elapsed,inference_seconds=res['inference_seconds'],detections=len(res['detections']),tags=len(res['tags']),within_candidate_half_second=elapsed<=.5,identity_matched=verdict['matched'])
   row['detection_records']=res['detections']
   report['samples'].append(row);save();print(json.dumps(row),flush=True)
  values=[x['replay_roundtrip_seconds'] for x in report['samples']]
  report['timing_summary']={'n':len(values),'median_s':float(np.median(values)),'p95_s':float(np.quantile(values,.95)),'max_s':max(values)}
  report['completed']=True
except Exception as error:
 report['error']=repr(error);report['completed']=False
 raise
finally:
 if worker is not None:stop_worker(worker)
 save()
