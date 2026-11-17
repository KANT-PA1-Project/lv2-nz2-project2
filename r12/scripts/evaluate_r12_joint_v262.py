import json,time,uuid,subprocess,tempfile
from pathlib import Path
import numpy as np
from arm_practice.perception_transport import JsonPipeReader,stop_worker,require_response
from arm_practice.observation_readiness import check_archived_content
root=Path(__file__).resolve().parents[1];out=root.parents[1]/'outputs/r12/joint_observation_v262'
report={'scope':'Static synthetic render replay; no live capture freshness, robot motion or model training','responses':[],'completed':False}
w=None
try:
 with (out/'worker.log').open('w') as log,tempfile.TemporaryDirectory(dir='/dev/shm') as temp:
  w=subprocess.Popen(['/home/seongmin/ros2_ws/src/turtlebot-swerve-cuvslam/.venv-perception-live/bin/python',str(root/'scripts/r12_perception_worker.py')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True)
  reader=JsonPipeReader(w.stdout);report['ready']=reader.receive(lambda x:x.get('ready') is True,120)
  run='v262-offline-'+uuid.uuid4().hex
  for index,source in enumerate(sorted(out.glob('frame_*.npz'))):
   with np.load(source) as f:data={k:f[k] for k in f.files}
   req=dict(run_id=run,clock_domain='synthetic_packet_20hz',frame_index=index,stamp_ns=1000000000+index*50000000,capture_monotonic_ns=time.monotonic_ns(),camera='r07_camera1',detect=True,max_age_ns=60000000000,frame=str(Path(temp)/'frame.npz'))
   data.update({k:np.array(req[k]) for k in ['run_id','clock_domain','frame_index','stamp_ns','capture_monotonic_ns']});np.savez(req['frame'],**data)
   w.stdin.write(json.dumps(req)+'\n');w.stdin.flush();res=reader.receive(lambda x:'stamp_ns' in x,60);require_response(req,res,req['max_age_ns'])
   checks=[dict(index=i,**check_archived_content(res,i)) for i,d in enumerate(res['detections']) if 0<=d['class_id']<4]
   report['responses'].append(dict(source=source.name,response=res,checks=checks))
   (out/'inference.json').write_text(json.dumps(report,indent=2)+'\n')
   print(source.name,'tags',len(res['tags']),'pucks',len(checks),'content_pass',sum(c['content_ready'] for c in checks),flush=True)
  report['completed']=True
finally:
 if w is not None:stop_worker(w)
 (out/'inference.json').write_text(json.dumps(report,indent=2)+'\n')
