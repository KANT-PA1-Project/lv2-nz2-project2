import json,time,sys,subprocess,tempfile,uuid
from pathlib import Path
import cv2,numpy as np
from arm_practice.perception_transport import JsonPipeReader,stop_worker,require_response
repo=Path(__file__).resolve().parents[1];out=repo.parents[1]/'outputs/r12';src=out/'negative_clutter_v269'
meta=json.loads((src/'manifest.json').read_text())['rows'];meta=[dict(r,tag_id=-1) for r in meta];images=[cv2.cvtColor(cv2.imread(str(src/f'frame_{r["index"]:02d}.png')),cv2.COLOR_BGR2RGB) for r in meta]
envs=[('4.11',Path('/home/seongmin/ros2_ws/src/turtlebot-swerve-cuvslam/.venv-perception-live/bin/python')),('4.14',repo/'.venv-r12/bin/python')]
reports=[]
for label,python in envs:
 w=None;report=dict(environment=label,rows=[],completed=False);reports.append(report)
 try:
  with tempfile.TemporaryDirectory(dir='/dev/shm') as tmp,(out/f'v269_worker_{label}.log').open('w') as log:
   start=time.monotonic_ns();w=subprocess.Popen([str(python),str(repo/'scripts/r12_tag_worker_v267.py')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True);reader=JsonPipeReader(w.stdout);report['ready']=reader.receive(lambda x:x.get('ready') is True,30);report['startup_ms']=(time.monotonic_ns()-start)/1e6
   run='offline-v269-'+uuid.uuid4().hex
   for i,(m,rgb) in enumerate(zip(meta,images)):
    start=time.monotonic_ns();req=dict(run_id=run,clock_domain='synthetic_packet_20hz',frame_index=i,stamp_ns=1000000000+i*50000000,capture_monotonic_ns=start,camera='r07_camera1',max_age_ns=60_000_000_000,frame=str(Path(tmp)/'frame.npz'))
    data={k:np.array(req[k]) for k in ['run_id','clock_domain','frame_index','stamp_ns','capture_monotonic_ns']};np.savez(req['frame'],rgb=rgb,rgb_camera=np.array(req['camera']),**data)
    w.stdin.write(json.dumps(req)+'\n');w.stdin.flush();res=reader.receive(lambda x:True,30);require_response(req,res,req['max_age_ns'])
    report['rows'].append(dict(index=m['index'],expected=m['tag_id'],accepted=[x['id'] for x in res['tags']],roundtrip_ms=(time.monotonic_ns()-start)/1e6))
   # Wrong identity must be rejected; restore neither metadata nor synthetic truth.
   bad=dict(req,run_id='wrong');w.stdin.write(json.dumps(bad)+'\n');w.stdin.flush();err=reader.receive(lambda x:True,30);report['identity_mismatch_rejected']='error' in err and 'tags' not in err
   report['completed']=True
 finally:
  if w is not None:stop_worker(w)
  (out/'v269_transport_benchmark.json').write_text(json.dumps(dict(reports=reports,scope='Archived replay timestamp measures serialization/file read/IPC/decode;NOT live capture age',runtime_changed=False),indent=2)+'\n')
 print(label,'completed',len(report['rows']),flush=True)
