"""Isolated replay IPC validation; no runtime integration."""
import json,time,subprocess,tempfile,uuid
from pathlib import Path
import cv2,numpy as np
from arm_practice.perception_transport import JsonPipeReader,stop_worker,require_response,fresh_response
repo=Path(__file__).resolve().parents[1];out=repo.parents[1]/'outputs/r12';report=dict(rows=[],cases=[],completed=False,runtime_changed=False);w=None
try:
 with tempfile.TemporaryDirectory(dir='/dev/shm') as tmp,(out/'v291_worker.log').open('w') as log:
  w=subprocess.Popen([str(repo/'.venv-r12/bin/python'),str(repo/'scripts/r12_tag_worker_v291.py')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True);reader=JsonPipeReader(w.stdout);report['ready']=reader.receive(lambda x:x.get('ready'),30)
  def call(rgb,mutation=None):
   start=time.monotonic_ns();q=dict(run_id='v291-'+uuid.uuid4().hex,clock_domain='synthetic_packet_20hz',frame_index=0,stamp_ns=1000000000,capture_monotonic_ns=start,camera='r07_camera2',max_age_ns=10_000_000_000,frame=str(Path(tmp)/'frame.npz'))
   d={k:np.array(q[k]) for k in ['run_id','clock_domain','frame_index','stamp_ns','capture_monotonic_ns']};d.update(rgb=rgb,rgb_camera=np.array(q['camera']))
   if mutation:mutation(q,d)
   np.savez(q['frame'],**d);w.stdin.write(json.dumps(q)+'\n');w.stdin.flush();res=reader.receive(lambda x:True,30)
   if 'error' not in res:require_response(q,res,q['max_age_ns'])
   return q,res,(time.monotonic_ns()-start)/1e6
  for kind,folder in [('positive','mission_holdout_v290'),('negative','mission_holdout_negative_v290')]:
   for i in range(18):
    rgb=cv2.cvtColor(cv2.imread(str(out/folder/f'frame_{i:02d}.png')),cv2.COLOR_BGR2RGB);q,res,ms=call(rgb);assert 'error' not in res,res
    report['rows'].append(dict(kind=kind,index=i,tags=res['tags'],roundtrip_ms=ms,coordinate_system=res['corner_coordinate_system']))
  rgb=cv2.cvtColor(cv2.imread(str(out/'mission_holdout_v290/frame_00.png')),cv2.COLOR_BGR2RGB)
  cases=[('wrong_run',lambda q,d:q.update(run_id='wrong')),('wrong_camera',lambda q,d:q.update(camera='arm_camera')),('stale',lambda q,d:q.update(max_age_ns=1)),('future_capture',lambda q,d:(q.update(capture_monotonic_ns=time.monotonic_ns()+10**12),d.update(capture_monotonic_ns=np.array(q['capture_monotonic_ns'])))),('missing_camera',lambda q,d:d.pop('rgb_camera')),('float_pixels',lambda q,d:d.update(rgb=d['rgb'].astype(float))),('missing_pixels',lambda q,d:d.pop('rgb')),('bad_stamp',lambda q,d:(q.update(stamp_ns=2),d.update(stamp_ns=np.array(2)))),('wrong_resolution',lambda q,d:d.update(rgb=d['rgb'][:480,:640])),('expires_during_decode',lambda q,d:q.update(max_age_ns=50_000_000))]
  for name,mut in cases:
   q,res,ms=call(rgb,mut);report['cases'].append(dict(name=name,rejected='error' in res and 'tags' not in res,response=res,roundtrip_ms=ms))
  q,res,ms=call(rgb);report['recovery_ids']=[t['id'] for t in res['tags']];report['cached_expired_dropped']=fresh_response(q,res,1)=={};report['completed']=True
finally:
 if w is not None:stop_worker(w)
 (out/'v291_ipc_validation.json').write_text(json.dumps(report,indent=2)+'\n')
print('completed',report['completed'],'cases',sum(x['rejected'] for x in report['cases']),'/',len(report['cases']))
