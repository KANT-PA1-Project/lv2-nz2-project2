"""Seeded non-tag stress images plus protocol failures; not mission negatives."""
import json,time,subprocess,tempfile,uuid,hashlib
from pathlib import Path
import numpy as np,cv2
from arm_practice.perception_transport import JsonPipeReader,stop_worker,require_response
repo=Path(__file__).resolve().parents[1];out=repo.parents[1]/'outputs/r12/v268_protocol';out.mkdir(exist_ok=True)
seed=2682026;rng=np.random.default_rng(seed);images=[]
for i in range(48):
 im=np.full((480,640,3),int(rng.integers(100,256)),np.uint8)
 if i%3==0:
  for _ in range(35):
   x,y=rng.integers([0,0],[620,460]);w,h=rng.integers(3,70,2);cv2.rectangle(im,(int(x),int(y)),(int(x+w),int(y+h)),(0,0,0),int(rng.integers(1,5)))
 elif i%3==1:
  im=rng.integers(0,256,(480,640,3),dtype=np.uint8)
 else:
  for y in range(20,460,25):
   for x in range(20,620,25):
    if rng.random()<.5:cv2.rectangle(im,(x,y),(x+12,y+12),(0,0,0),-1)
 cv2.imwrite(str(out/f'negative_{i:02d}.png'),im);images.append(cv2.cvtColor(im,cv2.COLOR_BGR2RGB))
report=dict(seed=seed,scope='48 new procedural rectangles/noise/grid stress images,not rendered mission scenes or certified non-tag patterns',cases=[],negatives=[],completed=False)
w=None
try:
 with tempfile.TemporaryDirectory(dir='/dev/shm') as tmp,(out/'worker.log').open('w') as log:
  w=subprocess.Popen([str(repo/'.venv-r12/bin/python'),str(repo/'scripts/r12_tag_worker_v267.py')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True);reader=JsonPipeReader(w.stdout);report['ready']=reader.receive(lambda x:x.get('ready'),30)
  def call(name,rgb,mutation=None):
   req=dict(run_id='v268-'+uuid.uuid4().hex,clock_domain='synthetic_packet_20hz',frame_index=0,stamp_ns=1000000000,capture_monotonic_ns=time.monotonic_ns(),camera='r07_camera1',max_age_ns=10_000_000_000,frame=str(Path(tmp)/'frame.npz'))
   data={k:np.array(req[k]) for k in ['run_id','clock_domain','frame_index','stamp_ns','capture_monotonic_ns']};data.update(rgb=rgb,rgb_camera=np.array(req['camera']))
   if mutation:mutation(req,data)
   np.savez(req['frame'],**data);w.stdin.write(json.dumps(req)+'\n');w.stdin.flush();res=reader.receive(lambda x:True,30)
   if 'error' not in res:require_response(req,res,req['max_age_ns'])
   return res
  cases=[('wrong_run',lambda q,d:q.update(run_id='wrong')),('wrong_camera',lambda q,d:q.update(camera='arm_camera')),('stale',lambda q,d:q.update(max_age_ns=1)),('future_capture',lambda q,d:(q.update(capture_monotonic_ns=time.monotonic_ns()+10**12),d.update(capture_monotonic_ns=np.array(q['capture_monotonic_ns'])))),('missing_camera',lambda q,d:d.pop('rgb_camera')),('float_pixels',lambda q,d:d.update(rgb=d['rgb'].astype(float))),('missing_pixels',lambda q,d:d.pop('rgb')),('bad_stamp',lambda q,d:(q.update(stamp_ns=2),d.update(stamp_ns=np.array(2))))]
  for name,mutation in cases:
   res=call(name,images[0],mutation);report['cases'].append(dict(name=name,rejected='error' in res and 'tags' not in res,response=res))
  for i,im in enumerate(images):
   res=call(str(i),im);report['negatives'].append(dict(index=i,sha256=hashlib.sha256(im.tobytes()).hexdigest(),tags=res.get('tags'),error=res.get('error')))
  # Ensure error recovery did not disable decoding.
  with np.load(repo.parents[1]/'outputs/r12/joint_observation_v262/frame_01.npz') as f:rgb=f['rgb'][0]
  res=call('positive_recovery',rgb);report['positive_recovery_ids']=[x['id'] for x in res.get('tags',[])];report['completed']=True
finally:
 if w is not None:stop_worker(w)
 (out/'results.json').write_text(json.dumps(report,indent=2)+'\n')
print('cases',sum(c['rejected'] for c in report['cases']),'/',len(report['cases']),'negative detections',sum(bool(n['tags']) for n in report['negatives']),'positive recovery',report.get('positive_recovery_ids'))
