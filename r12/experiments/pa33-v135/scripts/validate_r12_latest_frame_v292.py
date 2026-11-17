"""Bounded synthetic20Hz replay producer; no live camera or controller."""
import json,time,threading,subprocess,tempfile,uuid
from pathlib import Path
import numpy as np,cv2
from arm_practice.perception_transport import JsonPipeReader,stop_worker,require_response
repo=Path(__file__).resolve().parents[1];out=repo.parents[1]/'outputs/r12';cv2.setNumThreads(1)
images=[cv2.cvtColor(cv2.imread(str(out/'mission_holdout_v290'/f'frame_{i:02d}.png')),cv2.COLOR_BGR2RGB) for i in range(18)]
report=dict(trials=[],completed=False,scope='Archived positive images released at actual host20Hz;capture time means replay arrival,not camera capture. One in-flight and at most one pending. Budgets250/50ms diagnostic only,not control acceptance requirements.',runtime_changed=False)
for budget in [250_000_000,50_000_000]:
 w=None;trial=dict(budget_ns=budget,arrivals=[],dropped=[],responses=[]);report['trials'].append(trial)
 try:
  with tempfile.TemporaryDirectory(dir='/dev/shm') as tmp,(out/f'v292_worker_{budget}.log').open('w') as log:
   w=subprocess.Popen([str(repo/'.venv-r12/bin/python'),str(repo/'scripts/r12_tag_worker_v291.py')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True);reader=JsonPipeReader(w.stdout);reader.receive(lambda x:x.get('ready'),30)
   lock=threading.Lock();state=dict(pending=None,done=False,latest=None);run='v292-'+uuid.uuid4().hex
   def producer():
    start=time.monotonic_ns()
    try:
     for i in range(60):
      delay=(start+i*50_000_000-time.monotonic_ns())/1e9
      if delay>0:time.sleep(delay)
      capture=time.monotonic_ns();q=dict(run_id=run,clock_domain='synthetic_packet_20hz',frame_index=i,stamp_ns=1000000000+i*50000000,capture_monotonic_ns=capture,camera='r07_camera2',max_age_ns=budget,frame=str(Path(tmp)/'frame.npz'))
      with lock:
       if state['pending'] is not None:trial['dropped'].append(state['pending']['frame_index'])
       state['pending']=q;state['latest']=q;trial['arrivals'].append(dict(index=i,capture_ns=capture,schedule_lateness_ms=(capture-start-i*50_000_000)/1e6))
    finally:
     with lock:state['done']=True
   thread=threading.Thread(target=producer);thread.start()
   try:
    while True:
     with lock:q=state['pending'];state['pending']=None;done=state['done']
     if q is None:
      if done:break
      time.sleep(.001);continue
     dispatch=time.monotonic_ns();data={k:np.array(q[k]) for k in ['run_id','clock_domain','frame_index','stamp_ns','capture_monotonic_ns']};np.savez(q['frame'],rgb=images[q['frame_index']%18],rgb_camera=np.array(q['camera']),**data)
     w.stdin.write(json.dumps(q)+'\n');w.stdin.flush();res=reader.receive(lambda x:True,30);received=time.monotonic_ns();accepted=False;reason=res.get('detail')
     if 'error' not in res:
      try:require_response(q,res,budget);accepted=True
      except ValueError as e:reason=str(e)
     with lock:latest=dict(state['latest'])
     current_frame_eligible=False
     if accepted:
      try:require_response(latest,res,budget);current_frame_eligible=True
      except ValueError:pass
     trial['responses'].append(dict(index=q['frame_index'],capture_ns=q['capture_monotonic_ns'],dispatch_wait_ms=(dispatch-q['capture_monotonic_ns'])/1e6,age_ms=(received-q['capture_monotonic_ns'])/1e6,accepted_for_original_frame=accepted,current_latest_index=latest['frame_index'],eligible_as_latest_frame=current_frame_eligible,reason=reason,worker_phase=res.get('phase'),ids=[t['id'] for t in res.get('tags',[])]))
   finally:thread.join(timeout=5)
   arrived={x['index'] for x in trial['arrivals']};dispatched={x['index'] for x in trial['responses']};dropped=set(trial['dropped']);assert arrived==dispatched|dropped and not dispatched&dropped
   assert all(not r['accepted_for_original_frame'] or r['age_ms']<=budget/1e6 for r in trial['responses'])
   assert all(not r['eligible_as_latest_frame'] or r['index']==r['current_latest_index'] for r in trial['responses'])
   trial['summary']=dict(arrivals=len(arrived),dispatched=len(dispatched),dropped=len(dropped),accepted_original=sum(r['accepted_for_original_frame'] for r in trial['responses']),eligible_latest=sum(r['eligible_as_latest_frame'] for r in trial['responses']),max_pending=1,max_in_flight=1,max_dispatch_wait_ms=max(r['dispatch_wait_ms'] for r in trial['responses']),max_response_age_ms=max(r['age_ms'] for r in trial['responses']),max_arrival_lateness_ms=max(r['schedule_lateness_ms'] for r in trial['arrivals']),timestamp_preserved=all(r['capture_ns']==trial['arrivals'][r['index']]['capture_ns'] for r in trial['responses']))
 finally:
  if w is not None:stop_worker(w)
  (out/'v292_latest_frame.json').write_text(json.dumps(report,indent=2)+'\n')
 print(budget,trial.get('summary'),flush=True)
report['completed']=True;(out/'v292_latest_frame.json').write_text(json.dumps(report,indent=2)+'\n')
