"""Offline recorded-frame transport check. Never publishes robot observations."""
import os,sys,time,json,subprocess
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'src'))
from arm_practice.perception_transport import JsonPipeReader,require_response,stop_worker
out=root.parents[1]/'outputs/r12/v401_tensorrt';source=root.parents[1]/'outputs/r12/v398_mission/frame.npz'
with np.load(source,allow_pickle=False) as z:data={k:z[k] for k in z.files}
env=os.environ.copy();env.update(R12_PERCEPTION_BACKEND='tensorrt_fp16',R12_TRT_MODEL_DIR=str(out))
log=(out/'worker_smoke.log').open('w');p=subprocess.Popen([sys.executable,str(root/'scripts/r12_perception_worker.py')],env=env,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True);reader=JsonPipeReader(p.stdout);rows=[]
try:
 ready=reader.receive(lambda v:v.get('ready'),120)
 for i,camera in enumerate(('r07_camera1','r07_camera2','r07_camera3','arm_camera')):
  capture=time.monotonic_ns();identity={'run_id':'offline_trt_worker_v401','frame_index':i,'clock_domain':'synthetic_packet_20hz','stamp_ns':1_000_000_000+i*50_000_000,'capture_monotonic_ns':capture};data.update({k:np.asarray(v) for k,v in identity.items()});np.savez(out/'worker_frame.npz',**data)
  req={**identity,'camera':camera,'frame':str(out/'worker_frame.npz'),'max_age_ns':500_000_000,'detect':True};p.stdin.write(json.dumps(req)+'\n');p.stdin.flush();r=reader.receive(lambda v:'stamp_ns' in v,30);now=time.monotonic_ns()
  verdict=require_response(req,r,500_000_000);rows.append({'camera':camera,'age_ms':(now-capture)/1e6,'inference_ms':r['inference_seconds']*1000,'detections':len(r['detections']),'provenance':verdict,'backend':r['backend']})
finally:
 stop_worker(p);log.close()
 (out/'worker_smoke.json').write_text(json.dumps({'scope':'offline recorded-frame replay; timestamp is benchmark start, not original sensor acquisition; no control/mission claim','ready':locals().get('ready'),'rows':rows,'all_four_passed':len(rows)==4},indent=2))
print(json.dumps(rows))
