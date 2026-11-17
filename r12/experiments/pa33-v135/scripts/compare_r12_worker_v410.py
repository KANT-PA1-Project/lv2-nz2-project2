"""Offline recorded-image comparison; no ROS publisher or robot observation injection."""
import hashlib,json,os,statistics,subprocess,sys,tempfile,time
from pathlib import Path
import numpy as np

root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'src'))
from arm_practice.perception_transport import JsonPipeReader,require_response,stop_worker
out=root.parents[1]/'outputs/r12';destination=out/'v410_preparation'
original=out/'v409_code_snapshot/r12_perception_worker.py';candidate=root/'scripts/r12_perception_worker.py'
env=os.environ.copy();env.update(R12_PERCEPTION_BACKEND='tensorrt_fp16',R12_TRT_MODEL_DIR=str(out/'v401_tensorrt'),OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')
rows={}
cases=[(1800,camera,detect) for detect in [False,False,False,True] for camera in ['r07_camera1','r07_camera2','r07_camera3','arm_camera']]
with np.load(out/'v409_mission/observation_history/001800.npz',allow_pickle=False) as z:data={k:z[k] for k in z.files}
with tempfile.TemporaryDirectory(prefix='r12-worker-compare-',dir='/dev/shm') as temporary:
    frame=Path(temporary)/'frame.npz'
    for label,source in [('before',original),('after',candidate)]:
        log=(destination/(label+'_worker.log')).open('w')
        # Preserve the worker's original module location for dependency resolution.
        code="from pathlib import Path;import sys;p=Path(sys.argv[1]);exec(compile(p.read_text(),sys.argv[2],'exec'),{'__name__':'__main__','__file__':sys.argv[2]})"
        worker=subprocess.Popen([sys.executable,'-c',code,str(source),str(candidate)],env=env,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,text=True)
        reader=JsonPipeReader(worker.stdout);rows[label]=[]
        try:
            reader.receive(lambda v:v.get('ready'),60)
            for index,(_,camera,detect) in enumerate(cases):
                capture=time.monotonic_ns();identity=dict(run_id='offline_v410_'+label,frame_index=index,clock_domain='synthetic_packet_20hz',stamp_ns=1_000_000_000+index*50_000_000,capture_monotonic_ns=capture)
                data.update({k:np.asarray(v) for k,v in identity.items()});np.savez(frame,**data)
                request=dict(identity,camera=camera,frame=str(frame),max_age_ns=500_000_000,detect=detect)
                worker.stdin.write(json.dumps(request)+'\n');worker.stdin.flush();response=reader.receive(lambda v:'stamp_ns' in v,30);require_response(request,response,500_000_000)
                rows[label].append(dict(camera=camera,detect=detect,elapsed_ms=(time.monotonic_ns()-capture)/1e6,inference_ms=(response['inference_completed_monotonic_ns']-response['inference_started_monotonic_ns'])/1e6,payload={k:response[k] for k in ['barcodes','barcode_boxes','detections']},tags=response['tags'],detection_performed=response.get('detection_performed')))
        finally:
            stop_worker(worker);log.close()
matches=[a['payload']==b['payload'] and (not a['detect'] or a['tags']==b['tags']) for a,b in zip(rows['before'],rows['after'])]
report=dict(scope='Offline archived pixels; timestamps indicate benchmark start, not original acquisition. Only consumed payloads compared. No motion or mission accuracy claim.',cases=len(cases),payload_equal=matches,all_equal=all(matches),source_sha256={name:hashlib.sha256(p.read_bytes()).hexdigest() for name,p in [('before',original),('after',candidate)]},means={label:{'non_detection_inference_ms':statistics.mean(r['inference_ms'] for r in values if not r['detect']),'detection_inference_ms':statistics.mean(r['inference_ms'] for r in values if r['detect'])} for label,values in rows.items()},rows=rows)
(destination/'worker_comparison.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='rows'}))
if not report['all_equal']:raise SystemExit('Consumed payload mismatch')
