"""Cross-process diagnostic of shared slot ownership; no simulator/control."""
import copy, hashlib, json, os, subprocess, sys, time
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from arm_practice.shared_frame_prototype import SharedFrameSlot, snapshot
from arm_practice.capture_bundle_identity import inspect_capture_bundle

def hashes(arrays):
    return {k: hashlib.sha256(v.tobytes()).hexdigest() for k,v in arrays.items()}

if '--reader' in sys.argv:
    session=None;last_sequence=0
    for line in sys.stdin:
        envelope=json.loads(line);d=envelope['descriptor'];packet=envelope['packet']
        if session is None:session=d['session']
        assert d['session']==session and d['sequence']==last_sequence+1
        a,ack=snapshot(d);last_sequence=d['sequence']
        print(json.dumps({'ack':ack,'hashes':hashes(a),'identity':inspect_capture_bundle(a,packet)['matched']}),flush=True)
    sys.exit(0)

out=Path(__file__).resolve().parents[3]/'outputs/r12'
with np.load(out/'v341_stationary/frame.npz') as archive:
    original={k:archive[k] for k in archive.files}
checks={};timings=[]
child=subprocess.Popen([sys.executable,__file__,'--reader'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
try:
    with SharedFrameSlot() as slot:
        slot_path=slot.path
        retained=None;previous_ack=None
        with SharedFrameSlot(capacity=1) as small:
            try:small.publish(original)
            except ValueError:checks['capacity_overflow_rejected']=True
            else:raise AssertionError('capacity overflow allowed')
        for i in range(20):
            arrays={k:v.copy() for k,v in original.items()}
            arrays['frame_index']=np.int64(i);arrays['stamp_ns']=np.int64(1_000_000_000+i*50_000_000)
            arrays['capture_monotonic_ns']=np.int64(int(original['capture_monotonic_ns'])+i)
            t=time.monotonic_ns();d=slot.publish(arrays);written=time.monotonic_ns()
            try:slot.publish(arrays)
            except ValueError:checks['overwrite_before_ack_rejected']=True
            else:raise AssertionError('overwrite allowed')
            try:slot.acknowledge(dict(session=d['session'],sequence=d['sequence'],token='wrong'))
            except ValueError:checks['wrong_ack_rejected']=True
            else:raise AssertionError('wrong ack accepted')
            if previous_ack is not None:
                try:slot.acknowledge(previous_ack)
                except ValueError:checks['previous_frame_ack_rejected']=True
                else:raise AssertionError('stale ack allowed')
            packet={'capture_run_id':str(arrays['run_id']),'stamp_ns':int(arrays['stamp_ns']),'frame_capture_monotonic_ns':int(arrays['capture_monotonic_ns'])}
            child.stdin.write(json.dumps({'descriptor':d,'packet':packet})+'\n');child.stdin.flush()
            response=json.loads(child.stdout.readline());received=time.monotonic_ns()
            assert response['hashes']==hashes(arrays) and response['identity']
            if i==0:
                retained,_=snapshot(d);retained_hash=hashes(retained)
                bad=copy.deepcopy(d);bad['entries'][0]['nbytes']+=1
                try:snapshot(bad)
                except ValueError:checks['malformed_bounds_rejected']=True
                else:raise AssertionError('malformed bounds allowed')
            slot.acknowledge(response['ack']);previous_ack=response['ack']
            try:slot.acknowledge(response['ack'])
            except ValueError:checks['duplicate_ack_rejected']=True
            else:raise AssertionError('duplicate ack accepted')
            assert hashes(retained)==retained_hash
            timings.append({'write_ms':(written-t)/1e6,'write_to_reader_hash_response_ms':(received-t)/1e6})
        checks['snapshot_survives_reuse']=True
    checks['slot_cleaned']=not slot_path.exists()
finally:
    child.stdin.close()
    try:child.wait(timeout=5)
    except subprocess.TimeoutExpired:child.terminate();child.wait(timeout=5)
report={'scope':'Standalone recorded-frame IPC prototype; not GUI/ROS performance or capture freshness evidence','frames':20,'all_member_hash_matches':20,'checks':checks,'reader_exit':child.returncode,'metrics':{k:{'median':float(np.median([v[k] for v in timings])),'max':max(v[k] for v in timings)} for k in timings[0]},'timings':timings,'runtime_integrated':False,'performance_pass':False}
(out/'v342_shared_frame_audit.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k!='timings'},indent=2))
