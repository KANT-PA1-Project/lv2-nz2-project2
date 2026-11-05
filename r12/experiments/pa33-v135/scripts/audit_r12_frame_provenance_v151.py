"""Audit same-host recorded frame timing; never authorize actual execution."""
import json,time
from pathlib import Path
import numpy as np
from arm_practice.frame_packet_identity import check_live_frame_provenance
root=Path(__file__).resolve().parents[1];r=root/'reports/local/diagnostic_frame_provenance_v150';rows=json.loads((r/'ros_results.json').read_text());frame=np.load(r/'frame.npz',allow_pickle=False);run=str(frame['run_id']);results=[]
for i,p in enumerate(rows):
 # Intermediate images were not retained. These metadata-only checks do not certify their bytes.
 meta=dict(frame_index=np.int64(i),stamp_ns=np.int64(p['stamp_ns']),clock_domain=np.array('synthetic_packet_20hz'),run_id=np.array(p['capture_run_id']),capture_monotonic_ns=np.int64(p['frame_capture_monotonic_ns']))
 check=check_live_frame_provenance(meta,p,i,run,p['packet_processed_monotonic_ns'],250_000_000)
 results.append(dict(index=i,age_ms=(p['packet_processed_monotonic_ns']-p['frame_capture_monotonic_ns'])/1e6,eligibility=check,visual_pair_wait=p.get('visual_pair_wait')))
last=check_live_frame_provenance(frame,rows[-1],len(rows)-1,run,time.monotonic_ns(),250_000_000)
wrong=check_live_frame_provenance(frame,rows[-1],len(rows)-1,'different-run',rows[-1]['packet_processed_monotonic_ns'],250_000_000)
assert not last['matched'] and last['reason']=='stale_frame';assert not wrong['matched'];assert not results[0]['eligibility']['matched'];assert not results[20]['eligibility']['matched']
e=dict(complete=True,frames=len(rows),metadata_eligible_at_recorded_processing=sum(x['eligibility']['matched'] for x in results),rows=results,last_stored_image_now=last,cross_run_rejected=not wrong['matched'],execution_authorized=False,scope='250ms diagnostic freshness criterion. Only final image bytes retained and directly matched. Intermediate metadata from packets; no live IK integration or motion authorization. First frame also invalid encoder velocity; startup visual wait timed out. Frame20 latency cause unconfirmed.')
(root.parent.parent/'outputs/r12/v151_provenance_eligibility.json').write_text(json.dumps(e,indent=2)+'\n');print({k:v for k,v in e.items() if k!='rows'})
