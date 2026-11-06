import json,statistics
from pathlib import Path
out=Path(__file__).resolve().parents[3]/'outputs/r12';p=out/'v352_stationary'
rows=[json.loads(x) for x in (p/'gpu_acquisition_profile.jsonl').read_text().splitlines()]
def stats(values):return {'n':len(values),'median_ms':statistics.median(values),'max_ms':max(values)}
keys=['annotation_wall_ms','warp_sync_wall_ms','mask_clone_pack_submit_wall_ms','d2h_wait_wall_ms','unpack_wall_ms','torch_stream_elapsed_ms']
report={'frames':len(rows),'metrics':{k:stats([r[k] for r in rows]) for k in keys},'mask_stream_sum':stats([sum(x['ms'] for x in r['mask_stream_elapsed_ms']) for r in rows]),'mask_submit_wall_sum':stats([sum(r['mask_submit_wall_ms']) for r in rows]),'first_frame_devices':rows[0]['devices'],'scope':'Profiled run. CUDA event intervals include stream scheduling/host submission gaps; sums overlap wall intervals and must not be added to them. No performance pass.'}
(out/'v352_profile_summary.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
