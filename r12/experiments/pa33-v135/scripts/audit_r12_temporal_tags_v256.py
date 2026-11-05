"""Saved-frame native versus deployed multiscale decoder comparison."""
import json,sys,time
from pathlib import Path
import numpy as np,cv2
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'src'))
from arm_practice.tag_detection import detect_tags
out=root.parents[1]/'outputs/r12';src=out/'tag_surface_v255';meta=json.loads((src/'results.json').read_text())['rows'];cv2.setNumThreads(1);det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11));rows=[]
# Warm-up is excluded, image loading excluded from decoder-only timing.
images=[cv2.imread(str(src/f'frame_{i+1:02d}.png')) for i in range(len(meta))]
for _ in range(3):detect_tags(images[0],det)
for i,(m,image) in enumerate(zip(meta,images)):
 samples=[];decoded=[]
 for _ in range(3):
  start=time.perf_counter();got=detect_tags(image,det);samples.append(time.perf_counter()-start);decoded.append([x['id'] for x in got])
 rows.append(dict(frame=i,native=m['decoded'],multiscale=decoded[0],consistent_repeats=all(x==decoded[0] for x in decoded),seconds=samples,agreeing_scales=[x['agreeing_scales'] for x in got]))
def metrics(key):
 failures=[x['frame'] for x in rows if x[key]!=[0]];run=longest=0
 for x in rows:
  run=run+1 if x[key]!=[0] else 0;longest=max(longest,run)
 return dict(correct=len(rows)-len(failures),total=len(rows),failed_frames=failures,max_consecutive_failures=longest,wrong_ids=[x['frame'] for x in rows if x[key] and x[key]!=[0]])
times=[t for r in rows for t in r['seconds']];r=dict(native=metrics('native'),multiscale=metrics('multiscale'),decoder_only_timing=dict(median_s=float(np.median(times)),p95_s=float(np.quantile(times,.95))),rows=rows,scope='One static curved tag20cm,20 saved renderer frames; no measured real-time sensor cadence or independent scenes',runtime_changed=False)
(out/'v256_temporal_tag_audit.json').write_text(json.dumps(r,indent=2)+'\n');print({k:v for k,v in r.items() if k!='rows'})
