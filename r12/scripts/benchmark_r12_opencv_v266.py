"""Same saved pixels, fixed single-thread decoding; no package mutations."""
import json,time,hashlib,sys
from pathlib import Path
import cv2,numpy as np
from arm_practice.tag_detection import detect_tags
root=Path(__file__).resolve().parents[3]/'outputs/r12';src=root/'tag_render_v230'
meta=json.loads((src/'manifest.json').read_text())['rows'];images=[];digest=hashlib.sha256()
for r in meta:
 image=cv2.imread(str(src/f'frame_{r["index"]:03d}.png'));assert image is not None
 digest.update(image.tobytes());images.append(image)
cv2.setNumThreads(1);det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11))
for _ in range(3):detect_tags(images[0],det)
rows=[];times=[]
for repeat in range(3):
 for r,image in zip(meta,images):
  start=time.perf_counter_ns();tags=detect_tags(image,det);ms=(time.perf_counter_ns()-start)/1e6;times.append(ms)
  rows.append(dict(index=r['index'],repeat=repeat,expected=r['tag_id'],accepted=[x['id'] for x in tags],decode_ms=ms))
first=[r for r in rows if r['repeat']==0];pos=[r for r in first if r['expected']>=0];neg=[r for r in first if r['expected']<0]
summary=dict(opencv_version=cv2.__version__,python=sys.executable,pixel_sha256=digest.hexdigest(),frames=len(meta),timed_decodes=len(rows),
 positive_correct=sum(r['accepted']==[r['expected']] for r in pos),positive_total=len(pos),wrong_positive=sum(any(t!=r['expected'] for t in r['accepted']) for r in pos),negative_false_positive=sum(bool(r['accepted']) for r in neg),negative_total=len(neg),
 decode_median_ms=float(np.median(times)),decode_p95_ms=float(np.quantile(times,.95)),decode_max_ms=max(times),
 repeat_disagreements=sum(any(r['accepted']!=first[i]['accepted'] for r in rows if r['index']==f['index']) for i,f in enumerate(first)),
 scope='Previously consumed V230 synthetic corpus;decoder-only timing,one thread;not end-to-end or new holdout',runtime_changed=False)
(root/f'v266_opencv_{cv2.__version__}.json').write_text(json.dumps(dict(summary=summary,rows=rows),indent=2)+'\n');print(json.dumps(summary,indent=2))
