"""Saved-frame paired sampling controls; timing excludes disk I/O and sensors."""
import json,time,hashlib
from pathlib import Path
import cv2,numpy as np
from arm_practice.tag_detection import detect_tags
out=Path(__file__).resolve().parents[3]/'outputs/r12';cv2.setNumThreads(1);lut=np.round((np.arange(256)/255.)**2*255).astype(np.uint8)
det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11));jobs=[]
for kind,stem in [('positive','mission_holdout'),('negative','mission_holdout_negative')]:
 for r in json.loads((out/f'{stem}_v288/manifest.json').read_text())['rows']:
  low=cv2.imread(str(out/f'{stem}_v282'/f'frame_{r["index"]:02d}.png'));high=cv2.imread(str(out/f'{stem}_v288'/f'frame_{r["index"]:02d}.png'));jobs.append((kind,r,low,high))
rows=[];summary={}
for mode in ['native1280','native1920','downsample1920to1280']:
 for repeat in range(2):
  for kind,r,low,high in jobs:
   start=time.perf_counter();im=low if mode=='native1280' else high if mode=='native1920' else cv2.resize(high,(1280,960),interpolation=cv2.INTER_AREA)
   gray=cv2.LUT(cv2.cvtColor(im,cv2.COLOR_BGR2GRAY),lut);tags=detect_tags(gray,det);elapsed=(time.perf_counter()-start)*1000
   rows.append(dict(kind=kind,index=r['index'],light=r['light_intensity'],mode=mode,repeat=repeat,tags=tags,processing_ms=elapsed,pixel_sha256=hashlib.sha256(gray.tobytes()).hexdigest()))
 rs=[r for r in rows if r['mode']==mode];first=[r for r in rs if r['repeat']==0];second=[r for r in rs if r['repeat']==1]
 summary[mode]=dict(positive_per_repeat=18,id4_by_repeat=[sum(any(t['id']==4 for t in r['tags']) for r in rs if r['kind']=='positive' and r['repeat']==rep) for rep in range(2)],bright_id4_by_repeat=[sum(any(t['id']==4 for t in r['tags']) for r in rs if r['kind']=='positive' and r['light']==2400 and r['repeat']==rep) for rep in range(2)],negative_false_positive=sum(bool(r['tags']) for r in rs if r['kind']=='negative'),negative_evaluations=36,exact_tag_repeat_matches=sum(a['tags']==b['tags'] for a,b in zip(first,second)),repeat_pairs=36,median_ms=float(np.median([r['processing_ms'] for r in rs])),p95_ms=float(np.quantile([r['processing_ms'] for r in rs],.95)),max_ms=max(r['processing_ms'] for r in rs))
 report=dict(opencv=cv2.__version__,summary=summary,rows=rows,scope='Two offline repeats of same saved images;not sensor temporal robustness. Single-thread preprocessing+decoder timing excludes disk,IPC,rendering,YOLO,CenterPose. Fixed gamma2 applied after INTER_AREA resize;no runtime changes.')
 (out/f'v289_sampling_{cv2.__version__}.json').write_text(json.dumps(report,indent=2)+'\n');print(mode,summary[mode],flush=True)
