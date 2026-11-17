"""Fixed full-frame preprocessing controls; no fixture ROI in decoder inputs."""
import json,time,hashlib
from pathlib import Path
from collections import defaultdict
import cv2,numpy as np
from arm_practice.tag_detection import detect_tags
out=Path(__file__).resolve().parents[3]/'outputs/r12';cv2.setNumThreads(1)
pos=out/'mission_repeat_v276';neg=out/'mission_highres_negative_v275';meta=json.loads((pos/'manifest.json').read_text())['rows'];jobs=[]
for r in meta:jobs.append(dict(kind='positive',index=r['index'],offset=r['translation_from_reference_m'],light=r['light_intensity'],path=pos/f'frame_{r["index"]:02d}.png'))
for i in range(32):jobs.append(dict(kind='negative',index=i,path=neg/f'frame_{i:02d}.png'))
images=[cv2.cvtColor(cv2.imread(str(j['path'])),cv2.COLOR_BGR2GRAY) for j in jobs]
lut=np.round((np.arange(256)/255.)**2*255).astype(np.uint8);clahe=cv2.createCLAHE(clipLimit=2.,tileGridSize=(8,8))
report=dict(opencv=cv2.__version__,scope='Already consumed synthetic frames;full-frame fixed gamma2 or CLAHE2/8x8;no tag location or ID in preprocessing. Exploratory,not held-out validation.',modes={},runtime_changed=False)
for mode in ['baseline','gamma2','clahe2']:
 det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11));rows=[]
 for job,gray in zip(jobs,images):
  start=time.perf_counter();im=gray if mode=='baseline' else cv2.LUT(gray,lut) if mode=='gamma2' else clahe.apply(gray)
  tags=detect_tags(im,det);ids=[t['id'] for t in tags]
  rows.append(dict(kind=job['kind'],index=job['index'],offset=job.get('offset'),light=job.get('light'),ids=ids,processing_ms=(time.perf_counter()-start)*1000))
 groups=defaultdict(lambda:dict(frames=0,id4_present=0,other_id_frames=0,negative_false_positive=0))
 for r in rows:
  key='negative' if r['kind']=='negative' else str((r['offset'],r['light']));g=groups[key];g['frames']+=1
  if r['kind']=='negative':g['negative_false_positive']+=bool(r['ids'])
  else:g['id4_present']+=4 in r['ids'];g['other_id_frames']+=any(x!=4 for x in r['ids'])
 report['modes'][mode]=dict(groups=dict(groups),median_processing_ms=float(np.median([r['processing_ms'] for r in rows])),rows=rows)
 (out/'v280_photometric_controls.json').write_text(json.dumps(report,indent=2)+'\n');print(mode,dict(groups),flush=True)
