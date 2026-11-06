"""Offline spatial scoring only; projections never enter tag detection."""
import json,re
from pathlib import Path
import cv2,numpy as np
from arm_practice.tag_detection import detect_tags
from arm_practice.observation_readiness import iou
out=Path(__file__).resolve().parents[3]/'outputs/r12';src=out/'mission_repeat_v276'
m=json.loads((src/'manifest.json').read_text())['rows'];prior=json.loads((out/'v280_photometric_controls.json').read_text());cv2.setNumThreads(1)
lut=np.round((np.arange(256)/255.)**2*255).astype(np.uint8);clahe=cv2.createCLAHE(clipLimit=2.,tileGridSize=(8,8));rows=[]
for mode in ['gamma2','clahe2']:
 det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11))
 for r in prior['modes'][mode]['rows']:
  if 6 not in r['ids']:continue
  gray=cv2.cvtColor(cv2.imread(str(src/f'frame_{r["index"]:02d}.png')),cv2.COLOR_BGR2GRAY);im=cv2.LUT(gray,lut) if mode=='gamma2' else clahe.apply(gray)
  for t in detect_tags(im,det):
   if t['id']!=6:continue
   c=np.array(t['corners']);b=[*c.min(0).tolist(),*c.max(0).tolist()];best=max(m[r['index']]['diagnostic_projections'],key=lambda x:iou(b,x['bbox']));score=iou(b,best['bbox'])
   rows.append(dict(mode=mode,index=r['index'],**t,bbox=b,nearest_surface=best['path'],projection_iou=score,id_matches_nearest_surface=int(re.search(r'/Puck(\d+)/',best['path'])[1])==t['id']))
report=dict(rows=rows,scope='Offline nearest projected bbox association;no occlusion proof or runtime change',summary={mode:dict(faces=sum(r['mode']==mode for r in rows),matched=sum(r['mode']==mode and r['id_matches_nearest_surface'] for r in rows),min_iou=min(r['projection_iou'] for r in rows if r['mode']==mode)) for mode in ['gamma2','clahe2']})
(out/'v281_added_id_audit.json').write_text(json.dumps(report,indent=2)+'\n');print(report['summary'])
