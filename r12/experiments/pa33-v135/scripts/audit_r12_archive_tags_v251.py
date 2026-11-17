"""Read-only original image archive scan; decode-only results are not identity truth."""
import json,sys,time
from pathlib import Path
import cv2,numpy as np
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'src'))
from arm_practice.tag_detection import detect_tags
out=root.parents[1]/'outputs/r12';files=sorted((root/'reports/local/observed_history_v159/observation_history').glob('*.npz'))
files+=sorted((root/'reports/local').glob('*/terminal_observation/*.npz'))
files=list(dict.fromkeys(files));cv2.setNumThreads(1);det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11));rows=[]
for idx,path in enumerate(files):
 with np.load(path,allow_pickle=False) as f:
  for camera in ['r07_camera1','r07_camera2','r07_camera3','arm_camera']:
   key='rgb' if camera=='r07_camera1' else camera+'_left_optical'
   if key not in f.files:continue
   image=f[key][0] if key=='rgb' else f[key];gray=cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)
   corners,ids,_=det.detectMarkers(gray);native=[]
   if ids is not None:
    for c,tag in zip(corners,ids.flatten()):
     pts=c[0];native.append(dict(id=int(tag),corners=pts.tolist(),min_edge_px=float(np.min(np.linalg.norm(pts-np.roll(pts,1,axis=0),axis=1)))))
   agreed=detect_tags(gray,det)
   rows.append(dict(file=str(path),camera=camera,native=native,agreed=agreed))
 if idx%10==0:print('files',idx+1,'/',len(files),flush=True)
r=dict(files=len(files),images=len(rows),native_positive_images=sum(bool(x['native']) for x in rows),agreed_positive_images=sum(bool(x['agreed']) for x in rows),rows=rows,scope='Archived simulator images only; decoded IDs not validated against independent truth; no observations injected')
(out/'v251_archive_tag_scan.json').write_text(json.dumps(r,indent=2)+'\n');print({k:v for k,v in r.items() if k!='rows'})
