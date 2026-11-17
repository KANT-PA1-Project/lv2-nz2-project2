"""Read-only decoder diagnostics; never train on V181 audit samples."""
import json
from pathlib import Path
from collections import Counter
import cv2
import numpy as np
root=Path(__file__).resolve().parents[1]; out=root.parent.parent/'outputs/r12';run=root/'reports/local/observed_history_v159'
audit=json.loads((out/'v181_tag_quality_archive_evaluation.json').read_text());det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11))
def ids(im):
 _,v,_=det.detectMarkers(im)
 return [] if v is None else v.flatten().tolist()
controls={str(i):ids(cv2.imread(str(root/f'models/navigation/textures/tag36h11-{i}.png'),0)) for i in range(12)}
counts=Counter();hits=[];shapes=[]
by_stamp={}
for row in audit['rows']:by_stamp.setdefault(row['stamp_ns'],[]).append(row)
for file in sorted((run/'observation_history').glob('*.npz')):
 with np.load(file) as d:
  stamp=int(d['stamp_ns'])
  for r in by_stamp.get(stamp,[]):
   camera=r['camera'];rgb=d['rgb'][0] if camera=='r07_camera1' else d[camera+'_left_optical'];gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY)
   x1,y1,x2,y2=r['box'];pad=.25*max(x2-x1,y2-y1);crop=gray[max(0,int(y1-pad)):min(gray.shape[0],int(np.ceil(y2+pad))),max(0,int(x1-pad)):min(gray.shape[1],int(np.ceil(x2+pad)))];shapes.append(min(crop.shape))
   for name,im in [('native',crop),('up2',cv2.resize(crop,None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC)),('up4',cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC)),('mirror_up2',cv2.resize(cv2.flip(crop,1),None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC))]:
    found=ids(im);counts[name]+=bool(found)
    if found:hits.append(dict(stamp_ns=stamp,camera=camera,box=r['box'],variant=name,decoded_ids=found))
result=dict(complete=True,crops=len(shapes),texture_control_decodes=controls,crop_decode_counts=dict(counts),crop_short_side_px_percentiles=np.percentile(shapes,[0,50,95,100]).tolist(),hits=hits,scope='Diagnostic only. Upsampling/mirroring does not establish correct identity; V181 remains excluded from future training/tuning. No hardware images, arm motion or runtime changes.')
(out/'v182_decoder_diagnostic.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='hits'},indent=2))
