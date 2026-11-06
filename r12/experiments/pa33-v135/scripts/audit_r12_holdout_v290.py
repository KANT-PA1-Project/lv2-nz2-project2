"""Explore wider threshold window range; strict existing tag consensus unchanged."""
import json,time
from pathlib import Path
import cv2,numpy as np
from arm_practice.tag_detection import detect_tags
out=Path(__file__).resolve().parents[3]/'outputs/r12';cv2.setNumThreads(1);lut=np.round((np.arange(256)/255.)**2*255).astype(np.uint8)
p=cv2.aruco.DetectorParameters();original=p.adaptiveThreshWinSizeMax;p.adaptiveThreshWinSizeMax=original
params=dict(adaptiveThreshWinSizeMin=p.adaptiveThreshWinSizeMin,adaptiveThreshWinSizeMax=p.adaptiveThreshWinSizeMax,adaptiveThreshWinSizeStep=p.adaptiveThreshWinSizeStep,original_max=original)
det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),p);rows=[]
for kind,folder in [('positive','mission_holdout_v290'),('negative','mission_holdout_negative_v290')]:
 src=out/folder
 for r in json.loads((src/'manifest.json').read_text())['rows']:
  im=cv2.imread(str(src/f'frame_{r["index"]:02d}.png'));start=time.perf_counter();im=cv2.resize(im,(1280,960),interpolation=cv2.INTER_AREA);gray=cv2.LUT(cv2.cvtColor(im,cv2.COLOR_BGR2GRAY),lut)
  tags=detect_tags(gray,det);rows.append(dict(kind=kind,index=r['index'],light=r['light_intensity'],processing_ms=(time.perf_counter()-start)*1000,tags=tags))
summary={str(light):dict(frames=6,id4=sum(any(t['id']==4 for t in r['tags']) for r in rows if r['kind']=='positive' and r['light']==light)) for light in [750.,1500.,2700.]}
summary['negative_false_positive']=sum(bool(r['tags']) for r in rows if r['kind']=='negative')
report=dict(opencv=cv2.__version__,parameters=params,rows=rows,summary=summary,scope='Matched synthetic conditions with new native resolution;unused poses/lights native1920x1440->1280x960 INTER_AREA default detector;fixed gamma2 and consensus unchanged;no runtime integration')
(out/f'v290_holdout_{cv2.__version__}.json').write_text(json.dumps(report,indent=2)+'\n');print(summary)
