"""Explore wider threshold window range; strict existing tag consensus unchanged."""
import json
from pathlib import Path
import cv2,numpy as np
from arm_practice.tag_detection import detect_tags
out=Path(__file__).resolve().parents[3]/'outputs/r12';cv2.setNumThreads(1);lut=np.round((np.arange(256)/255.)**2*255).astype(np.uint8)
p=cv2.aruco.DetectorParameters();original=p.adaptiveThreshWinSizeMax;p.adaptiveThreshWinSizeMax=original
params=dict(adaptiveThreshWinSizeMin=p.adaptiveThreshWinSizeMin,adaptiveThreshWinSizeMax=p.adaptiveThreshWinSizeMax,adaptiveThreshWinSizeStep=p.adaptiveThreshWinSizeStep,original_max=original)
det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),p);rows=[]
for kind,folder in [('positive','mission_holdout_v288'),('negative','mission_holdout_negative_v288')]:
 src=out/folder
 for r in json.loads((src/'manifest.json').read_text())['rows']:
  gray=cv2.LUT(cv2.cvtColor(cv2.imread(str(src/f'frame_{r["index"]:02d}.png')),cv2.COLOR_BGR2GRAY),lut)
  tags=detect_tags(gray,det);rows.append(dict(kind=kind,index=r['index'],light=r['light_intensity'],tags=tags))
summary={str(light):dict(frames=6,id4=sum(any(t['id']==4 for t in r['tags']) for r in rows if r['kind']=='positive' and r['light']==light)) for light in [900.,1200.,2400.]}
summary['negative_false_positive']=sum(bool(r['tags']) for r in rows if r['kind']=='negative')
report=dict(opencv=cv2.__version__,parameters=params,rows=rows,summary=summary,scope='Matched synthetic conditions with new native resolution;native1920x1440 default detector;fixed gamma2 and consensus unchanged;no runtime integration')
(out/f'v288_native_{cv2.__version__}.json').write_text(json.dumps(report,indent=2)+'\n');print(summary)
