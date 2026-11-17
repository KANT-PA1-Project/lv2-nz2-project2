import json,re
from pathlib import Path
import cv2,numpy as np
from arm_practice.tag_detection import detect_tags
from arm_practice.observation_readiness import iou
out=Path(__file__).resolve().parents[3]/'outputs/r12';d=out/'mission_highres_v274';m=json.loads((d/'manifest.json').read_text());benchmark=json.loads((out/'v274_transport_benchmark.json').read_text());prior=next(r for r in benchmark['reports'] if r['environment']==cv2.__version__.rsplit('.',1)[0])
det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11));rows=[]
for row in prior['rows']:
 if not row['accepted']:continue
 index=row['index'];tags=detect_tags(cv2.imread(str(d/f'frame_{index:02d}.png')),det)
 assert [t['id'] for t in tags]==row['accepted']
 for t in tags:
  c=np.array(t['corners']);b=[*c.min(0).tolist(),*c.max(0).tolist()];projections=m['rows'][index]['diagnostic_projections'];best=max(projections,key=lambda p:iou(b,p['bbox']));expected=int(re.search(r'/Puck(\d+)/',best['path']).group(1));overlap=iou(b,best['bbox'])
  rows.append(dict(index=index,id=t['id'],corners=t['corners'],agreeing_scales=t['agreeing_scales'],nearest_projected_surface=best['path'],projected_bbox=best['bbox'],bbox_iou=overlap,id_matches_nearest_projection=t['id']==expected,pixel_coordinates_in_bounds=bool(np.all((c[:,0]>=0)&(c[:,0]<1280)&(c[:,1]>=0)&(c[:,1]<960))),scope='Projected mesh includes white tag margin;IoU is association evidence,not precise corner ground truth or occlusion validation'))
r=dict(opencv=cv2.__version__,rows=rows,geometry_used_only_for_scoring=True)
(out/f'v275_corners_{cv2.__version__}.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
