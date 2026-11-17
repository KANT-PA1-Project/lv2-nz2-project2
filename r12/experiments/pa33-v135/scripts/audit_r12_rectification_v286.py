"""Diagnostic rectifications from detector corners, not fixture corners."""
import json
from pathlib import Path
import cv2,numpy as np
from arm_practice.observation_readiness import iou
out=Path(__file__).resolve().parents[3]/'outputs/r12';src=out/'mission_holdout_v282';meta=json.loads((src/'manifest.json').read_text())['rows'];cv2.setNumThreads(1)
lut=np.round((np.arange(256)/255.)**2*255).astype(np.uint8);det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11));records=[];tiles=[]
for index in [0,3,6,9,12,15]:
 r=meta[index];gray=cv2.LUT(cv2.cvtColor(cv2.imread(str(src/f'frame_{index:02d}.png')),cv2.COLOR_BGR2GRAY),lut)
 cs,ids,rejected=det.detectMarkers(cv2.resize(gray,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
 candidates=[('accepted',int(i),c[0]/4) for c,i in zip(cs,[] if ids is None else ids.flatten())]+[('rejected',None,c[0]/4) for c in rejected]
 # Diagnostic selection AFTER decoding, never decoder input.
 b=next(p['bbox'] for p in r['diagnostic_projections'] if p['path'].endswith('/Puck4/top/Surface'))
 for kind in ['accepted','rejected']:
  matches=[(iou([*c.min(0).tolist(),*c.max(0).tolist()],b),identity,c) for k,identity,c in candidates if k==kind]
  if not matches:continue
  score,identity,c=max(matches,key=lambda x:x[0])
  if score<.2:continue
  H=cv2.getPerspectiveTransform(c.astype(np.float32),np.array([[0,0],[159,0],[159,159],[0,159]],np.float32));warp=cv2.warpPerspective(gray,H,(160,160))
  cells=np.array([[np.mean(warp[y*20+5:y*20+15,x*20+5:x*20+15]) for x in range(8)] for y in range(8)])
  border=np.r_[cells[0],cells[-1],cells[1:-1,0],cells[1:-1,-1]];edges=np.linalg.norm(c-np.roll(c,1,axis=0),axis=1)
  records.append(dict(index=index,light=r['light_intensity'],offset=r['translation_from_reference_m'],kind=kind,id=identity,iou=score,corners=c.tolist(),min_edge_px=float(edges.min()),cell_means_8x8=cells.tolist(),border_mean=float(border.mean()),interior_mean=float(cells[1:-1,1:-1].mean())))
  tile=np.full((205,220,3),255,np.uint8);tile[40:200,30:190]=cv2.cvtColor(warp,cv2.COLOR_GRAY2BGR);cv2.putText(tile,f'{index} L{int(r["light_intensity"])} {kind}',(4,16),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,0,0),1);cv2.putText(tile,f'IoU {score:.2f} edge {edges.min():.1f}px',(4,33),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,0,0),1);tiles.append(tile)
while len(tiles)%4:tiles.append(np.full_like(tiles[0],255))
cv2.imwrite(str(out/'v286_rectifications.png'),np.vstack([np.hstack(tiles[i:i+4]) for i in range(0,len(tiles),4)]))
(out/'v286_rectification.json').write_text(json.dumps(dict(records=records,scope='4x detector corners warped for inspection only;8x8 cell grid meaningful only if corners coincide with code boundary. No fixture corners supplied to decoding. Synthetic diagnostic,not performance validation.'),indent=2)+'\n')
for r in records:print({k:v for k,v in r.items() if k not in ['corners','cell_means_8x8']})
