"""Frozen-model exploratory diagnostics on one old rendered wrist image."""
import json
from pathlib import Path
import cv2,numpy as np,torch
root=Path(__file__).resolve().parents[1];base=root.parent.parent/'outputs/r12';torch.set_num_threads(2)
net=torch.nn.Sequential(torch.nn.Conv2d(1,8,5,stride=2,padding=2),torch.nn.ReLU(),torch.nn.Conv2d(8,16,3,stride=2,padding=1),torch.nn.ReLU(),torch.nn.Conv2d(16,24,3,stride=2,padding=1),torch.nn.ReLU(),torch.nn.Flatten(),torch.nn.Linear(24*8*8,32),torch.nn.ReLU(),torch.nn.Linear(32,1))
net.load_state_dict(torch.load(base/'tag_quality_v184/model.pt',weights_only=False)['weights']);net.eval();threshold=json.loads((base/'tag_quality_v184/evaluation.json').read_text())['validation_selected_threshold'];raw=json.loads((base/'v186_raw_positive_check.json').read_text())
with np.load(raw['source']) as d:rgb=d['arm_camera_left_optical'].copy()
gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY);annot=rgb.copy();rows=[];det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11))
for i,t in enumerate(raw['decoded_tags']):
 c=np.array(t['corners']);x1,y1=c.min(axis=0);x2,y2=c.max(axis=0);pad=.25*max(x2-x1,y2-y1);crop=gray[max(0,int(y1-pad)):min(480,int(np.ceil(y2+pad))),max(0,int(x1-pad)):min(640,int(np.ceil(x2+pad)))];f=min(1.,60/max(crop.shape));small=cv2.resize(crop,(max(1,round(crop.shape[1]*f)),max(1,round(crop.shape[0]*f))),interpolation=cv2.INTER_AREA);im=np.full((64,64),255,np.uint8);y=(64-small.shape[0])//2;x=(64-small.shape[1])//2;im[y:y+small.shape[0],x:x+small.shape[1]]=small
 _,ids,_=det.detectMarkers(im)
 with torch.no_grad():score=float(net(torch.tensor(im[None,None]/255,dtype=torch.float32)).sigmoid().item())
 rows.append(dict(region=i,full_frame_multiscale_id=t['id'],input64_decoded_ids=[] if ids is None else ids.flatten().tolist(),score=score,predicted_readable=score>=threshold,crop_shape=list(crop.shape)))
 cv2.polylines(annot,[c.astype(np.int32)],True,(255,30,30),1);cv2.putText(annot,f'{i}: ID{t["id"]}',(int(x1),int(y1)-5),cv2.FONT_HERSHEY_SIMPLEX,.4,(255,30,30),1)
cv2.imwrite(str(base/'v187_positive_regions.png'),cv2.cvtColor(annot,cv2.COLOR_RGB2BGR))
r=dict(complete=True,threshold=threshold,regions=rows,scope='One old rendered image with no timestamp. Decoder-selected regions, selection-biased; IDs0/11 overlap synthetic training/test identities. Not independent performance evidence, no retraining/tuning/deployment.')
(base/'v187_positive_diagnostic.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
