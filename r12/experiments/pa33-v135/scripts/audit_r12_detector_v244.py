"""Offline frozen detector sensitivity audit; no runtime settings are modified."""
import json,time,sys
from pathlib import Path
import numpy as np,cv2,torch
from ultralytics import YOLO
root=Path(__file__).resolve().parents[1];out=root.parents[1]/'outputs/r12'
audit=json.loads((out/'v243_visible_puck_audit.json').read_text())
source=json.loads((out/'v241_worker_replay/results.json').read_text())
torch.set_num_threads(2)
model=YOLO('/home/seongmin/ros2_ws/src/turtlebot-swerve-cuvslam/runs/perception/train-v11-domain/yolo/weights/best.pt')
def area(b):return max(0,b[2]-b[0])*max(0,b[3]-b[1])
def inter(a,b):return max(0,min(a[2],b[2])-max(a[0],b[0]))*max(0,min(a[3],b[3])-max(a[1],b[1]))
def iou(a,b):
 v=inter(a,b);return v/(area(a)+area(b)-v) if area(a)+area(b)>v else 0
rows=[]
for ann in audit['annotations']:
 sample=source['samples'][ann['sample_index']]
 with np.load(sample['scene'],allow_pickle=False) as f:image=cv2.cvtColor(f['rgb'][0] if sample['camera']=='r07_camera1' else f[sample['camera']+'_left_optical'],cv2.COLOR_RGB2BGR)
 for name,size,conf,tiles in [('full800',800,.25,False),('low_conf800',800,.05,False),('full1280',1280,.25,False),('fixed_tiles800',800,.25,True)]:
  start=time.monotonic();pred=[]
  windows=[(0,0,640,480)] if not tiles else [(x,y,x+400,y+320) for x in (0,240) for y in (0,160)]
  for x,y,u,v in windows:
   result=model.predict(image[y:v,x:u],imgsz=size,conf=conf,device='cpu',verbose=False)[0]
   for b in result.boxes:
    cls=int(b.cls.item())
    if cls>=4:continue
    box=b.xyxy[0].tolist();box=[box[0]+x,box[1]+y,box[2]+x,box[3]+y]
    pred.append(dict(bbox=box,class_name=result.names[cls],confidence=float(b.conf.item())))
  ordered=sorted(pred,key=lambda d:-d['confidence']);pred=[]
  for d in ordered:
   if not tiles or not any(d['class_name']==p['class_name'] and iou(d['bbox'],p['bbox'])>.5 for p in pred):pred.append(d)
  used=set();tp=fp=wrong=ignored=0;matches=[]
  for d in pred:
   if any(inter(d['bbox'],box)/area(d['bbox'])>.5 for box in ann['ignore']):ignored+=1;continue
   score,j=max([(iou(d['bbox'],o['bbox']),j) for j,o in enumerate(ann['objects']) if j not in used],default=(0,-1))
   if score>=.5:
    used.add(j);tp+=1;wrong+=d['class_name']!=ann['objects'][j]['class_name'];matches.append(dict(object_index=j,confidence=d['confidence'],predicted=d['class_name']))
   else:fp+=1
  row=dict(camera=sample['camera'],mode=name,seconds=time.monotonic()-start,tp=tp,fn=len(ann['objects'])-tp,fp=fp,class_wrong=wrong,ignored=ignored,matches=matches,detections=pred);rows.append(row)
  (out/'v244_detector_audit.json').write_text(json.dumps(dict(scope='2 manually annotated archived views; timing un-warmed and detector-only; exploratory, not independent validation',rows=rows),indent=2)+'\n')
  print(json.dumps({k:v for k,v in row.items() if k not in ('detections','matches')}),flush=True)
