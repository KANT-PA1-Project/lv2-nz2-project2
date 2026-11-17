"""Frozen-output ambiguity and image-only cropped tag diagnostic."""
import json,sys,time
from pathlib import Path
import cv2,numpy as np
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'src'))
from arm_practice.tag_detection import agreed_tags
out=root.parents[1]/'outputs/r12';samples=json.loads((out/'v247_worker_replay/results.json').read_text())['samples']
detector=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11));cv2.setNumThreads(1)
def iou(a,b):
 v=max(0,min(a[2],b[2])-max(a[0],b[0]))*max(0,min(a[3],b[3])-max(a[1],b[1]));aa=(a[2]-a[0])*(a[3]-a[1]);bb=(b[2]-b[0])*(b[3]-b[1]);return v/(aa+bb-v)
rows=[];conflicts=[]
for sample in samples:
 if sample['repeat']!=0:continue
 with np.load(sample['scene'],allow_pickle=False) as f:rgb=f['rgb'][0] if sample['camera']=='r07_camera1' else f[sample['camera']+'_left_optical']
 gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY);ds=[d for d in sample['detection_records'] if 'pose' in d]
 for i,d in enumerate(ds):
  for e in ds[i+1:]:
   overlap=iou(d['bbox'],e['bbox'])
   if overlap>=.5 and d['class_name']!=e['class_name']:conflicts.append(dict(scene=sample['scene'],camera=sample['camera'],iou=overlap,classes=[d['class_name'],e['class_name']],confidence=[d['confidence'],e['confidence']],decision='ambiguous; no forced shape label'))
  x,y,u,v=d['bbox'];pad=.25*max(u-x,v-y);x=max(0,int(x-pad));y=max(0,int(y-pad));u=min(640,int(u+pad));v=min(480,int(v+pad));crop=gray[y:v,x:u];obs=[];rejected=[];start=time.monotonic()
  for scale in (1,2,3,4):
   scaled=crop if scale==1 else cv2.resize(crop,None,fx=scale,fy=scale,interpolation=cv2.INTER_CUBIC)
   corners,ids,bad=detector.detectMarkers(scaled);rejected.append(len(bad))
   if ids is not None:obs.extend((scale,int(identity),corner[0]/scale+np.array([x,y])) for corner,identity in zip(corners,ids.flatten()))
  rows.append(dict(scene=sample['scene'],camera=sample['camera'],bbox=d['bbox'],crop_size=[u-x,v-y],raw_ids=[o[1] for o in obs],accepted_tags=agreed_tags(obs),rejected_quad_counts=rejected,seconds=time.monotonic()-start))
report=dict(scope='Original archived pixels,25percent bbox padding,scales1-4; no fixture IDs or enhanced/generated pixels',crop_count=len(rows),decoded_crop_count=sum(bool(x['accepted_tags']) for x in rows),raw_decoded_crop_count=sum(bool(x['raw_ids']) for x in rows),conflicts=conflicts,rows=rows,runtime_changed=False)
(out/'v249_tag_ambiguity_audit.json').write_text(json.dumps(report,indent=2)+'\n');print({k:v for k,v in report.items() if k!='rows'})
