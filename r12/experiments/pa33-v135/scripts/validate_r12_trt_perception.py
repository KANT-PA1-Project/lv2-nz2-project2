"""Paired validation-split accuracy/latency smoke check; no mission acceptance."""
import argparse,json,sys,time
from pathlib import Path
import cv2,numpy as np,torch
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'src'))
sys.path.insert(0,'/home/seongmin/ros2_ws/src/turtlebot-swerve-cuvslam/perception/scripts')
from train_centerpose import load_model,infer
from ultralytics import YOLO
from arm_practice.trt_perception import DetectorEngine,PoseEngine
p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--precision',choices=['fp16','int8'],required=True);a=p.parse_args();torch.set_num_threads(2)
source=Path('/home/seongmin/ros2_ws/src/turtlebot-swerve-cuvslam/runs/perception');data=source/'data-v11-domain';meta=json.loads((a.out/'manifest.json').read_text());dataset=json.loads((data/'manifest.json').read_text());items=[s for s in dataset['samples'] if s['split']=='val'][::4][:25]
cpu_yolo=YOLO(a.out/'yolo.pt');cpu_pose=load_model(source/'train-v11-domain/centerpose/best.pt',device='cpu');trt_yolo=DetectorEngine(a.out/('yolo_'+a.precision+'.engine'),meta['names']);trt_pose=PoseEngine(a.out/('centerpose_'+a.precision+'.engine'),meta['pose_metric_depth_head'])
cam=dataset['camera_intrinsics'];K=np.array([[cam['fx'],0,cam['cx']],[0,cam['fy'],cam['cy']],[0,0,1.]])
rows=[]
def timed(fn):
 start=time.perf_counter();v=fn();return v,(time.perf_counter()-start)*1000
# Exclude cold start from paired latency, while reporting end-to-end mission separately.
z=np.zeros((480,640,3),np.uint8)
cpu_yolo.predict(z,imgsz=800,rect=False,device='cpu',verbose=False);trt_yolo.predict(z)
for model in (cpu_pose,trt_pose):infer(model,z,[100,100,200,200],K,device='cpu',views='scale3_median')
for sample in items:
 image=cv2.imread(str(data/sample['image']));ref,rt=timed(lambda:cpu_yolo.predict(image,imgsz=800,conf=.25,rect=False,device='cpu',verbose=False)[0]);pred,tt=timed(lambda:trt_yolo.predict(image)[0]);det=[]
 for box in ref.boxes:
  candidates=[b for b in pred.boxes if int(b.cls.item())==int(box.cls.item())]
  def iou(b):
   x=box.xyxy[0].numpy();y=b.xyxy[0].numpy();inter=np.maximum(0,np.minimum(x[2:],y[2:])-np.maximum(x[:2],y[:2])).prod();return float(inter/(np.prod(x[2:]-x[:2])+np.prod(y[2:]-y[:2])-inter+1e-9))
  det.append(max([iou(b) for b in candidates],default=0.))
 row={'image':sample['image'],'cpu_detector_ms':rt,'trt_detector_ms':tt,'reference_detections':len(ref.boxes),'engine_detections':len(pred.boxes),'reference_match_ious':det,'poses':[]}
 for box in [b for b in sample['boxes'] if b['class_id']<4][:1]:
  ref,rt=timed(lambda:infer(cpu_pose,image,box['xyxy'],K,device='cpu',views='scale3_median'));pred,tt=timed(lambda:infer(trt_pose,image,box['xyxy'],K,device='cpu',views='scale3_median'))
  r={'cpu_ms':rt,'trt_ms':tt,'valid_match':ref['valid']==pred['valid'],'translation_valid_match':ref['translation_valid']==pred['translation_valid']}
  if ref['translation_valid'] and pred['translation_valid']:r['translation_delta_m']=float(np.linalg.norm(np.array(ref['center_camera_m'])-pred['center_camera_m']))
  if ref.get('rotation_valid') and pred.get('rotation_valid'):
   R=np.array(ref['rotation_matrix']).T@np.array(pred['rotation_matrix']);r['rotation_delta_deg']=float(np.degrees(np.arccos(np.clip((np.trace(R)-1)/2,-1,1))))
  row['poses'].append(r)
 rows.append(row);print('VALIDATED',len(rows),flush=True)
poses=[p for r in rows for p in r['poses']];ious=[v for r in rows for v in r['reference_match_ious']]
summary={'precision':a.precision,'scope':'25 validation images, paired backend smoke test; not mission or real-hardware acceptance','images':len(rows),'pose_crops':len(poses),'cpu_detector_median_ms':float(np.median([r['cpu_detector_ms'] for r in rows])),'trt_detector_median_ms':float(np.median([r['trt_detector_ms'] for r in rows])),'cpu_pose_median_ms':float(np.median([r['cpu_ms'] for r in poses])),'trt_pose_median_ms':float(np.median([r['trt_ms'] for r in poses])),'minimum_detection_iou':min(ious,default=0),'detection_count_mismatches':sum(r['reference_detections']!=r['engine_detections'] for r in rows),'pose_validity_mismatches':sum(not r['valid_match'] or not r['translation_valid_match'] for r in poses),'max_translation_delta_m':max([r.get('translation_delta_m',0) for r in poses],default=0),'max_rotation_delta_deg':max([r.get('rotation_delta_deg',0) for r in poses],default=0),'performance_accepted':False}
(a.out/('validation_'+a.precision+'.json')).write_text(json.dumps({'summary':summary,'rows':rows},indent=2));print(json.dumps(summary),flush=True)
