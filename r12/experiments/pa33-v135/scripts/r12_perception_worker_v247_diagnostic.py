"""V247 OFFLINE DIAGNOSTIC ONLY: enlarged input and unvalidated size precheck."""
import sys,json,hashlib,time
from pathlib import Path
import numpy as np,cv2,torch
root=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(root/'src'),str(root/'work/perception_deps'),'/home/seongmin/ros2_ws/src/turtlebot-swerve-cuvslam/perception/scripts']
from arm_practice.tag_detection import detect_tags
from arm_practice.perception_transport import frame_identity, require_response
import zxingcpp
from ultralytics import YOLO
from train_centerpose import load_model,infer
run=Path('/home/seongmin/ros2_ws/src/turtlebot-swerve-cuvslam/runs/perception/train-v11-domain')
cfg=json.loads((run/'inference.json').read_text());torch.set_num_threads(2)
yolo=YOLO(run/'yolo/weights/best.pt');pose=load_model(run/'centerpose/best.pt',device='cpu')
tags=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11))
K=np.array([[400.,0,320],[0,400,240],[0,0,1.]])
print(json.dumps({'ready':True,'sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [run/'yolo/weights/best.pt',run/'centerpose/best.pt',run/'inference.json']}}),flush=True)
for line in sys.stdin:
    req=json.loads(line)
    with np.load(req['frame'],allow_pickle=False) as archive:data={k:archive[k] for k in archive.files}
    identity=frame_identity(data,req['camera']);started=time.monotonic_ns()
    require_response(req,dict(identity,inference_started_monotonic_ns=started,inference_completed_monotonic_ns=started),req['max_age_ns'])
    rgb=data['rgb'][0];image=cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR);start=time.monotonic()
    decoded=[r for r in zxingcpp.read_barcodes(image) if r.text in ['P2-01','P2-02','P2-03','P2-04']]
    codes=[r.text for r in decoded]
    barcode_boxes=[]
    for r in decoded:
        pts=[r.position.top_left,r.position.top_right,r.position.bottom_left,r.position.bottom_right]
        barcode_boxes.append([min(p.x for p in pts),min(p.y for p in pts),max(p.x for p in pts),max(p.y for p in pts)])
    camera=req.get('camera','r07_camera1');camera_rgb=rgb if camera=='r07_camera1' else data[camera+'_left_optical'];image=cv2.cvtColor(camera_rgb,cv2.COLOR_RGB2BGR);camera_depth=data['depth'] if camera=='r07_camera1' else data[camera+'_depth']
    taglist=detect_tags(image,tags)
    detections=[]
    if req.get('detect',False):
        result=yolo.predict(image,imgsz=1280,conf=cfg['detector_confidence'],device='cpu',verbose=False)[0]
        for box in result.boxes:
            cls=int(box.cls.item());xy=box.xyxy[0].tolist();record={'class_id':cls,'class_name':result.names[cls],'bbox':xy,'confidence':float(box.conf.item())}
            if cls<4:
                x0,y0,x1,y1=xy;cx=int((x0+x1)/2);cy=int((y0+y1)/2)
                patch=camera_depth[max(0,cy-2):min(480,cy+3),max(0,cx-2):min(640,cx+3)]
                valid_depth=patch[np.isfinite(patch)&(patch>.02)&(patch<10)]
                surface=float(np.median(valid_depth)) if valid_depth.size else None
                extent=None if surface is None else max(x1-x0,y1-y0)*surface/400
                record['diagnostic_extent_m']=extent
                if extent is None or extent>.10:
                    record['diagnostic_pose_skipped']='unvalidated_size_precheck'
                    detections.append(record)
                    continue
                estimate=infer(pose,image,xy,K,device='cpu',dimensions_m=(.03,.03,.06),views=cfg['pose_views'])
                record['pose']=estimate
                x0,y0,x1,y1=xy;cx=int((x0+x1)/2);cy=int((y0+y1)/2)
                patch=camera_depth[max(0,cy-2):min(480,cy+3),max(0,cx-2):min(640,cx+3)]
                valid_depth=patch[np.isfinite(patch)&(patch>.02)&(patch<10)]
                surface=float(np.median(valid_depth)) if valid_depth.size else None
                record['depth_surface_m']=surface
                record['depth_consistent']=bool(surface is not None and estimate.get('translation_valid') and abs(surface-estimate['center_camera_m'][2])<.1)
                record['tag_ids']=[t['id'] for t in taglist if 0<=t['id']<12 and all(xy[0]<=x<=xy[2] and xy[1]<=y<=xy[3] for x,y in t['corners'])]
            detections.append(record)
    def safe(o):
        if isinstance(o,dict):return {k:safe(v) for k,v in o.items()}
        if isinstance(o,(list,tuple)):return [safe(v) for v in o]
        if isinstance(o,np.ndarray):return safe(o.tolist())
        if isinstance(o,(float,np.floating)):return float(o) if np.isfinite(o) else None
        if isinstance(o,np.integer):return int(o)
        return o
    print(json.dumps(safe({**identity,'inference_started_monotonic_ns':started,'inference_completed_monotonic_ns':time.monotonic_ns(),'camera_frame':camera+'_left_optical','barcodes':codes,'barcode_boxes':barcode_boxes,'tags':taglist,'detections':detections,'inference_seconds':time.monotonic()-start,'source':'rendered_camera'})),flush=True)
