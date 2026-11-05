"""CPU inference on rendered RGB/depth, using the user's final frozen models."""
import sys,json,hashlib,time,os
from pathlib import Path
import numpy as np,cv2,torch
root=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(root/'src'),str(root/'work/perception_deps'),'/home/seongmin/ros2_ws/src/turtlebot-swerve-cuvslam/perception/scripts']
from arm_practice.tag_detection import detect_tags
from arm_practice.perception_transport import frame_identity, require_response
from arm_practice.isaac_pixel_geometry import camera_matrix,metadata as pixel_geometry_metadata,require_archive_geometry
import zxingcpp
from ultralytics import YOLO
from train_centerpose import load_model,infer
run=Path('/home/seongmin/ros2_ws/src/turtlebot-swerve-cuvslam/runs/perception/train-v11-domain')
cfg=json.loads((run/'inference.json').read_text());torch.set_num_threads(2)
backend=os.getenv('R12_PERCEPTION_BACKEND','torch_cpu')
if backend=='torch_cpu':
    yolo=YOLO(run/'yolo/weights/best.pt');pose=load_model(run/'centerpose/best.pt',device='cpu')
elif backend in ('tensorrt_fp16','tensorrt_int8'):
    from arm_practice.trt_perception import DetectorEngine,PoseEngine
    engine_dir=Path(os.environ['R12_TRT_MODEL_DIR']);metadata=json.loads((engine_dir/'manifest.json').read_text())
    for source,digest in metadata['sources'].items():
        if hashlib.sha256(Path(source).read_bytes()).hexdigest()!=digest:raise RuntimeError('TensorRT source hash mismatch')
    if metadata['pose_views']!=cfg['pose_views'] or metadata['imgsz']!=cfg['imgsz']:raise RuntimeError('TensorRT inference contract mismatch')
    precision=backend.removeprefix('tensorrt_')
    yolo=DetectorEngine(engine_dir/('yolo_'+precision+'.engine'),metadata['names'])
    pose=PoseEngine(engine_dir/('centerpose_'+precision+'.engine'),metadata['pose_metric_depth_head'])
else:raise ValueError('Unknown perception backend '+backend)

tags=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11))
K=camera_matrix()
# Initialize lazy inference paths before accepting a timestamped sensor frame.
warmup_started=time.monotonic_ns()
warmup_image=np.zeros((480,640,3),dtype=np.uint8)
yolo.predict(warmup_image,imgsz=cfg['imgsz'],conf=cfg['detector_confidence'],device='cpu',verbose=False)
infer(pose,warmup_image,[160,120,320,360],K,device='cpu',dimensions_m=(.03,.03,.06),views=cfg['pose_views'])
warmup_ns=time.monotonic_ns()-warmup_started
print(json.dumps({'ready':True,'backend':backend,'pixel_geometry':pixel_geometry_metadata(),'warmup_ns':warmup_ns,'sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [run/'yolo/weights/best.pt',run/'centerpose/best.pt',run/'inference.json']}}),flush=True)
for line in sys.stdin:
    req=json.loads(line);request_received_ns=time.monotonic_ns()
    with np.load(req['frame'],allow_pickle=False) as archive:data={k:archive[k] for k in archive.files}
    require_archive_geometry(data)
    identity=frame_identity(data,req['camera']);started=time.monotonic_ns()
    require_response(req,dict(identity,inference_started_monotonic_ns=started,inference_completed_monotonic_ns=started),req['max_age_ns'])
    rgb=data['rgb'][0];image=cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR);start=time.monotonic()
    stage_times={'frame_loaded':started,'barcode_start':time.monotonic_ns()}
    decoded=[r for r in zxingcpp.read_barcodes(image) if r.text in ['P2-01','P2-02','P2-03','P2-04']]
    stage_times['barcode_done']=time.monotonic_ns()
    codes=[r.text for r in decoded]
    barcode_boxes=[]
    for r in decoded:
        pts=[r.position.top_left,r.position.top_right,r.position.bottom_left,r.position.bottom_right]
        barcode_boxes.append([min(p.x for p in pts),min(p.y for p in pts),max(p.x for p in pts),max(p.y for p in pts)])
    camera=req.get('camera','r07_camera1');camera_rgb=rgb if camera=='r07_camera1' else data[camera+'_left_optical'];image=cv2.cvtColor(camera_rgb,cv2.COLOR_RGB2BGR);camera_depth=data['depth'] if camera=='r07_camera1' else data[camera+'_depth']
    stage_times['tags_start']=time.monotonic_ns()
    # Barcode/signal processing remains per frame; tag association needs detections.
    taglist=detect_tags(image,tags) if req.get('detect',False) else []
    stage_times['tags_done']=time.monotonic_ns()
    detections=[]
    if req.get('detect',False):
        stage_times['detector_start']=time.monotonic_ns()
        result=yolo.predict(image,imgsz=cfg['imgsz'],conf=cfg['detector_confidence'],device='cpu',verbose=False)[0]
        stage_times['detector_done']=time.monotonic_ns()
        pose_duration_ns=0
        for box in result.boxes:
            cls=int(box.cls.item());xy=box.xyxy[0].tolist();record={'class_id':cls,'class_name':result.names[cls],'bbox':xy,'confidence':float(box.conf.item())}
            if cls<4:
                pose_start_ns=time.monotonic_ns()
                estimate=infer(pose,image,xy,K,device='cpu',dimensions_m=(.03,.03,.06),views=cfg['pose_views'])
                pose_duration_ns+=time.monotonic_ns()-pose_start_ns
                record['pose']=estimate
                x0,y0,x1,y1=xy;cx=int((x0+x1)/2);cy=int((y0+y1)/2)
                patch=camera_depth[max(0,cy-2):min(480,cy+3),max(0,cx-2):min(640,cx+3)]
                valid_depth=patch[np.isfinite(patch)&(patch>.02)&(patch<10)]
                surface=float(np.median(valid_depth)) if valid_depth.size else None
                record['depth_surface_m']=surface
                record['depth_consistent']=bool(surface is not None and estimate.get('translation_valid') and abs(surface-estimate['center_camera_m'][2])<.1)
                record['tag_ids']=[t['id'] for t in taglist if 0<=t['id']<12 and all(xy[0]<=x<=xy[2] and xy[1]<=y<=xy[3] for x,y in t['corners'])]
            detections.append(record)
        stage_times['pose_calls_total_ns']=pose_duration_ns
        stage_times['detections_done']=time.monotonic_ns()
    def safe(o):
        if isinstance(o,dict):return {k:safe(v) for k,v in o.items()}
        if isinstance(o,(list,tuple)):return [safe(v) for v in o]
        if isinstance(o,np.ndarray):return safe(o.tolist())
        if isinstance(o,(float,np.floating)):return float(o) if np.isfinite(o) else None
        if isinstance(o,np.integer):return int(o)
        return o
    print(json.dumps(safe({**identity,'inference_started_monotonic_ns':started,'inference_completed_monotonic_ns':time.monotonic_ns(),'request_received_monotonic_ns':request_received_ns,'stage_times_ns':stage_times,'detection_performed':bool(req.get('detect',False)),'camera_frame':camera+'_left_optical','barcodes':codes,'barcode_boxes':barcode_boxes,'tags':taglist,'detections':detections,'inference_seconds':time.monotonic()-start,'source':'rendered_camera','backend':backend,'pixel_geometry':pixel_geometry_metadata()})),flush=True)
