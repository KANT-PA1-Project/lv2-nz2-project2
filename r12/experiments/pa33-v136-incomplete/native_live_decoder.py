"""Warmed unchanged GPU models on a bounded trusted same-host live capture stream."""
import argparse,hashlib,json,os,socket,sys,time,subprocess
from pathlib import Path
root=Path(__file__).resolve().parent;sys.path.insert(0,str(root/'src'))
from arm_practice.native_rgbd.runtime_settings import RuntimeSettings
settings=RuntimeSettings.from_env();paths,pin=settings.decoder()
settings.verify_models(paths,pin)
sys.path[:0]=[str(root/'src'),str(paths['decoder_deps']),str(paths['decoder_scripts'])]
import numpy as np,cv2,torch
from ultralytics import YOLO
from train_centerpose import load_model,infer
from arm_practice.tag_detection import detect_tags
from arm_practice.native_rgbd.distinct_identity import diagnostic_identity
from arm_practice.native_rgbd.capture_fk import CaptureFK
from arm_practice.native_rgbd.live_shadow_inputs import LiveShadowInputs
from arm_practice.native_rgbd.mixed_camera_schema import MixedCameraSchema
from arm_practice.native_rgbd.trusted_capture_ipc import send_packet,receive_packet,require_peer,peer_credentials
from arm_practice.native_rgbd.live_decode_contract import validate,fresh_at_completion
p=argparse.ArgumentParser();p.add_argument('--socket');p.add_argument('--producer-pid',type=int);p.add_argument('--run-id');p.add_argument('--output');a=p.parse_args();out=Path(a.output)
def safe(x):
 if isinstance(x,dict):return {k:safe(v)for k,v in x.items()}
 if isinstance(x,(tuple,list)):return [safe(v)for v in x]
 if isinstance(x,np.ndarray):return safe(x.tolist())
 if isinstance(x,np.integer):return int(x)
 if isinstance(x,(float,np.floating)):return float(x)if np.isfinite(x)else None
 return x
def write(name,x):(out/name).write_text(json.dumps(safe(x),indent=2))
def free():return int(subprocess.check_output(['nvidia-smi','--query-gpu=memory.free','--format=csv,noheader,nounits'],text=True).splitlines()[0])
sock=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);sock.settimeout(60);sock.connect(a.socket);require_peer(peer_credentials(sock),a.producer_pid,os.getuid())
model=paths['model_root']
schema=MixedCameraSchema.from_file(root/'configs/mixed_camera_schema.json');fk=CaptureFK(root)
fx,fy,cx,cy=schema.config['cameras']['arm_camera_left_optical']['intrinsics'];K=np.array([[fx,0,cx],[0,fy,cy],[0,0,1.]])
cfg=json.loads((model/'inference.json').read_text());cv2.setNumThreads(4);torch.set_num_threads(2);load_start=time.monotonic_ns()
yolo=YOLO(model/'yolo/weights/best.pt');pose=load_model(model/'centerpose/best.pt',device='cuda:0');torch.cuda.synchronize();load_end=time.monotonic_ns()
def decode(rgb,depth):
 t={};t['preprocess_start_ns']=time.monotonic_ns();image=cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR);gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY);t['preprocess_end_ns']=time.monotonic_ns()
 t['tag_decode_start_ns']=time.monotonic_ns();tags=detect_tags(gray,cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)));t['tag_decode_end_ns']=time.monotonic_ns()
 torch.cuda.synchronize();t['YOLO_start_ns']=time.monotonic_ns();results=yolo.predict(image,imgsz=cfg['imgsz'],conf=cfg['detector_confidence'],device='cuda:0',verbose=False)[0];torch.cuda.synchronize();t['YOLO_end_ns']=time.monotonic_ns();records=[];t['objects']=[]
 for box in results.boxes:
  cls=int(box.cls.item());xy=box.xyxy[0].tolist();r=dict(class_id=cls,class_name=results.names[cls],bbox=xy,confidence=float(box.conf.item()))
  if cls<4:
   tt={'pose_start_ns':time.monotonic_ns()};estimate=infer(pose,image,xy,K,device='cuda:0',dimensions_m=(.03,.03,.06),views=cfg['pose_views']);torch.cuda.synchronize();tt['pose_end_ns']=time.monotonic_ns();tt['postprocess_start_ns']=time.monotonic_ns()
   u,v=int((xy[0]+xy[2])/2),int((xy[1]+xy[3])/2);patch=depth[max(0,v-2):min(depth.shape[0],v+3),max(0,u-2):min(depth.shape[1],u+3)];valid=patch[np.isfinite(patch)&(patch>.02)&(patch<10)]
   r.update(pose=estimate,depth_surface_m=float(np.median(valid))if len(valid)else None,tag_ids=[z['id']for z in tags if 0<=z['id']<12 and all(xy[0]<=x<=xy[2]and xy[1]<=y<=xy[3]for x,y in z['corners'])]);tt['postprocess_end_ns']=time.monotonic_ns();t['objects'].append(tt)
  records.append(r)
 return tags,records,t
# Explicit historical image used only to warm model kernels; never emitted as live.
archive=paths['warmup_npz']
with np.load(archive,allow_pickle=False)as z:wrgb=z['arm_camera_left_optical'];wdepth=z['arm_camera_depth']
warm=[]
for i in range(2):
 start=time.monotonic_ns();_,_,stages=decode(wrgb,wdepth);warm.append(dict(index=i,elapsed_ms=(time.monotonic_ns()-start)/1e6,stages=stages,live=False))
del wrgb,wdepth
ready=dict(kind='ready',run_id=a.run_id,warmed_at_ns=time.monotonic_ns(),model_load_ms=(load_end-load_start)/1e6,warmup=warm,warmup_archive_sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),model_sha256=pin,gpu_free_MiB=free(),motion_authorized=False)
write('decoder_warmup.json',ready)
if ready['gpu_free_MiB']<1024:raise RuntimeError('VRAM reserve before live capture')
send_packet(sock,ready);rows=[];last=-1;previous_frame=-2;previous_id=None;streak=0
shadow_inputs=LiveShadowInputs(sock,a.producer_pid,a.run_id,schema)
while True:
 wait_start=time.monotonic_ns();msg,blobs=receive_packet(sock,wire_version=schema.wire_schema,camera_schema=schema,allow_control_finish=True);received=time.monotonic_ns()
 if msg['kind']=='finish':break
 check_start=time.monotonic_ns();b=validate(msg,blobs,run_id=a.run_id,last_step=last,now_ns=check_start);last=b['capture_physics_step'];check_end=time.monotonic_ns()
 arrays={d['name']:np.frombuffer(blob,dtype=d['dtype']).reshape(d['shape'])for d,blob in zip(msg['arrays'],blobs)}
 shadow_context=shadow_inputs.begin(msg,arrays)
 fk_start=time.monotonic_ns();geometry=fk.produce(run_id=a.run_id,frame_index=msg['frame'],capture_ns=b['capture_monotonic_lower_bound_ns'],history=b['encoder_history'],reported_camera_T=b['encoder_history']['camera_fk']['arm_camera_left_optical'],now_ns=fk_start);fk_end=time.monotonic_ns()
 tags,objects,stages=decode(arrays['arm_camera_left_optical'],arrays['arm_camera_depth']);pose_join_start=time.monotonic_ns()
 for o in objects:
  if o.get('pose',{}).get('valid'):
   T=np.eye(4);T[:3,:3]=o['pose']['rotation_matrix'];T[:3,3]=o['pose']['center_camera_m'];T=np.array(geometry['T_base_camera'])@T;o['observed_T_base_object']=T.tolist();o['object_minus_TCP_base_m']=(T[:3,3]-np.array(geometry['T_base_tcp'])[:3,3]).tolist()
 done=time.monotonic_ns();fresh=fresh_at_completion(b['capture_monotonic_lower_bound_ns'],done)
 ident=diagnostic_identity(objects,fresh=fresh)
 streak=(streak+1 if msg['frame']==previous_frame+1 and ident==previous_id else 1)if ident is not None else 0;previous_frame=msg['frame'];previous_id=ident
 row=dict(kind='decoded',frame=msg['frame'],capture_step=last,capture_ns=b['capture_monotonic_lower_bound_ns'],decoder_completed_ns=done,capture_to_diagnostic_ms=(done-b['capture_monotonic_lower_bound_ns'])/1e6,camera_fresh_at_completion=fresh,run_id=a.run_id,live_source_authenticated=True,raw_hashes_verified=len(blobs),tags=tags,detections=objects,geometry=geometry,consecutive_fresh_same_id=streak,diagnostic_three_fresh_ids=streak>=3,pose_error_budget=None,observation_admitted=False,motion_authorized=False,stages=dict(receive_wait_start_ns=wait_start,receive_complete_ns=received,hash_start_ns=check_start,hash_end_ns=check_end,FK_start_ns=fk_start,FK_end_ns=fk_end,pose_join_start_ns=pose_join_start,pose_join_end_ns=done,**stages))
 shadow_start=time.monotonic_ns()
 try:row['live_shadow']=shadow_inputs.finish(shadow_context,tags,arrays['arm_camera_depth'],K,row,streak)
 except (ValueError,KeyError) as error:
  shadow_inputs.inputs.discard_pair(shadow_context['ch'],shadow_context['fh']);row['live_shadow']={'rejected':str(error),'adapter_ready':False,'motion_authorized':False}
 row['shadow_completed_ns']=time.monotonic_ns();row['capture_to_shadow_complete_ms']=(row['shadow_completed_ns']-row['capture_ns'])/1e6;row['shadow_fresh_at_completion']=fresh_at_completion(row['capture_ns'],row['shadow_completed_ns'])
 rows.append(row);write('live_decoder_results.json',rows);send_packet(sock,safe(row))
 if free()<1024:raise RuntimeError('VRAM reserve during live decode')
write('live_decoder_completion.json',dict(process_complete=True,frames=len(rows),fresh_count=sum(x['camera_fresh_at_completion']for x in rows),three_fresh_ids=any(x['diagnostic_three_fresh_ids']for x in rows),mission_success=False,motion_authorized=False));sock.close()
