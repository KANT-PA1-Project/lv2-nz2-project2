"""Bounded saved-image transport probe. Replay identity is not acquisition identity."""
import json,time,uuid,sys,hashlib
from pathlib import Path
import cv2,numpy as np,rclpy
from sensor_msgs.msg import Image,CameraInfo
repo=Path(__file__).resolve().parents[1];sys.path.insert(0,str(repo/'src'));out=repo.parents[1]/'outputs/r12'
from arm_practice.capture_bundle_identity import CaptureSessionLedger
from arm_practice.image_publication_evidence import message_evidence
rclpy.init();node=rclpy.create_node('r12_v325_probe',enable_rosout=False,start_parameter_services=False);events=[];sent=[];pending={};report={}
def spin(dt):
 end=time.monotonic()+dt
 while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
def cb(kind):
 def receive(msg,info):
  key=(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,msg.header.frame_id)
  part=pending.setdefault(key,{})
  part[kind]=msg;part[kind+'_gid']=list(info['publisher_gid']['data'])
  if 'image' in part and 'info' in part:
   events.append(dict(evidence=message_evidence(part['image'],part['info']),image_gid=part['image_gid'],info_gid=part['info_gid'],received_ns=time.monotonic_ns()));del pending[key]
 return receive
try:
 spin(1);assert all(n=='r12_v325_probe' for n,ns in node.get_node_names_and_namespaces()),'occupied domain'
 manifest=json.loads((out/'v299_replay_manifest.json').read_text());pairs=[];subs=[];rgb={}
 for row in manifest['images']:
  path=Path(row['image']);assert hashlib.sha256(path.read_bytes()).hexdigest()==row['sha256']
  pixels=cv2.cvtColor(cv2.imread(str(path)),cv2.COLOR_BGR2RGB);rgb[row['camera_frame']]=pixels
  im=Image();im.header.frame_id=row['camera_frame'];im.width=640;im.height=480;im.encoding='rgb8';im.step=1920;im.data=pixels.tobytes()
  ci=CameraInfo();ci.header.frame_id=row['camera_frame'];ci.width=640;ci.height=480;ci.distortion_model='plumb_bob';ci.d=[0.]*5;ci.k=row['K'];ci.r=np.eye(3).flatten().tolist();ci.p=row['P']
  pubs=[node.create_publisher(cls,row['topic_base']+'/'+suffix,10) for cls,suffix in [(Image,'image'),(CameraInfo,'camera_info')]]
  subs.extend(node.create_subscription(cls,row['topic_base']+'/'+suffix,cb(kind),10) for cls,suffix,kind in [(Image,'image','image'),(CameraInfo,'camera_info','info')])
  pairs.append((pubs,im,ci))
 deadline=time.monotonic()+5
 while not all(p.get_subscription_count()==1 for pubs,_,_ in pairs for p in pubs) and time.monotonic()<deadline:spin(.05)
 assert all(p.get_subscription_count()==1 for pubs,_,_ in pairs for p in pubs)
 run=uuid.uuid4().hex;ledger=CaptureSessionLedger(run,'v325');valid=0
 for frame in range(10):
  ns=1000000000+frame*50000000;capture=time.monotonic_ns()
  bundle=dict(run_id=np.array(run),clock_domain=np.array('synthetic_packet_20hz'),frame_index=np.int64(frame),stamp_ns=np.int64(ns),capture_monotonic_ns=np.int64(capture),rgb=np.stack([rgb['r07_camera1_'+s+'_optical'] for s in ('left','right')]))
  bundle.update({k:v for k,v in rgb.items() if not k.startswith('r07_camera1')})
  verdict=ledger.inspect(bundle,dict(capture_run_id=run,stamp_ns=ns,frame_capture_monotonic_ns=capture),generation='v325');assert verdict['sequence_eligible'];valid+=1
  for pubs,im,ci in pairs:
   im.header.stamp.sec=ci.header.stamp.sec=ns//10**9;im.header.stamp.nanosec=ci.header.stamp.nanosec=ns%10**9
   evidence=message_evidence(im,ci);assert evidence['image_sha256']==verdict['image_sha256'][im.header.frame_id]
   sent.append(evidence);pubs[1].publish(ci);pubs[0].publish(im)
  deadline=time.monotonic()+2
  while len(events)<len(sent) and time.monotonic()<deadline:spin(.01)
  assert len(events)==len(sent)
 expected={(e['stamp_ns'],e['frame_id']):e for e in sent}
 mismatches=[i for i,e in enumerate(events) if e['evidence']!=expected[(e['evidence']['stamp_ns'],e['evidence']['frame_id'])]]
 # Mutations must be visible in evidence; frame/stamp inconsistency must reject.
 _,im,ci=pairs[0];original=message_evidence(im,ci);ci.k[0]+=1;assert message_evidence(im,ci)['calibration_sha256']!=original['calibration_sha256'];ci.k[0]-=1
 im.data[0]^=1;assert message_evidence(im,ci)['image_sha256']!=original['image_sha256'];im.data[0]^=1
 ci.header.frame_id='wrong'
 try:message_evidence(im,ci)
 except ValueError:report['identity_mismatch_rejected']=True
 assert report.get('identity_mismatch_rejected') and not mismatches
 report.update(diagnostic_pass=True,bundles=valid,sent_image_info_pairs=len(sent),received_pairs=len(events),digest_mismatches=mismatches,mutation_checks_passed=3)
finally:
 node.destroy_node();rclpy.shutdown();report.update(owned_node_destroyed=True,source_session_verified=False,capture_freshness_verified=False,pose_join_verified=False,execution_authorized=False,runtime_integrated=False,scope='10 synthetic replay bundles from saved images; same-process real ROS publisher/subscriber. Hashes describe outgoing bytes and independent observer delivery, not cuVSLAM consumption or original acquisition. Monotonic value is replay preparation time.')
 (out/'v325_publication_events.json').write_text(json.dumps(dict(sent=sent,received=events),indent=2)+'\n');(out/'v325_report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
