"""Static stereo replay only: images, CameraInfo, rig TF and simulation clock."""
import json,time,uuid,hashlib
from pathlib import Path
import numpy as np,cv2,rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image,CameraInfo
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from scipy.spatial.transform import Rotation
repo=Path(__file__).resolve().parents[1];out=repo.parents[1]/'outputs/r12';manifest=json.loads((out/'v299_replay_manifest.json').read_text());rclpy.init();node=Node('r12_static_replay',enable_rosout=False);clock=node.create_publisher(Clock,'/clock',10);tf=StaticTransformBroadcaster(node);pairs=[];trans=[]
run=uuid.uuid4().hex;epoch=uuid.uuid4().hex;rows=[]
try:
 for row in manifest['images']:
  path=Path(row['image']);assert hashlib.sha256(path.read_bytes()).hexdigest()==row['sha256'];rgb=cv2.cvtColor(cv2.imread(str(path)),cv2.COLOR_BGR2RGB);assert rgb.shape==(480,640,3)
  image=Image();image.header.frame_id=row['camera_frame'];image.height=480;image.width=640;image.encoding='rgb8';image.step=1920;image.data=rgb.tobytes()
  ci=CameraInfo();ci.header.frame_id=row['camera_frame'];ci.width=640;ci.height=480;ci.distortion_model='plumb_bob';ci.d=[0.]*5;ci.k=row['K'];ci.r=np.eye(3).flatten().tolist();ci.p=row['P']
  pairs.append((node.create_publisher(Image,row['topic_base']+'/image',10),node.create_publisher(CameraInfo,row['topic_base']+'/camera_info',10),image,ci))
  T=np.array(row['T_base_optical']);m=TransformStamped();m.header.frame_id='base_link';m.child_frame_id=row['camera_frame'];m.transform.translation.x,m.transform.translation.y,m.transform.translation.z=T[:3,3].tolist();q=Rotation.from_matrix(T[:3,:3]).as_quat();m.transform.rotation.x,m.transform.rotation.y,m.transform.rotation.z,m.transform.rotation.w=q.tolist();trans.append(m)
 tf.sendTransform(trans);deadline=time.monotonic()+10
 while time.monotonic()<deadline and not all(p.get_subscription_count() for p,_,_,_ in pairs):rclpy.spin_once(node,timeout_sec=.1)
 counts=[p.get_subscription_count() for p,_,_,_ in pairs]
 if not all(counts):raise RuntimeError('Missing image subscribers:'+str(counts))
 start=time.monotonic_ns()
 for i in range(50):
  delay=(start+i*200000000-time.monotonic_ns())/1e9
  if delay>0:time.sleep(delay)
  ns=1000000000+i*200000000;c=Clock();c.clock.sec=ns//10**9;c.clock.nanosec=ns%10**9;clock.publish(c);capture=time.monotonic_ns()
  for pub,info,image,ci in pairs:ci.header.stamp=c.clock;image.header.stamp=c.clock;info.publish(ci);pub.publish(image)
  rows.append(dict(run_id=run,clock_epoch=epoch,frame_index=i,stamp_ns=ns,replay_publish_monotonic_ns=capture,replay_publish_complete_monotonic_ns=time.monotonic_ns()));rclpy.spin_once(node,timeout_sec=0)
 for _ in range(20):rclpy.spin_once(node,timeout_sec=.1)
finally:
 (out/'v304_replay.json').write_text(json.dumps(dict(rows=rows,run_id=run,clock_epoch=epoch,scope='Static saved image replay;publish time not real capture;no world pose/control data',completed=len(rows)==50),indent=2)+'\n');node.destroy_node();rclpy.shutdown()
