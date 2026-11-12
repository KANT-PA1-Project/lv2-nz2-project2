"""Capture one D435 RGB image, its intrinsics and optical TF at exposure time (ROS 2)."""
import argparse,json,time
from pathlib import Path
import cv2,numpy as np,rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo,Image
from tf2_ros import Buffer,TransformListener,TransformException
from scipy.spatial.transform import Rotation

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('prefix',type=Path,nargs='?',default=Path('slot_capture'))
    parser.add_argument('--track',type=Path,help='Continuously publish calibrated observations using this offsets JSON')
    parser.add_argument('--image',default='/wrist_d435/color/image_raw')
    parser.add_argument('--info',default='/wrist_d435/color/camera_info')
    parser.add_argument('--base',default='base_link')
    parser.add_argument('--sim-time',action='store_true')
    args=parser.parse_args()
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true'] if args.sim_time else [])
    node=Node('capture_slot_calibration')
    buf=Buffer(); listener=TransformListener(buf,node); images=[];infos=[]
    def remember(store,msg):
        store.append(msg)
        del store[:-15]
    sub1=node.create_subscription(Image,args.image,lambda m:remember(images,m),qos_profile_sensor_data)
    sub2=node.create_subscription(CameraInfo,args.info,lambda m:remember(infos,m),qos_profile_sensor_data)
    try:
        if args.track:
            from locate import SlotTracker
            from std_msgs.msg import String
            tracker=SlotTracker(json.loads(args.track.read_text()))
            publisher=node.create_publisher(String,'/puck_slots/centers',10)
        last_stamp=None
        until=time.monotonic()+20
        while rclpy.ok() and (args.track or time.monotonic()<until):
            rclpy.spin_once(node,timeout_sec=.1)
            if not images or not infos:continue
            im=images[-1]
            matches=[c for c in infos if c.header.frame_id==im.header.frame_id and (c.width,c.height)==(im.width,im.height)]
            if not matches:continue
            info=matches[-1];stamp=Time.from_msg(im.header.stamp)
            if last_stamp==stamp.nanoseconds:continue
            try:t=buf.lookup_transform(args.base,im.header.frame_id,stamp).transform
            except TransformException:continue
            trans=t.translation;q=t.rotation;mat=np.eye(4);mat[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();mat[:3,3]=[trans.x,trans.y,trans.z]
            if info.distortion_model not in ('','plumb_bob','rational_polynomial'):raise ValueError(info.distortion_model)
            if args.track:
                frame=CvBridge().imgmsg_to_cv2(im,'bgr8')
                result=tracker.update(frame,np.asarray(info.k).reshape(3,3),np.asarray(info.d),mat,stamp.nanoseconds*1e-9)
                message=String();message.data=json.dumps(dict(frame=args.base,observations=result))
                publisher.publish(message);last_stamp=stamp.nanoseconds
                continue
            args.prefix.parent.mkdir(parents=True,exist_ok=True)
            if not cv2.imwrite(str(args.prefix.with_suffix('.png')),CvBridge().imgmsg_to_cv2(im,'bgr8')):raise OSError('Image write failed')
            args.prefix.with_suffix('.json').write_text(json.dumps(dict(K=list(info.k),distortion=list(info.d),base_from_optical=mat.tolist(),stamp=stamp.nanoseconds*1e-9),indent=2))
            break
        else:raise SystemExit('Timed out waiting for RGB, matching camera info and base_link <- optical TF at image time.')
    finally:
        node.destroy_node();rclpy.shutdown()
