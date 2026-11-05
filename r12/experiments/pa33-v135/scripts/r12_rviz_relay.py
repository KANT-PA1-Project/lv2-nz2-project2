"""One-way CPU ROS sensor visualization relay across domains, never commands.

Keep Isaac ROS native-buffer discovery out of RViz's domain. Message headers,
timestamps and data are forwarded unchanged. No destination subscriptions exist.
"""
import argparse,json,signal,time
from pathlib import Path
from collections import Counter
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from rosidl_runtime_py.utilities import get_message

p=argparse.ArgumentParser();p.add_argument('--source-domain',type=int,required=True);p.add_argument('--display-domain',type=int,required=True);p.add_argument('--report',type=Path,required=True);a=p.parse_args()
if a.source_domain==a.display_domain:p.error('visualization domain must be isolated')
source_context=Context();display_context=Context()
rclpy.init(context=source_context,domain_id=a.source_domain);rclpy.init(context=display_context,domain_id=a.display_domain)
source=rclpy.create_node('r12_display_readonly_source',context=source_context)
display=rclpy.create_node('r12_display_oneway_sink',context=display_context)
executor=SingleThreadedExecutor(context=source_context);executor.add_node(source)
sensor=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
reliable=QoSProfile(depth=20,reliability=ReliabilityPolicy.RELIABLE)
latched=QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
topics={
 '/clock':('rosgraph_msgs/msg/Clock',reliable),
 '/tf':('tf2_msgs/msg/TFMessage',reliable),
 '/tf_static':('tf2_msgs/msg/TFMessage',latched),
 '/robot_description':('std_msgs/msg/String',latched),
 '/map':('nav_msgs/msg/OccupancyGrid',latched),
 '/global_costmap/costmap':('nav_msgs/msg/OccupancyGrid',latched),
 '/plan':('nav_msgs/msg/Path',reliable),
 '/r12/joint_states':('sensor_msgs/msg/JointState',sensor),
 '/r12/points':('sensor_msgs/msg/PointCloud2',sensor),
 '/r12/points_rgb':('sensor_msgs/msg/PointCloud2',sensor),
}
for name in ['stereo','r07_camera2','r07_camera3','arm_camera']:topics[f'/r12/{name}/left/image']=('sensor_msgs/msg/Image',sensor)
for name in ['front_right','rear_left','low_front_left','low_rear_right']:topics[f'/r12/lidar/{name}']=('sensor_msgs/msg/LaserScan',sensor)
received=Counter();sent=Counter();last={};pubs={};subs=[];running=True;last_save=0
def callback(topic):
 def receive(message):
  received[topic]+=1
  # Limit display copies by wall time only; preserve original sensor timestamps.
  now=time.monotonic()
  if (topic.endswith('/image') or topic in ('/r12/points','/r12/points_rgb')) and now-last.get(topic,0)<.2:return
  pubs[topic].publish(message);sent[topic]+=1;last[topic]=now
 return receive
for topic,(kind,qos) in topics.items():
 cls=get_message(kind);pubs[topic]=display.create_publisher(cls,topic,qos)
 subs.append(source.create_subscription(cls,topic,callback(topic),qos))
def stop(*_):
 global running
 running=False
signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
def save():
 d={'source_domain':a.source_domain,'display_domain':a.display_domain,'received':dict(received),'forwarded':dict(sent),'display_subscriber_counts':{t:p.get_subscription_count() for t,p in pubs.items()},'source_publishers':0,'display_subscriptions':0,'headers_and_data_unchanged':True,'control_topics_forwarded':False,'monotonic_ns':time.monotonic_ns()}
 tmp=a.report.with_suffix('.tmp');tmp.write_text(json.dumps(d,indent=2)+'\n');tmp.replace(a.report)
try:
 while running:
  executor.spin_once(timeout_sec=.1)
  if time.monotonic()-last_save>2:save();last_save=time.monotonic()
finally:
 save();executor.shutdown();source.destroy_node();display.destroy_node();source_context.shutdown();display_context.shutdown()
