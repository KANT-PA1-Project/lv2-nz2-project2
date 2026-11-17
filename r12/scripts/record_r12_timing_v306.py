"""Bounded read-only ROS subscriptions. No controller, publishing, or inference."""
import argparse,base64,json,time,uuid,os
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--seconds',type=float,default=5);args=p.parse_args()
if not 0<args.seconds<=30:raise ValueError('Bounded duration required')
out=Path(args.output);out.mkdir(exist_ok=False);recording_id=uuid.uuid4().hex
rclpy.init();node=Node('r12_readonly_visual_recorder',enable_rosout=False,start_parameter_services=False);counts={};errors=[];topics={'/visual_slam/tracking/odometry':Odometry,'/visual_slam/status':VisualSlamStatus};started=time.monotonic_ns()
manifest=json.loads((Path(__file__).resolve().parents[3]/'outputs/r12/v299_replay_manifest.json').read_text())
topics.update({row['topic_base']+'/image':Image for row in manifest['images']})
try:
 with (out/'raw_messages.jsonl').open('x') as log:
  def receive(topic,message,info):
   if sum(counts.values())>=512:return
   received=time.monotonic_ns();wall=time.time_ns();raw=serialize_message(message) if topics[topic] is not Image else b''
   if len(raw)>1048576:errors.append('message_size_limit');return
   record=dict(recording_id=recording_id,source_run_id=None,source_clock_epoch=None,source_session_verified=False,topic=topic,type=topics[topic].__name__,received_monotonic_ns=received,callback_wall_ns=wall,dds_source_timestamp_ns=info['source_timestamp'],dds_received_timestamp_ns=info['received_timestamp'],publisher_gid=list(info['publisher_gid']),source_stamp_ns=message.header.stamp.sec*10**9+message.header.stamp.nanosec,source_frame_id=message.header.frame_id,cdr_base64=base64.b64encode(raw).decode('ascii'))
   log.write(json.dumps(record)+'\n');log.flush();counts[topic]=counts.get(topic,0)+1
  qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
  def callback(topic):
   def cb(msg,info):receive(topic,msg,info)
   return cb
  subs=[node.create_subscription(cls,topic,callback(topic),qos) for topic,cls in topics.items()]
  while time.monotonic_ns()-started<args.seconds*1e9:rclpy.spin_once(node,timeout_sec=.1)
  endpoints={topic:[dict(node_name=e.node_name,node_namespace=e.node_namespace,topic_type=e.topic_type,gid=list(e.endpoint_gid)) for e in node.get_publishers_info_by_topic(topic)] for topic in topics}
  report=dict(recording_id=recording_id,domain=os.getenv('ROS_DOMAIN_ID'),duration_s=(time.monotonic_ns()-started)/1e9,counts=counts,errors=errors,publishers=endpoints,source_session_verified=False,pose_join_verified=False,execution_authorized=False,scope='Original odometry/status CDR and image arrival metadata only; additional image observer can affect timing;local recording identity is not source run identity. No sensor publisher or simulator started.')
  (out/'summary.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
finally:node.destroy_node();rclpy.shutdown()
