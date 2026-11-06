"""Bounded read-only ROS subscriptions. No controller, publishing, or inference."""
import argparse,base64,json,time,uuid,os
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from nav_msgs.msg import Odometry
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--seconds',type=float,default=5);args=p.parse_args()
if not 0<args.seconds<=30:raise ValueError('Bounded duration required')
out=Path(args.output);out.mkdir(exist_ok=False);recording_id=uuid.uuid4().hex
rclpy.init();node=Node('r12_readonly_visual_recorder',enable_rosout=False,start_parameter_services=False);counts={};errors=[];topics={'/visual_slam/tracking/odometry':Odometry,'/visual_slam/status':VisualSlamStatus};started=time.monotonic_ns()
try:
 with (out/'raw_messages.jsonl').open('x') as log:
  def receive(topic,message):
   if sum(counts.values())>=256:return
   received=time.monotonic_ns();raw=serialize_message(message)
   if len(raw)>1048576:errors.append('message_size_limit');return
   record=dict(recording_id=recording_id,source_run_id=None,source_clock_epoch=None,source_session_verified=False,topic=topic,type=topics[topic].__name__,received_monotonic_ns=received,source_stamp_ns=message.header.stamp.sec*10**9+message.header.stamp.nanosec,source_frame_id=message.header.frame_id,cdr_base64=base64.b64encode(raw).decode('ascii'))
   log.write(json.dumps(record)+'\n');log.flush();counts[topic]=counts.get(topic,0)+1
  qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
  subs=[node.create_subscription(cls,topic,lambda msg,t=topic:receive(t,msg),qos) for topic,cls in topics.items()]
  while time.monotonic_ns()-started<args.seconds*1e9:rclpy.spin_once(node,timeout_sec=.1)
  endpoints={topic:[dict(node_name=e.node_name,node_namespace=e.node_namespace,topic_type=e.topic_type,gid=list(e.endpoint_gid)) for e in node.get_publishers_info_by_topic(topic)] for topic in topics}
  report=dict(recording_id=recording_id,domain=os.getenv('ROS_DOMAIN_ID'),duration_s=(time.monotonic_ns()-started)/1e9,counts=counts,errors=errors,publishers=endpoints,source_session_verified=False,pose_join_verified=False,execution_authorized=False,scope='Original serialized ROS messages only;local recording identity is not source run identity. No sensor publisher or simulator started.')
  (out/'summary.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
finally:node.destroy_node();rclpy.shutdown()
