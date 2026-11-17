"""Bounded read-only ROS subscriptions. No controller, publishing, or inference."""
import argparse,base64,json,time,uuid,os,sys
from rosgraph_msgs.msg import Clock
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from sensor_msgs.msg import Image,CameraInfo
from nav_msgs.msg import Odometry
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--seconds',type=float,default=5);args=p.parse_args()
if not 0<args.seconds<=30:raise ValueError('Bounded duration required')
out=Path(args.output);out.mkdir(exist_ok=False);recording_id=uuid.uuid4().hex
rclpy.init();node=Node('r12_readonly_visual_recorder',enable_rosout=False,start_parameter_services=False);counts={};live_endpoints={};errors=[];topics={'/visual_slam/tracking/odometry':Odometry,'/visual_slam/status':VisualSlamStatus};started=time.monotonic_ns()
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from arm_practice.raw_visual_callback import RawVisualCallback
from arm_practice.image_publication_evidence import message_evidence
image_pending={};image_evidence=[]
def record_pair(msg,kind):
 key=(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,msg.header.frame_id)
 if len(image_evidence)>=1200:return
 if key not in image_pending and len(image_pending)>=24:image_pending.pop(next(iter(image_pending)))
 pair=image_pending.setdefault(key,{})
 pair[kind]=msg
 if 'image' in pair and 'info' in pair:
  image_evidence.append(message_evidence(pair['image'],pair['info']));del image_pending[key]
adapter=None;expected_stamp=None;clock_fault=False;clock_events=[];generation=uuid.uuid4().hex
manifest=json.loads((Path(__file__).resolve().parents[3]/'outputs/r12/v299_replay_manifest.json').read_text())
topics.update({row['topic_base']+'/image':Image for row in manifest['images']})
try:
 with (out/'raw_messages.jsonl').open('x') as log:
  def receive(topic,message,info):
   if sum(counts.values())>=2048:return
   if topics[topic] is Image:record_pair(message,'image')
   received=time.monotonic_ns();wall=time.time_ns();raw=serialize_message(message) if topics[topic] is not Image else b''
   if len(raw)>1048576:errors.append('message_size_limit');return
   record=dict(recording_id=recording_id,source_run_id=None,source_clock_epoch=None,source_session_verified=False,topic=topic,type=topics[topic].__name__,received_monotonic_ns=received,callback_wall_ns=wall,dds_source_timestamp_ns=info['source_timestamp'],dds_received_timestamp_ns=info['received_timestamp'],publisher_gid=({'implementation_identifier':info['publisher_gid']['implementation_identifier'],'data':list(info['publisher_gid']['data'])} if info['publisher_gid'] else None),source_stamp_ns=message.header.stamp.sec*10**9+message.header.stamp.nanosec,source_frame_id=message.header.frame_id,cdr_base64=base64.b64encode(raw).decode('ascii'))
   if topics[topic] is not Image:
    record['decision_now_ns']=time.monotonic_ns();record['expected_stamp_ns']=expected_stamp
    if adapter is None or expected_stamp is None or clock_fault:record['admission']={'state':'not_ready_or_clock_fault','structurally_eligible':False}
    else:
     try:
      answer=adapter.receive('odometry' if topics[topic] is Odometry else 'status',message,info,callback_generation=generation,received_ns=received,now_ns=record['decision_now_ns'],expected_stamp_ns=expected_stamp)
      record['admission']={k:v for k,v in answer.items() if k not in ('received_snapshot','raw_odometry')}
     except (ValueError,KeyError,TypeError) as exc:record['admission']={'error':str(exc),'structurally_eligible':False}
   log.write(json.dumps(record)+'\n');log.flush();counts[topic]=counts.get(topic,0)+1
  def snapshot_live_endpoints():
   global adapter
   for topic in topics:
    if topic in live_endpoints:continue
    endpoints=node.get_publishers_info_by_topic(topic)
    if endpoints:live_endpoints[topic]=dict(observed_monotonic_ns=time.monotonic_ns(),endpoints=[dict(node_name=e.node_name,gid=list(e.endpoint_gid)) for e in endpoints])
   if adapter is None and all(t in live_endpoints and len(live_endpoints[t]['endpoints'])==1 for t in ('/visual_slam/tracking/odometry','/visual_slam/status')):
    pins={k:dict(implementation_identifier='rmw_fastrtps_cpp',data=live_endpoints[t]['endpoints'][0]['gid']) for k,t in [('odometry','/visual_slam/tracking/odometry'),('status','/visual_slam/status')]}
    adapter=RawVisualCallback(generation,recording_id,'local-epoch',expected_gids=pins)
    (out/'admission_ready.json').write_text(json.dumps(dict(generation=generation,recording_id=recording_id,pins=pins)))
  def receive_clock(msg):
   global expected_stamp,clock_fault
   ns=msg.clock.sec*10**9+msg.clock.nanosec
   if expected_stamp is not None and ns<expected_stamp:clock_fault=True
   expected_stamp=ns
   if len(clock_events)<2048:clock_events.append(dict(stamp_ns=ns,received_ns=time.monotonic_ns()))
  def info_cb(msg):record_pair(msg,'info')
  info_subs=[node.create_subscription(CameraInfo,row['topic_base']+'/camera_info',info_cb,10) for row in manifest['images']]
  clock_sub=node.create_subscription(Clock,'/clock',receive_clock,10)
  endpoint_timer=node.create_timer(.5,snapshot_live_endpoints)
  qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
  image_qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
  def callback(topic):
   def cb(msg,info):receive(topic,msg,info)
   return cb
  subs=[node.create_subscription(cls,topic,callback(topic),image_qos if cls is Image else qos) for topic,cls in topics.items()]
  while time.monotonic_ns()-started<args.seconds*1e9:rclpy.spin_once(node,timeout_sec=.1)
  endpoints={topic:[dict(node_name=e.node_name,node_namespace=e.node_namespace,topic_type=e.topic_type,gid=list(e.endpoint_gid)) for e in node.get_publishers_info_by_topic(topic)] for topic in topics}
  report=dict(recording_id=recording_id,domain=os.getenv('ROS_DOMAIN_ID'),duration_s=(time.monotonic_ns()-started)/1e9,counts=counts,errors=errors,publishers=endpoints,live_publishers=live_endpoints,source_session_verified=False,pose_join_verified=False,execution_authorized=False,scope='Original odometry/status CDR and image arrival metadata only; additional image observer can affect timing;local recording identity is not source run identity. No sensor publisher or simulator started.')
  (out/'image_evidence.json').write_text(json.dumps(dict(pairs=image_evidence,pending=len(image_pending))))
  (out/'clock_events.json').write_text(json.dumps(dict(events=clock_events,fault=clock_fault)))
  (out/'summary.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
finally:node.destroy_node();rclpy.shutdown()
