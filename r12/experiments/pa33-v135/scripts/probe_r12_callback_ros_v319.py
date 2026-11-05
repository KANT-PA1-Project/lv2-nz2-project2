"""Isolated ROS transport replay of saved CDR, no simulator/controller/cuVSLAM inference."""
import base64,json,sys,time,uuid
from pathlib import Path
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.serialization import deserialize_message,serialize_message
from nav_msgs.msg import Odometry
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
repo=Path(__file__).resolve().parents[1];sys.path.insert(0,str(repo/'src'));out=repo.parents[1]/'outputs/r12'
from arm_practice.raw_visual_callback import RawVisualCallback
rclpy.init();observer=rclpy.create_node('r12_v319_observer',enable_rosout=False,start_parameter_services=False);source=None;executor=SingleThreadedExecutor();executor.add_node(observer);rows=[];report={};adapter=None;expected=None;phase='original';generation=uuid.uuid4().hex
try:
 for _ in range(10):executor.spin_once(timeout_sec=.1)
 others=[n for n,ns in observer.get_node_names_and_namespaces() if n!='r12_v319_observer']
 if others:raise RuntimeError('Domain occupied:'+str(others))
 topics={'odometry':('/visual_slam/tracking/odometry',Odometry),'status':('/visual_slam/status',VisualSlamStatus)}
 def callback(kind):
  def cb(msg,info):
   receipt=time.monotonic_ns();row=dict(kind=kind,phase=phase,receipt_ns=receipt,expected_stamp_ns=expected,publisher_gid=dict(implementation_identifier=info['publisher_gid']['implementation_identifier'],data=list(info['publisher_gid']['data'])),cdr_base64=base64.b64encode(serialize_message(msg)).decode())
   try:
    answer=adapter.receive(kind,msg,info,callback_generation=generation,received_ns=receipt,now_ns=time.monotonic_ns(),expected_stamp_ns=expected)
    row['decision']={k:v for k,v in answer.items() if k not in ('raw_odometry','received_snapshot')}
   except (ValueError,KeyError,TypeError) as e:row['error']=str(e)
   rows.append(row)
  return cb
 subs=[observer.create_subscription(cls,topic,callback(kind),10) for kind,(topic,cls) in topics.items()]
 def create_source():
  node=rclpy.create_node('r12_v319_recorded_source',enable_rosout=False,start_parameter_services=False)
  return node,{kind:node.create_publisher(cls,topic,10) for kind,(topic,cls) in topics.items()}
 source,pubs=create_source()
 def discover():
  deadline=time.monotonic()+5
  while time.monotonic()<deadline:
   executor.spin_once(timeout_sec=.05)
   endpoints={kind:observer.get_publishers_info_by_topic(topic) for kind,(topic,cls) in topics.items()}
   if all(len(x)==1 for x in endpoints.values()) and all(p.get_subscription_count()==1 for p in pubs.values()):
    return {k:dict(implementation_identifier='rmw_fastrtps_cpp',data=list(v[0].endpoint_gid)) for k,v in endpoints.items()}
  raise RuntimeError('Endpoint discovery timeout')
 pins=discover();adapter=RawVisualCallback(generation,'local-v319','epoch1',expected_gids=pins)
 recorded={}
 for line in (out/'v314_raw/raw_messages.jsonl').read_text().splitlines():
  e=json.loads(line)
  if e['type']=='Image':continue
  recorded.setdefault(e['source_stamp_ns'],{})['odometry' if e['type']=='Odometry' else 'status']=e
 samples=[(stamp,pair) for stamp,pair in sorted(recorded.items()) if len(pair)==2][:20]
 for expected,pair in samples:
  before=len(rows)
  for kind,(topic,cls) in topics.items():pubs[kind].publish(deserialize_message(base64.b64decode(pair[kind]['cdr_base64']),cls))
  deadline=time.monotonic()+.5
  while len(rows)<before+2 and time.monotonic()<deadline:executor.spin_once(timeout_sec=.01)
  if len(rows)<before+2:raise RuntimeError('Missing replay callback')
 phase='restarted_endpoint';source.destroy_node();source=None
 # Allow discovery to remove the old endpoints before constructing a new source.
 for _ in range(10):executor.spin_once(timeout_sec=.05)
 source,pubs=create_source();new_pins=discover();assert all(pins[k]!=new_pins[k] for k in pins)
 expected,pair=samples[-1];before=len(rows)
 for kind,(topic,cls) in topics.items():pubs[kind].publish(deserialize_message(base64.b64decode(pair[kind]['cdr_base64']),cls))
 deadline=time.monotonic()+1
 while len(rows)<before+2 and time.monotonic()<deadline:executor.spin_once(timeout_sec=.01)
 assert len(rows)==before+2
 report.update(original_pins=pins,restarted_pins=new_pins,original_pairs=sum(r.get('decision',{}).get('structurally_eligible',False) for r in rows if r['phase']=='original'),restart_errors=[r.get('error') for r in rows if r['phase']=='restarted_endpoint'],adapter_faulted=adapter.faulted)
 assert report['original_pairs']==20 and report['adapter_faulted']
 assert report['restart_errors']==['publisher_endpoint_changed','callback_faulted_requires_new_instance']
finally:
 if source is not None:source.destroy_node()
 executor.remove_node(observer);observer.destroy_node();executor.shutdown();rclpy.shutdown()
 (out/'v319_callback_events.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
 report.update(events=len(rows),owned_nodes_destroyed=True,scope='Saved CDR replay through real ROS endpoints in isolated domain219. Expected stamp assigned by harness;not live capture or cuVSLAM inference. Source session authentication and motion acceptance remain false.',source_session_verified=False,execution_authorized=False)
 (out/'v319_ros_callback_report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
