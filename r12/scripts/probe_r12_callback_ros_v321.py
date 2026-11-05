"""Separate-process saved-CDR replay; no controller, simulator or inference."""
import base64,json,sys,time,uuid,multiprocessing as mp
from pathlib import Path
import rclpy
from rclpy.serialization import deserialize_message,serialize_message
from nav_msgs.msg import Odometry
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
repo=Path(__file__).resolve().parents[1];sys.path.insert(0,str(repo/'src'));out=repo.parents[1]/'outputs/r12'
from arm_practice.raw_visual_callback import RawVisualCallback
TOPICS={'odometry':('/visual_slam/tracking/odometry',Odometry),'status':('/visual_slam/status',VisualSlamStatus)}
def source(pipe):
 rclpy.init();node=rclpy.create_node('r12_v321_saved_source',enable_rosout=False,start_parameter_services=False)
 pubs={k:node.create_publisher(cls,t,10) for k,(t,cls) in TOPICS.items()}
 try:
  pipe.send('ready')
  while True:
   if pipe.poll(.01):
    cmd=pipe.recv()
    if cmd is None:break
    for k,(_,cls) in TOPICS.items():pubs[k].publish(deserialize_message(base64.b64decode(cmd[k]['cdr_base64']),cls))
   rclpy.spin_once(node,timeout_sec=0)
 finally:node.destroy_node();rclpy.shutdown();pipe.close()
def main():
 rclpy.init();node=rclpy.create_node('r12_v321_observer',enable_rosout=False,start_parameter_services=False)
 proc=None;pipe=None;rows=[];report={};adapter=None;expected=None;phase='initial';generation=uuid.uuid4().hex
 def spin(seconds):
  end=time.monotonic()+seconds
  while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
 def cb(kind):
  def receive(msg,info):
   receipt=time.monotonic_ns();row=dict(phase=phase,kind=kind,receipt_ns=receipt,expected_stamp_ns=expected,generation=generation,publisher_gid=dict(implementation_identifier=info['publisher_gid']['implementation_identifier'],data=list(info['publisher_gid']['data'])),cdr_base64=base64.b64encode(serialize_message(msg)).decode())
   try:
    a=adapter.receive(kind,msg,info,callback_generation=generation,received_ns=receipt,now_ns=time.monotonic_ns(),expected_stamp_ns=expected)
    row['decision']={k:v for k,v in a.items() if k not in ('raw_odometry','received_snapshot')}
   except ValueError as e:row['error']=str(e)
   rows.append(row)
  return receive
 def start():
  parent,child=mp.get_context('spawn').Pipe();p=mp.get_context('spawn').Process(target=source,args=(child,));p.start();child.close()
  if not parent.poll(10):raise RuntimeError('source startup timeout')
  assert parent.recv()=='ready'
  return p,parent
 def stop():
  if proc is not None:
   if proc.is_alive():pipe.send(None)
   proc.join(3)
   if proc.is_alive():proc.terminate();proc.join(3)
   if proc.is_alive():proc.kill();proc.join()
   pipe.close()
 def discover():
  end=time.monotonic()+5
  while time.monotonic()<end:
   spin(.05);ep={k:node.get_publishers_info_by_topic(t) for k,(t,_) in TOPICS.items()}
   if all(len(v)==1 for v in ep.values()):
    spin(.3)
    return {k:dict(implementation_identifier='rmw_fastrtps_cpp',data=list(v[0].endpoint_gid)) for k,v in ep.items()}
  raise RuntimeError('discovery timeout')
 def send(pair):
  before=len(rows);pipe.send(pair);end=time.monotonic()+1
  while len(rows)<before+2 and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
  assert len(rows)==before+2,'missing callback'
 try:
  spin(1);assert all(n=='r12_v321_observer' for n,ns in node.get_node_names_and_namespaces()),'domain occupied'
  subs=[node.create_subscription(cls,t,cb(k),10) for k,(t,cls) in TOPICS.items()]
  recorded={}
  for line in (out/'v320_raw/raw_messages.jsonl').read_text().splitlines():
   e=json.loads(line)
   if e['type']!='Image':recorded.setdefault(e['source_stamp_ns'],{})['odometry' if e['type']=='Odometry' else 'status']=e
  samples=[(ns,p) for ns,p in sorted(recorded.items()) if len(p)==2][:20]
  proc,pipe=start();pid1=proc.pid;pins=discover();adapter=RawVisualCallback(generation,'v321-local','epoch1',expected_gids=pins)
  for expected,pair in samples:send(pair)
  stop();exit1=proc.exitcode;proc=None;spin(.5)
  phase='restart_old_adapter';proc,pipe=start();pid2=proc.pid;newpins=discover();assert all(pins[k]!=newpins[k] for k in pins)
  expected,pair=samples[-1];send(pair);assert adapter.faulted
  oldgen=generation;generation=uuid.uuid4().hex;adapter=RawVisualCallback(generation,'v321-local-new','epoch2',expected_gids=newpins)
  # A queued callback from the prior generation must not consume the new collector.
  msg=deserialize_message(base64.b64decode(pair['odometry']['cdr_base64']),Odometry);now=time.monotonic_ns()
  try:adapter.receive('odometry',msg,{'publisher_gid':newpins['odometry']},callback_generation=oldgen,received_ns=now,now_ns=now,expected_stamp_ns=expected)
  except ValueError as e:report['old_generation_error']=str(e)
  assert report['old_generation_error']=='old_callback_generation'
  phase='explicit_new_generation'
  for expected,pair in samples:send(pair)
  report.update(initial_pairs=sum(r.get('decision',{}).get('structurally_eligible',False) for r in rows if r['phase']=='initial'),new_generation_pairs=sum(r.get('decision',{}).get('structurally_eligible',False) for r in rows if r['phase']=='explicit_new_generation'),restart_errors=[r.get('error') for r in rows if r['phase']=='restart_old_adapter'],pids=[pid1,pid2],first_exit=exit1,original_pins=pins,new_pins=newpins)
  assert report['initial_pairs']==20 and report['new_generation_pairs']==20
  assert report['restart_errors']==['publisher_endpoint_changed','callback_faulted_requires_new_instance']
  report['diagnostic_pass']=True
 finally:
  stop();report['last_child_exit']=proc.exitcode if proc else None
  node.destroy_node();rclpy.shutdown();report.update(events=len(rows),owned_processes_reaped=True,source_session_verified=False,capture_freshness_verified=False,execution_authorized=False,scope='Separate-process ROS source restart using saved static-image CDR. Harness assigns expected stamp and explicitly opens new local generation. Not cuVSLAM restart, authenticated capture, controller integration or motion.')
  (out/'v321_callback_events.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows));(out/'v321_report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
if __name__=='__main__':main()
