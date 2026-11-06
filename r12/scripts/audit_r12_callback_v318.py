import json,base64,bisect,collections,sys
from pathlib import Path
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
repo=Path(__file__).resolve().parents[1];sys.path.insert(0,str(repo/'src'));out=repo.parents[1]/'outputs/r12'
from arm_practice.raw_visual_callback import RawVisualCallback
from arm_practice.raw_visual_evidence import snapshot_raw_odometry,snapshot_raw_status
runs={}
for v in ('v313','v314'):
 replay=json.loads((out/f'{v}_replay.json').read_text());sent=replay['rows'];times=[x['replay_publish_monotonic_ns'] for x in sent];events=[json.loads(x) for x in (out/f'{v}_raw/raw_messages.jsonl').read_text().splitlines() if json.loads(x)['type']!='Image']
 modes={}
 for mode in ('recorded','processing_delayed_51ms'):
  summary=json.loads((out/f'{v}_raw/summary.json').read_text());pinned={}
  for kind,topic in [('odometry','/visual_slam/tracking/odometry'),('status','/visual_slam/status')]:
   endpoints=summary['live_publishers'][topic]['endpoints'];assert len(endpoints)==1
   pinned[kind]=dict(implementation_identifier='rmw_fastrtps_cpp',data=endpoints[0]['gid'])
  a=RawVisualCallback(v,replay['run_id'],replay['clock_epoch'],expected_gids=pinned);counts=collections.Counter();eligible=0;rejected=[]
  for e in events:
   cls=Odometry if e['type']=='Odometry' else VisualSlamStatus;msg=deserialize_message(base64.b64decode(e['cdr_base64']),cls);kind='odometry' if cls is Odometry else 'status';snapshot=snapshot_raw_odometry if cls is Odometry else snapshot_raw_status
   r=snapshot(msg,run_id=replay['run_id'],clock_epoch=replay['clock_epoch'],received_ns=e['received_monotonic_ns']);now=e['received_monotonic_ns']+(51000000 if mode!='recorded' else 0);index=bisect.bisect_right(times,now)-1;assert index>=0
   answer=a.receive(kind,msg,{'publisher_gid':e['publisher_gid']},callback_generation=v,received_ns=e['received_monotonic_ns'],now_ns=now,expected_stamp_ns=sent[index]['stamp_ns']);counts[answer.get('reason',answer['state'])]+=1;eligible+=answer['structurally_eligible']
   if answer['state']=='paired' and not answer['structurally_eligible']:rejected.append(dict(stamp_ns=r['stamp_ns'],expected_stamp_ns=sent[index]['stamp_ns'],reason=answer['reason']))
  modes[mode]=dict(events=len(events),eligible_pairs=eligible,states=dict(counts),rejected_pairs=rejected)
 runs[v]=modes
report=dict(runs=runs,scope='Current stamp derived from recorded replay publication timeline;local session labels assigned offline,not source authentication. Delayed mode is synthetic processing delay,not measured transport. Eligibility only structural/receipt/current-stamp;no capture freshness or motion authorization.',runtime_changed=False,endpoint_pins_from_live_discovery=True)
(out/'v318_callback_audit.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
