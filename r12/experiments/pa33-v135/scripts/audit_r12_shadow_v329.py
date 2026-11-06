import json,base64,collections,sys
from pathlib import Path
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
repo=Path(__file__).resolve().parents[1];sys.path.insert(0,str(repo/'src'))
from arm_practice.raw_visual_callback import RawVisualCallback
out=repo.parents[1]/'outputs/r12';raw=out/'v329_raw';ready=json.loads((raw/'admission_ready.json').read_text())
a=RawVisualCallback(ready['generation'],ready['recording_id'],'local-epoch',expected_gids=ready['pins'])
events=[json.loads(x) for x in (raw/'raw_messages.jsonl').read_text().splitlines() if json.loads(x)['type']!='Image'];states=collections.Counter();mismatch=[];eligible=0
for i,e in enumerate(events):
 cls=Odometry if e['type']=='Odometry' else VisualSlamStatus
 msg=deserialize_message(base64.b64decode(e['cdr_base64']),cls)
 answer=a.receive('odometry' if cls is Odometry else 'status',msg,{'publisher_gid':e['publisher_gid']},callback_generation=ready['generation'],received_ns=e['received_monotonic_ns'],now_ns=e['decision_now_ns'],expected_stamp_ns=e['expected_stamp_ns'])
 result={k:v for k,v in answer.items() if k not in ('received_snapshot','raw_odometry')}
 if result!=e['admission']:mismatch.append(i)
 states[result.get('reason',result['state'])]+=1;eligible+=result['structurally_eligible']
clock=json.loads((raw/'clock_events.json').read_text())
report=dict(events=len(events),eligible_pairs=eligible,states=dict(states),offline_live_mismatches=mismatch,clock_events=len(clock['events']),clock_regression=clock['fault'],callback_expected_stamp_mismatches=sum(e['expected_stamp_ns']!=e['source_stamp_ns'] for e in events),source_session_verified=False,capture_freshness_verified=False,execution_authorized=False,runtime_changed=False,scope='Live diagnostics on static-image cuVSLAM outputs; offline replay uses recorded clock/receipt/decision times. Pinned endpoint continuity and local clock observation do not authenticate capture provenance.')
(out/'v329_shadow_audit.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
