import json,base64,collections,sys
from pathlib import Path
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
repo=Path(__file__).resolve().parents[1];sys.path.insert(0,str(repo/'src'));out=repo.parents[1]/'outputs/r12'
from arm_practice.raw_visual_evidence import snapshot_raw_odometry
from arm_practice.raw_odometry_gate import check_raw_odometry
counts=collections.Counter();rows=[]
for v in ('v313','v314'):
 last=None
 for line in (out/f'{v}_raw/raw_messages.jsonl').read_text().splitlines():
  e=json.loads(line)
  if e['type']!='Odometry':continue
  m=deserialize_message(base64.b64decode(e['cdr_base64']),Odometry)
  r=snapshot_raw_odometry(m,run_id='offline-unverified',clock_epoch=v,received_ns=e['received_monotonic_ns'])
  check=check_raw_odometry(r,expected_stamp_ns=e['source_stamp_ns'],last_consumed_stamp_ns=last,now_ns=e['received_monotonic_ns'],max_receipt_age_ns=50000000)
  counts[check['reason']]+=1;rows.append(dict(run=v,stamp_ns=e['source_stamp_ns'],**check));last=e['source_stamp_ns']
report=dict(samples=len(rows),reasons=dict(counts),rows=rows,scope='At recorded receipt time;expected stamps from same recorded event are not independently capture-verified. Original CDR unchanged;no bridge integration.',runtime_changed=False,source_session_verified=False)
(out/'v316_raw_gate_audit.json').write_text(json.dumps(report,indent=2)+'\n');print(dict(samples=len(rows),reasons=dict(counts)))
