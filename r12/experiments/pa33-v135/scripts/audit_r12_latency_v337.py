import json,base64,collections
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
out=Path(__file__).resolve().parents[3]/'outputs/r12';p=out/'v337_stationary';events=[json.loads(x) for x in (p/'raw_callback_audit.jsonl').read_text().splitlines()];rows=json.loads((p/'ros_results.json').read_text());by={r['stamp_ns']:r for r in rows}
def stat(v):return dict(n=len(v),median_ms=float(np.median(v)),max_ms=max(v),within50=sum(x<=50 for x in v)) if v else dict(n=0)
metrics=collections.defaultdict(list);missing=0;tracking=0
for e in events:
 c=e['capture']
 if c is None:missing+=1;continue
 assert c['run_id']==by[e['stamp_ns']]['capture_run_id']
 metrics['capture_to_'+e['kind']].append((e['receipt_monotonic_ns']-c['capture_monotonic_ns'])/1e6)
 metrics['packet_to_'+e['kind']].append((e['receipt_monotonic_ns']-c['packet_received_monotonic_ns'])/1e6)
 if e['kind']=='status':
  msg=deserialize_message(base64.b64decode(e['cdr_base64']),VisualSlamStatus);tracking+=msg.vo_state==1;metrics['reported_slam_callback'].append(msg.node_callback_execution_time*1000)
report=dict(events=len(events),missing_capture_links=missing,current_synthetic_stamp_matches=sum(e['stamp_ns']==e['current_packet_stamp_ns'] for e in events),tracking_success=tracking,metrics={k:stat(v) for k,v in metrics.items()},scope='Same-host wall monotonic timing;current synthetic stamp does not establish <=50ms capture age. Acquisition start includes render/readback/self masking/NPZ and packet preparation. Reported SLAM callback duration is not full pipeline latency.',performance_pass=False,execution_authorized=False)
(out/'v337_latency_audit.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
