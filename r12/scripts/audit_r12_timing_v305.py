"""Offline diagnostic timing, never grants observation or motion acceptance."""
import json,collections,base64
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
out=Path(__file__).resolve().parents[3]/'outputs/r12';replay=json.loads((out/'v305_replay.json').read_text());sent={r['stamp_ns']:r for r in replay['rows']};events=[json.loads(x) for x in (out/'v305_raw/raw_messages.jsonl').read_text().splitlines()]
def stat(v):
 return dict(n=len(v),median=float(np.median(v)),p95=float(np.quantile(v,.95)),max=max(v),min=min(v)) if v else dict(n=0)
metrics=collections.defaultdict(list);groups=collections.defaultdict(list);rows=[]
for e in events:
 kind=e['type'];groups[e['source_stamp_ns']].append(e)
 if e['dds_source_timestamp_ns']>0 and e['dds_received_timestamp_ns']>0:
  metrics[kind+'_dds_source_to_receive_ms'].append((e['dds_received_timestamp_ns']-e['dds_source_timestamp_ns'])/1e6)
  metrics[kind+'_dds_receive_to_callback_ms'].append((e['callback_wall_ns']-e['dds_received_timestamp_ns'])/1e6)
for ns,group in groups.items():
 s=sent[ns];imgs=[e for e in group if e['type']=='Image'];statuses=[e for e in group if e['type']=='VisualSlamStatus'];row=dict(frame_index=s['frame_index'],image_count=len(imgs),has_status=bool(statuses))
 if len(imgs)==6:
  row['image_callback_span_ms']=(max(e['received_monotonic_ns'] for e in imgs)-min(e['received_monotonic_ns'] for e in imgs))/1e6
  row['publish_to_last_image_ms']=(max(e['received_monotonic_ns'] for e in imgs)-s['replay_publish_monotonic_ns'])/1e6
 if statuses:
  e=statuses[0];msg=deserialize_message(base64.b64decode(e['cdr_base64']),VisualSlamStatus)
  row['publish_to_status_ms']=(e['received_monotonic_ns']-s['replay_publish_monotonic_ns'])/1e6
  row['publish_to_status_dds_source_ms']=(e['dds_source_timestamp_ns']-s['replay_publish_wall_ns'])/1e6
  row['slam_callback_ms']=msg.node_callback_execution_time*1000
 for k,v in row.items():
  if k.endswith('_ms'):metrics[k].append(v)
 rows.append(row)
report=dict(sent=len(sent),counts=dict(collections.Counter(e['type'] for e in events)),complete_image_sets=sum(r['image_count']==6 for r in rows),metrics={k:stat(v) for k,v in metrics.items()},rows=rows,scope='Image observer is separate from cuVSLAM subscriptions and may perturb timing. DDS timestamps compared with system wall clock,not simulation or monotonic clock. No cuVSLAM input callback timestamp;cannot locate delay inside synchronizer conclusively.',source_session_verified=False,runtime_changed=False,performance_pass=False)
(out/'v305_timing_analysis.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k!='rows'},indent=2))
