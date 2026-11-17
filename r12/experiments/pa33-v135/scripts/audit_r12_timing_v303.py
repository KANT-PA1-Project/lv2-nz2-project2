"""Decode unmodified CDR timing fields; no runtime or freshness authorization."""
import json,base64,sys
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
out=Path(__file__).resolve().parents[3]/'outputs/r12'
def stats(v):
 return dict(count=len(v),median=float(np.median(v)),p95=float(np.quantile(v,.95)),max=max(v)) if v else dict(count=0)
reports={}
for version in sys.argv[1:]:
 replay=json.loads((out/f'{version}_replay.json').read_text());sent={r['stamp_ns']:r for r in replay['rows']};rows=[]
 for line in (out/f'{version}_raw/raw_messages.jsonl').read_text().splitlines():
  event=json.loads(line)
  if event['type']!='VisualSlamStatus':continue
  msg=deserialize_message(base64.b64decode(event['cdr_base64']),VisualSlamStatus);ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec;s=sent[ns]
  row=dict(frame_index=s['frame_index'],state=msg.vo_state,publish_to_status_ms=(event['received_monotonic_ns']-s['replay_publish_monotonic_ns'])/1e6,callback_ms=msg.node_callback_execution_time*1000,track_ms=msg.track_execution_time*1000)
  if 'replay_publish_complete_monotonic_ns' in s:
   row.update(publish_duration_ms=(s['replay_publish_complete_monotonic_ns']-s['replay_publish_monotonic_ns'])/1e6,complete_to_status_ms=(event['received_monotonic_ns']-s['replay_publish_complete_monotonic_ns'])/1e6)
  rows.append(row)
 summary={k:stats([r[k] for r in rows]) for k in rows[0] if k not in ('frame_index','state')};summary.update(sent=len(sent),received=len(rows),within50ms=sum(r['publish_to_status_ms']<=50 for r in rows),within250ms=sum(r['publish_to_status_ms']<=250 for r in rows),source_session_verified=False)
 reports[version]=dict(summary=summary,rows=rows)
 print(version,json.dumps(summary))
(out/'v303_timing_analysis.json').write_text(json.dumps(dict(runs=reports,scope='Static replay only; callback fields seconds per installed ROS msg. End-to-end minus callback is not a direct transport measurement; callback boundaries differ and asynchronous work may overlap. No freshness or motion proof.'),indent=2)+'\n')
