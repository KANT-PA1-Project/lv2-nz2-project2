"""Replay recorded CDR arrival order/times; session labels are offline mapping only."""
import json,base64,collections
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from arm_practice.raw_visual_evidence import snapshot_raw_odometry,snapshot_raw_status
from arm_practice.raw_visual_collector import RawVisualCollector
out=Path(__file__).resolve().parents[3]/'outputs/r12';replay=json.loads((out/'v300_replay.json').read_text());sent={r['stamp_ns']:r for r in replay['rows']};events=[json.loads(x) for x in (out/'v300_raw/raw_messages.jsonl').read_text().splitlines()];assert all(a['received_monotonic_ns']<=b['received_monotonic_ns'] for a,b in zip(events,events[1:]));rows=[];states=collections.Counter();c=RawVisualCollector(replay['run_id'],replay['clock_epoch'],capacity=8,timeout_ns=250000000);peak=0
for event in events:
 cls=Odometry if event['type']=='Odometry' else VisualSlamStatus;message=deserialize_message(base64.b64decode(event['cdr_base64']),cls);kind='odometry' if cls is Odometry else 'status';snapshot=snapshot_raw_odometry if cls is Odometry else snapshot_raw_status
 record=snapshot(message,run_id=replay['run_id'],clock_epoch=replay['clock_epoch'],received_ns=event['received_monotonic_ns']);result=c.push(kind,record,now_ns=event['received_monotonic_ns']);states[result['state']]+=1;peak=max(peak,len(c.pending))
 if result['pair']:
  o=result['pair']['raw_odometry'];s=o['raw_status'];source=sent[o['stamp_ns']];done=max(o['received_monotonic_ns'],s['received_monotonic_ns']);age=(done-source['replay_publish_monotonic_ns'])/1e6
  rows.append(dict(frame_index=source['frame_index'],stamp_ns=o['stamp_ns'],publish_to_pair_ms=age,pair_arrival_gap_ms=abs(o['received_monotonic_ns']-s['received_monotonic_ns'])/1e6,within50ms=age<=50,within250ms=age<=250,tracking_valid=o['tracking_valid'],source_session_verified=False))
summary={}
for name,subset in [('all',rows),('frames0to29',[r for r in rows if r['frame_index']<30]),('frames30to99',[r for r in rows if r['frame_index']>=30])]:
 vals=[r['publish_to_pair_ms'] for r in subset];summary[name]=dict(pairs=len(vals),median_ms=float(np.median(vals)),p95_ms=float(np.quantile(vals,.95)),max_ms=max(vals),within50ms=sum(r['within50ms'] for r in subset),within250ms=sum(r['within250ms'] for r in subset))
report=dict(events=len(events),states=dict(states),peak_pending=peak,remaining_pending=len(c.pending),paired=len(rows),summary=summary,max_pair_gap_ms=max(r['pair_arrival_gap_ms'] for r in rows),slow_frames=[r['frame_index'] for r in rows if not r['within250ms']],rows=rows,scope='First30frames is a declared analysis partition,not detected init boundary. Pair timeout starts at first message receipt,not image publish. Successful pair is not a fresh observation. Offline assigned replay session labels are not source-authenticated.',runtime_changed=False,pose_join_verified=False)
(out/'v301_arrival_analysis.json').write_text(json.dumps(report,indent=2)+'\n');print({k:v for k,v in report.items() if k!='rows'})
