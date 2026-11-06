"""Check per-topic endpoint consistency; GID consistency is not source run identity."""
import json,collections
from pathlib import Path
out=Path(__file__).resolve().parents[3]/'outputs/r12'
events=[json.loads(x) for x in (out/'v313_raw/raw_messages.jsonl').read_text().splitlines()];summary=json.loads((out/'v313_raw/summary.json').read_text());replay=json.loads((out/'v313_replay.json').read_text());sent={r['stamp_ns'] for r in replay['rows']};topics={}
for topic in summary['publishers']:
 rows=[e for e in events if e['topic']==topic];gids={tuple(e['publisher_gid']['data']) for e in rows};endpoints={tuple(e['gid']) for e in summary['live_publishers'].get(topic,{}).get('endpoints',[])};counts=collections.Counter(e['source_stamp_ns'] for e in rows)
 topics[topic]=dict(events=len(rows),unique_stamps=len(counts),duplicate_stamp_count=sum(n-1 for n in counts.values()),gid_count=len(gids),discovered_endpoint_count=len(endpoints),gids_match_discovered_endpoints=(gids<=endpoints if endpoints else None),stamps_outside_replay=len(set(counts)-sent))
report=dict(topics=topics,raw_events=len(events),recorder_limit=2048,limit_reached=len(events)>=2048,source_run_id_transmitted=False,source_clock_epoch_transmitted=False,source_session_verified=False,pose_join_verified=False,execution_authorized=False,scope='Endpoint GID checks are recording consistency only; no authenticated source capture/run identity. Static replay time is not real sensor capture time.')
(out/'v313_identity_audit.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
