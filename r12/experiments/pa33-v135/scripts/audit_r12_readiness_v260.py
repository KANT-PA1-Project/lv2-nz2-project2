"""Replay recorded content, without inventing capture or response timestamps."""
import json
from collections import Counter
from pathlib import Path
from arm_practice.observation_readiness import check_archived_content

root=Path(__file__).resolve().parents[3]/'outputs/r12'
source=root/'v247_worker_replay/results.json'
data=json.loads(source.read_text())
rows=[]
for sample in data['samples']:
    # V247 retained tag count, not tag corner evidence. Zero is reconstructible;
    # nonzero cannot be recovered and must not be converted into accepted tags.
    assert sample['tags']==0, 'Missing retained tag evidence: cannot audit this sample'
    response={'detections':sample['detection_records'],'tags':[]}
    for i,d in enumerate(response['detections']):
        if not 0<=d['class_id']<4:continue
        verdict=check_archived_content(response,i)
        rows.append(dict(scene=sample['scene'],camera=sample['camera'],repeat=sample['repeat'],
                         detection_index=i,class_id=d['class_id'],**verdict))
report=dict(source=str(source),scope='archived content only; no identity/timestamps synthesized',
            samples=len(data['samples']),puck_candidate_records=len(rows),
            unique_view_candidate_records=sum(r['repeat']==0 for r in rows),
            reasons=dict(Counter(r['content_reason'] for r in rows)),
            repeat_zero_reasons=dict(Counter(r['content_reason'] for r in rows if r['repeat']==0)),
            evidence_ready_count=sum(r['evidence_ready'] for r in rows),
            live_freshness_verified=False,execution_authorized=False,
            limitation='First rejection only; missing tags prevent subsequent pose/depth evaluation. Repetitions are not independent scenes.',rows=rows)
(root/'v260_readiness_audit.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='rows'},indent=2))
