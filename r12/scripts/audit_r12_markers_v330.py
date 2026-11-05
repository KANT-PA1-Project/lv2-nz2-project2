import json,hashlib
from pathlib import Path
import numpy as np
from PIL import Image
out=Path(__file__).resolve().parents[3]/'outputs/r12'
meta=[json.loads(x) for x in (out/'v330_debug/frame_metadata.jsonl').read_text().splitlines()]
manifest=json.loads((out/'v299_replay_manifest.json').read_text());sent=json.loads((out/'v330_outgoing_evidence.json').read_text());lookup={(e['stamp_ns'],e['frame_id']):e for e in sent};rows=[]
for group in meta:
 for c in group['cams']:
  frame=(c['timestamp']-1000000000)//50000000;camera=c['id'];expected=2*(1+frame*6+camera)
  a=np.asarray(Image.open(out/'v330_debug'/c['filename']));patch=a[:16,:16].astype(int)
  rgb=np.asarray(Image.open(manifest['images'][camera]['image']).convert('RGB')).copy();rgb[:16,:16,:]=expected
  digest=hashlib.sha256(rgb.tobytes()).hexdigest();original=lookup[(c['timestamp'],manifest['images'][camera]['camera_frame'])]
  rows.append(dict(frame=frame,camera=camera,stamp=c['timestamp'],expected=expected,observed_min=int(patch.min()),observed_max=int(patch.max()),exact=bool(np.all(patch==expected)),within_one=bool(np.all(abs(patch-expected)<=1)),outgoing_reconstruction_matches=digest==original['image_sha256']))
raw=[json.loads(x) for x in (out/'v330_raw/raw_messages.jsonl').read_text().splitlines()];stamps={e['source_stamp_ns'] for e in raw if e['type']=='Odometry'}
r=dict(debug_sets=len(meta),images=len(rows),exact_marker_images=sum(x['exact'] for x in rows),within_one_images=sum(x['within_one'] for x in rows),outgoing_reconstructions_match=sum(x['outgoing_reconstruction_matches'] for x in rows),dump_stamps_match_output_stamps={x['stamp'] for x in rows}==stamps,rows=rows,scope='Synthetic corner markers validate camera/time/content correspondence of internal debug dumps. They can perturb tracking. Dump/track call ordering and full grayscale conversion remain unverified.',input_consumption_verified=False,execution_authorized=False)
(out/'v330_marker_audit.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({k:v for k,v in r.items() if k!='rows'},indent=2))
