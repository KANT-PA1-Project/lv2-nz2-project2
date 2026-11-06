"""Frozen paired evaluation. Does not update weights or select thresholds."""
import json,hashlib
from pathlib import Path
import cv2,numpy as np,torch
root=Path(__file__).resolve().parents[1];b=root.parent.parent/'outputs/r12';src=b/'tag_render_v199';rows=json.loads((src/'manifest.json').read_text())['rows'];plan=json.loads((b/'v199_frozen_evaluation_plan.json').read_text());torch.set_num_threads(2);assert len(rows)==144
X=torch.tensor(np.array([cv2.imread(str(src/f'replicate_{r["index"]:03d}.png'),0) for r in rows])[:,None]/255,dtype=torch.float32);y=np.array([r['tag_id'] in r['decoder_variants']['input64_replicate'] for r in rows]);negative=np.array([r['tag_id']<0 for r in rows]);assert not any(r['decoder_variants']['input64_replicate'] for r in rows if r['tag_id']<0)
net=torch.nn.Sequential(torch.nn.Conv2d(1,8,5,stride=2,padding=2),torch.nn.ReLU(),torch.nn.Conv2d(8,16,3,stride=2,padding=1),torch.nn.ReLU(),torch.nn.Conv2d(16,24,3,stride=2,padding=1),torch.nn.ReLU(),torch.nn.Flatten(),torch.nn.Linear(24*8*8,32),torch.nn.ReLU(),torch.nn.Linear(32,1));result={}
p=b/'tag_quality_v198/model.pt';assert hashlib.sha256(p.read_bytes()).hexdigest()==plan['model_sha256'];net.load_state_dict(torch.load(p,weights_only=False)['weights']);net.eval()
with torch.no_grad():scores=net(X).flatten().sigmoid().numpy()
records=[]
for i,row in enumerate(rows):
 if row['tag_id']<0:continue
 im=(X[i,0].numpy()*255).round().astype(np.uint8);ys,xs=np.where(im<100)
 bounds=None if not len(xs) else [int(xs.max()-xs.min()+1),int(ys.max()-ys.min()+1)]
 records.append(dict(index=row['index'],camera=row['camera'],distance_m=row['distance_m'],tilt_rad=row['tilt_rad'],light_intensity=row['light_intensity'],dark_pixel_extent_proxy=bounds,score=float(scores[i]),decoder_success=bool(y[i]),false_positive=bool(scores[i]>=plan['threshold'] and not y[i]),false_negative=bool(scores[i]<plan['threshold'] and y[i])))
summary={}
for distance in sorted({r['distance_m'] for r in records}):
 group=[r for r in records if r['distance_m']==distance];summary[str(distance)]={'n':len(group),'decoder_success':sum(r['decoder_success'] for r in group),'false_positive':sum(r['false_positive'] for r in group),'false_negative':sum(r['false_negative'] for r in group)}
r={'complete':True,'by_distance':summary,'false_positives':[r for r in records if r['false_positive']],'false_negatives':[r for r in records if r['false_negative']],'rows':records,'scope':'Posthoc diagnosis only. Dark-pixel extent is an image proxy, not a measured physical tag boundary. V199 now consumed for diagnostics; do not tune on it and claim fresh holdout performance. No threshold or model change.'}
(b/'v200_error_diagnosis.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({k:v for k,v in r.items() if k!='rows'},indent=2))
