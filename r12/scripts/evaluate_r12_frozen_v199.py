"""Frozen paired evaluation. Does not update weights or select thresholds."""
import json,hashlib
from pathlib import Path
import cv2,numpy as np,torch
root=Path(__file__).resolve().parents[1];b=root.parent.parent/'outputs/r12';src=b/'tag_render_v199';rows=json.loads((src/'manifest.json').read_text())['rows'];plan=json.loads((b/'v199_frozen_evaluation_plan.json').read_text());torch.set_num_threads(2);assert len(rows)==144
X=torch.tensor(np.array([cv2.imread(str(src/f'replicate_{r["index"]:03d}.png'),0) for r in rows])[:,None]/255,dtype=torch.float32);y=np.array([r['tag_id'] in r['decoder_variants']['input64_replicate'] for r in rows]);negative=np.array([r['tag_id']<0 for r in rows]);assert not any(r['decoder_variants']['input64_replicate'] for r in rows if r['tag_id']<0)
net=torch.nn.Sequential(torch.nn.Conv2d(1,8,5,stride=2,padding=2),torch.nn.ReLU(),torch.nn.Conv2d(8,16,3,stride=2,padding=1),torch.nn.ReLU(),torch.nn.Conv2d(16,24,3,stride=2,padding=1),torch.nn.ReLU(),torch.nn.Flatten(),torch.nn.Linear(24*8*8,32),torch.nn.ReLU(),torch.nn.Linear(32,1));result={}
for version,key,cutoff in [('v184','baseline_model_sha256','baseline_threshold'),('v198','model_sha256','threshold')]:
 p=b/f'tag_quality_{version}/model.pt';assert hashlib.sha256(p.read_bytes()).hexdigest()==plan[key];net.load_state_dict(torch.load(p,weights_only=False)['weights']);net.eval()
 with torch.no_grad():scores=net(X).flatten().sigmoid().numpy()
 selected=scores>=plan[cutoff];tp=int((selected&y).sum());fp=int((selected&~y).sum());fn=int((~selected&y).sum());result[version]={'true_positive':tp,'false_positive':fp,'false_negative':fn,'precision':tp/(tp+fp) if tp+fp else None,'recall':tp/(tp+fn) if tp+fn else None,'brier':float(np.mean((scores-y)**2)),'tag_absent_false_positive':int(selected[negative].sum()),'threshold':plan[cutoff]}
r={'complete':True,'samples':len(rows),'decoder_positive':int(y.sum()),'tag_absent_samples':int(negative.sum()),'models':result,'scope':plan['scope'],'runtime_deployed':False};(b/'v199_frozen_evaluation.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
