"""Frozen V207 evaluation; feature function loaded without training side effects."""
from pathlib import Path
import ast,json,hashlib,cv2,numpy as np,torch
root=Path(__file__).resolve().parents[1];base=root.parent.parent/'outputs/r12';src=base/'tag_render_v208';plan=json.loads((base/'v208_acquisition_plan.json').read_text());model=base/'tag_quality_v207/model.pt'
assert hashlib.sha256(model.read_bytes()).hexdigest()==plan['model_sha256']
tree=ast.parse((root/'scripts/train_r12_tag_quality_v207.py').read_text());f=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='features');exec(compile(ast.Module(body=[f],type_ignores=[]),'v207_features','exec'))
net=torch.nn.Sequential(torch.nn.Linear(133,16),torch.nn.ReLU(),torch.nn.Linear(16,1));saved=torch.load(model,weights_only=False);net.load_state_dict(saved['weights']);net.eval()
manifest=json.loads((src/'manifest.json').read_text());assert manifest['complete'];rows=manifest['rows'];assert len(rows)==144
xx=[];yy=[]
for r in rows:
 im=cv2.imread(str(src/f'replicate_{r["index"]:03d}.png'),0);assert im is not None
 xx.append(features(im)[0]);yy.append(r['tag_id'] in r['decoder_variants']['input64_replicate'])
with torch.no_grad():p=net(torch.tensor((np.array(xx)-saved['mean'])/saved['scale'])).flatten().sigmoid().numpy()
y=np.array(yy);mask=p>=plan['threshold'];tp=int((mask&y).sum());fp=int((mask&~y).sum());fn=int((~mask&y).sum());precision=tp/(tp+fp) if tp+fp else None
report={'complete':True,'samples':len(rows),'positive_labels':int(y.sum()),'threshold':plan['threshold'],'tp':tp,'fp':fp,'fn':fn,'precision':precision,'recall':tp/int(y.sum()) if y.sum() else None,'brier':float(np.mean((p-y)**2)),'criterion_met':bool(tp+fp>=10 and precision is not None and precision>=.95),'model_sha256':plan['model_sha256'],'runtime_deployed':False,'limitations':'Fresh IDs/seeds but same controlled renderer; not mission/hardware validation. No model or threshold tuning.','predictions':[{'index':r['index'],'score':float(v),'correct_decode_label':bool(t)} for r,v,t in zip(rows,p,y)]}
(base/'v208_frozen_evaluation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k!='predictions'}))
