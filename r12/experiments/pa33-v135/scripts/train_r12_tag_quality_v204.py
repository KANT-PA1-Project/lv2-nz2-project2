"""Full-network adaptation with validation early stopping; train/validation only. Development experiment, no deployment."""
from pathlib import Path
import json,hashlib,cv2,numpy as np,torch
root=Path(__file__).resolve().parents[1];base=root.parent.parent/'outputs/r12';src=base/'tag_render_v201';out=base/'tag_quality_v204';out.mkdir(exist_ok=False);torch.set_num_threads(2);torch.manual_seed(202);rng=np.random.default_rng(202)
rows=json.loads((src/'manifest.json').read_text())['rows'];assert len(rows)==480;rows=[r for r in rows if r['split']!='test']
splits={s:np.array([i for i,r in enumerate(rows) if r['split']==s]) for s in ['train','validation']}
hashes={s:{rows[i]['texture_sha256'] for i in ix} for s,ix in splits.items()};assert not (hashes['train']&hashes['validation'])
images=np.array([cv2.imread(str(src/f'replicate_{r["index"]:03d}.png'),0) for r in rows]);ys=np.array([int(r['tag_id'] in r['decoder_variants']['input64_replicate']) for r in rows]);assert all(len(set(ys[ix]))==2 for ix in splits.values());assert not any(r['decoder_variants']['input64_replicate'] for r in rows if r['tag_id']<0)
X=torch.tensor(images[:,None]/255,dtype=torch.float32);Y=torch.tensor(ys,dtype=torch.float32)
net=torch.nn.Sequential(torch.nn.Conv2d(1,8,5,stride=2,padding=2),torch.nn.ReLU(),torch.nn.Conv2d(8,16,3,stride=2,padding=1),torch.nn.ReLU(),torch.nn.Conv2d(16,24,3,stride=2,padding=1),torch.nn.ReLU(),torch.nn.Flatten(),torch.nn.Linear(24*8*8,32),torch.nn.ReLU(),torch.nn.Linear(32,1));parent=base/'tag_quality_v184/model.pt';net.load_state_dict(torch.load(parent,weights_only=False)['weights'])
for param in net.parameters():param.requires_grad_(True)
opt=torch.optim.Adam(net.parameters(),lr=.0001)
def predict(ix):
 with torch.no_grad():return net(X[ix]).flatten().sigmoid().numpy()
initial={s:predict(ix).tolist() for s,ix in splits.items()};best=float('inf');trace=[];stale=0
for step in range(601):
 ix=splits['train'];loss=torch.nn.functional.binary_cross_entropy_with_logits(net(X[ix]).flatten(),Y[ix]);opt.zero_grad();loss.backward();opt.step()
 if step%10==0:
  v=predict(splits['validation']);brier=float(np.mean((v-ys[splits['validation']])**2));trace.append({'step':step,'validation_brier':brier});print('step',step,'validation_brier',brier,flush=True)
  if brier<best:stale=0;best=brier;torch.save({'weights':net.state_dict(),'step':step,'input_contract':'v197_replicate64','task':'native64_replicate_readability'},out/'model.pt')
  else:stale+=1
  if stale>=10:break
saved=torch.load(out/'model.pt',weights_only=False);net.load_state_dict(saved['weights']);v=predict(splits['validation']);choices=[float(t) for t in np.linspace(.5,.99,50) if (v>=t).sum()>=10 and ys[splits['validation']][v>=t].mean()>=.95];threshold=min(choices) if choices else None
metrics={}
for s,ix in splits.items():
 p=predict(ix);mask=p>=threshold if threshold is not None else np.zeros(len(ix),bool);metrics[s]={'n':len(ix),'positive_labels':int(ys[ix].sum()),'brier_before':float(np.mean((np.array(initial[s])-ys[ix])**2)),'brier_after':float(np.mean((p-ys[ix])**2)),'predicted_positive':int(mask.sum()),'precision':float(ys[ix][mask].mean()) if mask.any() else None,'recall':float((ys[ix]*mask).sum()/ys[ix].sum())}
report={'complete':True,'completed_steps':step+1,'test_images_read':False,'early_stop_patience_checks':10,'learning_rate':.0001,'selected_step':saved['step'],'threshold_selected_on_validation':threshold,'metrics':metrics,'parent_sha256':hashlib.sha256(parent.read_bytes()).hexdigest(),'dataset_manifest_sha256':hashlib.sha256((src/'manifest.json').read_bytes()).hexdigest(),'trained_parameters':sum(p.numel() for p in net.parameters() if p.requires_grad),'input_contract':'New replicate padding native64 target. Old white64 labels and models unchanged.','limitations':'Development ablation on reused train/validation. Full network adapted from V184; same seed as V202, lower learning rate and early stopping. Not isolated causal proof. No test images loaded; fresh holdout required before qualification.','runtime_deployed':False}
(out/'evaluation.json').write_text(json.dumps(report,indent=2)+'\n');(out/'trace.json').write_text(json.dumps(trace,indent=2));print('COMPLETE',json.dumps(report),flush=True)
