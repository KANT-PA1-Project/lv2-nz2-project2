"""Proposal-distribution training, disjoint scene seeds/identities by split."""
from pathlib import Path
import ast,json,hashlib,cv2,numpy as np,torch
root=Path(__file__).resolve().parents[1];base=root.parent.parent/'outputs/r12';src=base/'tag_render_v218';out=base/'tag_quality_v223';out.mkdir(exist_ok=False);torch.set_num_threads(2);torch.manual_seed(215)
for filename,fn in [('train_r12_tag_quality_v207.py','features'),('evaluate_r12_proposals_v209.py','proposals')]:
 t=ast.parse((root/'scripts'/filename).read_text());f=next(n for n in t.body if isinstance(n,ast.FunctionDef) and n.name==fn);exec(compile(ast.Module(body=[f],type_ignores=[]),filename,'exec'))
manifest=json.loads((src/'manifest.json').read_text());assert manifest['complete'] and len(manifest['rows'])==192
rows=manifest['rows'];seeds={s:{r['background_seed'] for r in rows if r['split']==s} for s in ['train','validation']};assert not seeds['train']&seeds['validation']
det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11));xx=[];yy=[];meta=[]
for r in rows:
 im=cv2.imread(str(src/f'frame_{r["index"]:03d}.png'),0);assert im is not None
 boxes,crops=proposals(im)
 for box,crop in zip(boxes,crops):
  feat,_=features(crop);xx.append(feat)
  _,ids,_=det.detectMarkers(crop);decoded=[] if ids is None else ids.flatten().tolist();yy.append(r['tag_id']>=0 and r['tag_id'] in decoded);meta.append({'frame':r['index'],'split':r['split'],'box':box})
x=np.array(xx);y=np.array(yy,np.float32);ix={s:np.array([i for i,r in enumerate(meta) if r['split']==s]) for s in seeds};assert all(len(set(y[i]))==2 for i in ix.values())
mean=x[ix['train']].mean(0);scale=np.maximum(x[ix['train']].std(0),.01);X=torch.tensor((x-mean)/scale);Y=torch.tensor(y)
net=torch.nn.Sequential(torch.nn.Linear(133,16),torch.nn.ReLU(),torch.nn.Linear(16,1));opt=torch.optim.AdamW(net.parameters(),lr=.001,weight_decay=.01)
weight=1.0
def predict(i):
 with torch.no_grad():return net(X[i]).flatten().sigmoid().numpy()
best=-1.;stale=0;trace=[]
def average_precision(p,y):
 order=np.argsort(-p);yy=y[order];ss=p[order];ends=np.r_[np.where(ss[:-1]!=ss[1:])[0],len(ss)-1];tp=np.cumsum(yy)[ends];recall=tp/y.sum();return float(np.sum(np.diff(np.r_[0,recall])*tp/(ends+1)))
print('DATA',json.dumps({s:{'n':len(i),'positive':int(y[i].sum())} for s,i in ix.items()}),flush=True)
for step in range(1001):
 loss=torch.nn.functional.binary_cross_entropy_with_logits(net(X[ix['train']]).flatten(),Y[ix['train']],pos_weight=torch.tensor(weight));opt.zero_grad();loss.backward();opt.step()
 if step%10==0:
  p=predict(ix['validation']);brier=float(np.mean((p-y[ix['validation']])**2));ap=average_precision(p,y[ix['validation']]);trace.append({'step':step,'validation_brier':brier,'validation_ap':ap});print(step,brier,flush=True)
  if ap>best:
   best=ap;stale=0;torch.save({'weights':net.state_dict(),'mean':mean,'scale':scale,'step':step,'input_contract':'v197_replicate64_image_only_contour_cells'},out/'model.pt')
  else:stale+=1
  if stale>=10:break
saved=torch.load(out/'model.pt',weights_only=False);net.load_state_dict(saved['weights']);v=predict(ix['validation']);choices=[float(t) for t in np.linspace(.5,.99,50) if (v>=t).sum()>=10 and y[ix['validation']][v>=t].mean()>=.95];threshold=min(choices) if choices else None
metrics={}
for s,i in ix.items():
 p=predict(i);mask=p>=threshold if threshold is not None else np.zeros(len(i),bool);metrics[s]={'n':len(i),'positive_labels':int(y[i].sum()),'brier':float(np.mean((p-y[i])**2)),'predicted_positive':int(mask.sum()),'precision':float(y[i][mask].mean()) if mask.any() else None,'recall':float(y[i][mask].sum()/y[i].sum()),'constant_prevalence_brier':float(np.mean((y[ix['train']].mean()-y[i])**2))}
r={'complete':True,'completed_steps':step+1,'selected_step':saved['step'],'threshold':threshold,'metrics':metrics,'train_positive_weight':weight,'checkpoint_selection':'Validation average precision, patience10 checks; Same seed/data/optimizer/AP selection as V222; positive weight changed from8.2413 to1.0. Diagnostic ablation, fresh holdout required.','runtime_deployed':False,'prior_evaluation_data_used_for_training':False,'features':'Unchanged image-only V207 features and V209 proposals; decoder used only as labels. Train-only normalization.','limitations':'Proposal crops correlated by frame/scene. Small number of tag identities/backgrounds, disjoint between training and validation; fresh holdout required.','manifest_sha256':hashlib.sha256((src/'manifest.json').read_bytes()).hexdigest()}
(out/'evaluation.json').write_text(json.dumps(r,indent=2)+'\n');(out/'trace.json').write_text(json.dumps(trace,indent=2)+'\n');(out/'proposal_manifest.json').write_text(json.dumps(meta)+'\n');print('COMPLETE',json.dumps(r),flush=True)
