"""Read-only train/validation diagnosis; excludes test images and labels."""
from pathlib import Path
import json,cv2,numpy as np,torch
root=Path(__file__).resolve().parents[1];b=root.parent.parent/'outputs/r12';src=b/'tag_render_v201';rows=[r for r in json.loads((src/'manifest.json').read_text())['rows'] if r['split'] in ['train','validation']];torch.set_num_threads(2)
X=torch.tensor(np.array([cv2.imread(str(src/f'replicate_{r["index"]:03d}.png'),0) for r in rows])[:,None]/255,dtype=torch.float32);y=np.array([r['tag_id'] in r['decoder_variants']['input64_replicate'] for r in rows]);net=torch.nn.Sequential(torch.nn.Conv2d(1,8,5,stride=2,padding=2),torch.nn.ReLU(),torch.nn.Conv2d(8,16,3,stride=2,padding=1),torch.nn.ReLU(),torch.nn.Conv2d(16,24,3,stride=2,padding=1),torch.nn.ReLU(),torch.nn.Flatten(),torch.nn.Linear(24*8*8,32),torch.nn.ReLU(),torch.nn.Linear(32,1));net.load_state_dict(torch.load(b/'tag_quality_v202/model.pt',weights_only=False)['weights']);net.eval()
with torch.no_grad():scores=net(X).flatten().sigmoid().numpy()
report={}
for split in ['train','validation']:
 ix=np.array([i for i,r in enumerate(rows) if r['split']==split]);p=scores[ix];labels=y[ix];rank=ix[np.argsort(-p)];points=[]
 for t in np.unique(p):
  selected=p>=t
  if selected.sum()>=10:points.append({'threshold':float(t),'count':int(selected.sum()),'precision':float(labels[selected].mean())})
 best=max(points,key=lambda r:r['precision']);by_tag={}
 for tag in sorted({rows[i]['tag_id'] for i in ix}):
  ti=np.array([i for i in ix if rows[i]['tag_id']==tag]);by_tag[str(tag)]={'n':len(ti),'decoder_positive':int(y[ti].sum()),'predicted_at_05':int((scores[ti]>=.5).sum()),'brier':float(np.mean((scores[ti]-y[ti])**2))}
 report[split]={'positive_score_quantiles':np.quantile(p[labels],[0,.25,.5,.75,1]).tolist(),'negative_score_quantiles':np.quantile(p[~labels],[0,.25,.5,.75,1]).tolist(),'best_precision_over_all_observed_thresholds_min10':best,'by_tag':by_tag,'top10':[{'index':rows[i]['index'],'tag_id':rows[i]['tag_id'],'distance_m':rows[i]['distance_m'],'score':float(scores[i]),'decoder_success':bool(y[i])} for i in rank[:10]]}
trace=json.loads((b/'tag_quality_v202/trace.json').read_text());r={'complete':True,'train_validation_samples':len(rows),'test_images_read':False,'partitions':report,'best_validation_step':min(trace,key=lambda x:x['validation_brier']),'last_validation_step':trace[-1],'scope':'Posthoc diagnosis only. No new threshold selection or deployment; original95%precision/min10 criterion unchanged.'};(b/'v203_overlap_diagnosis.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
