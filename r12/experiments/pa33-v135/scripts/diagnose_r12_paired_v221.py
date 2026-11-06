from pathlib import Path
import json,numpy as np
b=Path(__file__).resolve().parents[3]/'outputs/r12';runs={v:json.loads((b/f'v220_evaluation_{v}.json').read_text()) for v in [215,219]}
def flatten(r):return {(x['index'],tuple(p['box'])):p for x in r['records'] for p in x['proposals']}
f={v:flatten(r) for v,r in runs.items()};keys=list(f[215]);assert set(keys)==set(f[219]);assert all(f[215][k]['label']==f[219][k]['label'] for k in keys)
y=np.array([f[215][k]['label'] for k in keys]);scores={v:np.array([f[v][k]['score'] for k in keys]) for v in runs};metrics={}
for v,s in scores.items():
 pos=s[y];neg=s[~y];auc=float(np.mean((pos[:,None]>neg[None,:])+.5*(pos[:,None]==neg[None,:])))
 # AP grouped by equal scores, avoiding arbitrary tie ordering.
 order=np.argsort(-s);yy=y[order];ss=s[order];ends=np.r_[np.where(ss[:-1]!=ss[1:])[0],len(ss)-1];tp=np.cumsum(yy)[ends];recall=tp/y.sum();precision=tp/(ends+1);ap=float(np.sum(np.diff(np.r_[0,recall])*precision))
 metrics[str(v)]={'roc_auc':auc,'average_precision':ap,'brier':float(np.mean((s-y)**2)),'positive_score_quantiles':np.quantile(pos,[0,.25,.5,.75,1]).tolist(),'negative_score_quantiles':np.quantile(neg,[0,.25,.5,.75,1]).tolist()}
a=scores[215]>=runs[215]['threshold'];c=scores[219]>=runs[219]['threshold'];groups={}
for label in [True,False]:
 m=y==label;groups[str(label)]={'both_accept':int((m&a&c).sum()),'only_v215':int((m&a&~c).sum()),'only_v219':int((m&~a&c).sum()),'both_reject':int((m&~a&~c).sum())}
fail=[{'frame':k[0],'box':list(k[1]),'v215_score':float(scores[215][i]),'v219_score':float(scores[219][i])} for i,k in enumerate(keys) if c[i] and not y[i]]
r={'complete':True,'proposals':len(keys),'positive_labels':int(y.sum()),'metrics':metrics,'paired_acceptances_by_label':groups,'v219_false_acceptances':fail,'scope':'Consumed V220 diagnostic only. No threshold search, model changes, or deployment. Ranking metrics compare identical proposals; not independent confidence estimates.'};(b/'v221_paired_diagnosis.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2))
