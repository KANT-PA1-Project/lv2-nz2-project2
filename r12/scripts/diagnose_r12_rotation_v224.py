from pathlib import Path
import ast,json,cv2,numpy as np,torch
root=Path(__file__).resolve().parents[1];b=root.parent.parent/'outputs/r12';src=b/'tag_render_v218';torch.set_num_threads(2)
for filename,fn in [('train_r12_tag_quality_v207.py','features'),('evaluate_r12_proposals_v209.py','proposals')]:
 t=ast.parse((root/'scripts'/filename).read_text());f=next(n for n in t.body if isinstance(n,ast.FunctionDef) and n.name==fn);exec(compile(ast.Module(body=[f],type_ignores=[]),filename,'exec'))
det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11));saved=torch.load(b/'tag_quality_v223/model.pt',weights_only=False);net=torch.nn.Sequential(torch.nn.Linear(133,16),torch.nn.ReLU(),torch.nn.Linear(16,1));net.load_state_dict(saved['weights']);net.eval();records=[]
for r in json.loads((src/'manifest.json').read_text())['rows']:
 im=cv2.imread(str(src/f'frame_{r["index"]:03d}.png'),0);boxes,crops=proposals(im)
 for box,crop in zip(boxes,crops):
  ff=[];labels=[]
  for k in range(4):
   rotated=np.ascontiguousarray(np.rot90(crop,k));ff.append(features(rotated)[0]);_,ids,_=det.detectMarkers(rotated);labels.append(r['tag_id']>=0 and ids is not None and r['tag_id'] in ids.flatten())
  with torch.no_grad():scores=net(torch.tensor((np.array(ff)-saved['mean'])/saved['scale'])).flatten().sigmoid().numpy()
  records.append({'frame':r['index'],'split':r['split'],'box':box,'labels':list(map(bool,labels)),'scores':scores.tolist(),'range':float(np.ptp(scores))})
summary={}
for s in ['train','validation']:
 rr=[r for r in records if r['split']==s];stable=[r for r in rr if len(set(r['labels']))==1];pos=[r for r in stable if r['labels'][0]]
 summary[s]={'proposals':len(rr),'decoder_label_changed_under_rotation':sum(len(set(r['labels']))>1 for r in rr),'stable_label_score_range_median_p90_max':np.quantile([r['range'] for r in stable],[.5,.9,1]).tolist(),'stable_positive_count':len(pos),'stable_positive_score_range_median_p90_max':np.quantile([r['range'] for r in pos],[.5,.9,1]).tolist()}
report={'complete':True,'model':'V223','summary':summary,'largest_stable_positive_changes':sorted([r for r in records if all(r['labels'])],key=lambda r:r['range'],reverse=True)[:10],'scope':'Train/validation V218 only. Exact90deg rotations, redecoded labels; not assuming decoder invariance. No test data, training, threshold tuning or deployment.'};(b/'v224_rotation_diagnosis.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(summary,indent=2))
