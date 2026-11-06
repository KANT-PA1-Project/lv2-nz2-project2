from pathlib import Path
import ast,json,time,cv2,numpy as np,torch
root=Path(__file__).resolve().parents[1];b=root.parent.parent/'outputs/r12';src=b/'tag_render_v230';torch.set_num_threads(2);cv2.setNumThreads(1)
for filename,fn in [('train_r12_tag_quality_v207.py','features'),('evaluate_r12_proposals_v209.py','proposals')]:
 t=ast.parse((root/'scripts'/filename).read_text());f=next(n for n in t.body if isinstance(n,ast.FunctionDef) and n.name==fn);exec(compile(ast.Module(body=[f],type_ignores=[]),filename,'exec'))
base_features=features
t=ast.parse((root/'scripts/train_r12_tag_quality_v229.py').read_text());f=next(n for n in t.body if isinstance(n,ast.FunctionDef) and n.name=='features');exec(compile(ast.Module(body=[f],type_ignores=[]),'v229','exec'))
saved=torch.load(b/'tag_quality_v229/model.pt',weights_only=False);net=torch.nn.Sequential(torch.nn.Linear(139,16),torch.nn.ReLU(),torch.nn.Linear(16,1));net.load_state_dict(saved['weights']);net.eval();det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11))
rows=json.loads((src/'manifest.json').read_text())['rows'];images=[cv2.imread(str(src/f'frame_{r["index"]:03d}.png'),0) for r in rows]
def decode(im):
 _,ids,_=det.detectMarkers(im);return set() if ids is None else set(map(int,ids.flatten()))
def run(im,mode):
 if mode=='full_frame':return decode(im)
 _,crops=proposals(im)
 if mode=='learned_gate' and crops:
  ff=np.array([features(c)[0] for c in crops]);
  with torch.no_grad():scores=net(torch.tensor((ff-saved['mean'])/saved['scale'])).flatten().sigmoid().numpy()
  crops=[c for c,s in zip(crops,scores) if s>=.5]
 ids=set()
 for c in crops:ids|=decode(c)
 return ids
modes=['full_frame','all_proposals','learned_gate'];times={m:[] for m in modes};results={m:[] for m in modes}
for im in images[:3]:
 for m in modes:run(im,m)
for rep in range(3):
 for i,im in enumerate(images):
  for m in modes[rep:]+modes[:rep]:
   start=time.perf_counter_ns();ids=run(im,m);times[m].append((time.perf_counter_ns()-start)/1e6)
   if rep==0:results[m].append(sorted(ids))
summary={}
for m in modes:
 summary[m]={'median_ms':float(np.median(times[m])),'p95_ms':float(np.quantile(times[m],.95)),'mean_ms':float(np.mean(times[m])),'correct_frames':sum(r['tag_id']>=0 and r['tag_id'] in ids for r,ids in zip(rows,results[m])),'negative_frames_with_ids':sum(r['tag_id']<0 and bool(ids) for r,ids in zip(rows,results[m])),'frames_with_wrong_ids':sum(any(i!=r['tag_id'] for i in ids) for r,ids in zip(rows,results[m]))}
a=json.loads((b/'v230_evaluation_225.json').read_text());c=json.loads((b/'v230_evaluation_229.json').read_text())
def errors(r):return {(x['index'],tuple(p['box'])) for x in r['records'] for p in x['proposals'] if p['score']>=r['threshold'] and not p['label']}
report={'complete':True,'frames':144,'repeats':3,'summary':summary,'shared_false_accepts':len(errors(a)&errors(c)),'scope':'Local CPU microbenchmark, preloaded grayscale frames, warmup3 frames, OpenCV1thread/Torch2threads; proposal and feature costs included. Learned gate batched per frame then actual decoder confirms IDs. No simulator/render/IO/ROS load included; not real-time or hardware certification. Consumed V230 data, no training/tuning.','decoded_ids':results};(b/'v231_runtime_benchmark.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k!='decoded_ids'},indent=2))
