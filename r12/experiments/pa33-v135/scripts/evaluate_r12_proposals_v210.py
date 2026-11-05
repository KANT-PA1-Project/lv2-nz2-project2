"""Posthoc full-frame proposal diagnostic. No geometry/identity in proposal function."""
from pathlib import Path
import ast,json,hashlib,cv2,numpy as np,torch
root=Path(__file__).resolve().parents[1];base=root.parent.parent/'outputs/r12';src=base/'tag_render_v210';plan=json.loads((base/'v210_acquisition_plan.json').read_text());model=base/'tag_quality_v207/model.pt'
assert hashlib.sha256(model.read_bytes()).hexdigest()==plan['model_sha256']
tree=ast.parse((root/'scripts/train_r12_tag_quality_v207.py').read_text());f=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='features');exec(compile(ast.Module(body=[f],type_ignores=[]),'v207_features','exec'))
net=torch.nn.Sequential(torch.nn.Linear(133,16),torch.nn.ReLU(),torch.nn.Linear(16,1));saved=torch.load(model,weights_only=False);net.load_state_dict(saved['weights']);net.eval();torch.set_num_threads(2)
def proposals(im):
 _,bw=cv2.threshold(im,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU);cc,_=cv2.findContours(bw,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE);boxes=[]
 for c in cc:
  q=cv2.approxPolyDP(c,.03*cv2.arcLength(c,True),True);area=cv2.contourArea(q)
  if len(q)!=4 or not cv2.isContourConvex(q) or not 25<=area<=im.size*.25:continue
  x,y,w,h=cv2.boundingRect(q)
  if min(w,h)<5 or max(w,h)/min(w,h)>3:continue
  boxes.append((x,y,w,h))
 kept=[]
 for box in sorted(set(boxes),key=lambda b:b[2]*b[3],reverse=True):
  x,y,w,h=box
  if any(max(0,min(x+w,a+c)-max(x,a))*max(0,min(y+h,b+d)-max(y,b))/(w*h+c*d-max(0,min(x+w,a+c)-max(x,a))*max(0,min(y+h,b+d)-max(y,b)))>.5 for a,b,c,d in kept):continue
  kept.append(box)
 crops=[]
 for x,y,w,h in kept:
  pad=int(np.ceil(max(w,h)*.25));crop=im[max(0,y-pad):min(im.shape[0],y+h+pad),max(0,x-pad):min(im.shape[1],x+w+pad)];scale=min(1,60/max(crop.shape));small=cv2.resize(crop,(round(crop.shape[1]*scale),round(crop.shape[0]*scale)),interpolation=cv2.INTER_AREA);hh,ww=small.shape;crops.append(cv2.copyMakeBorder(small,(64-hh)//2,64-hh-(64-hh)//2,(64-ww)//2,64-ww-(64-ww)//2,cv2.BORDER_REPLICATE))
 return kept,crops
rows=json.loads((src/'manifest.json').read_text())['rows'];det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11));records=[]
for r in rows:
 im=cv2.imread(str(src/f'frame_{r["index"]:03d}.png'),0);boxes,crops=proposals(im);rr=[]
 for box,crop in zip(boxes,crops):
  feat,_=features(crop)
  with torch.no_grad():score=float(net(torch.tensor((feat-saved['mean'])/saved['scale'])).sigmoid().item())
  _,ids,_=det.detectMarkers(crop);decoded=[] if ids is None else ids.flatten().tolist()
  rr.append({'box':box,'score':score,'decoded':decoded,'label':r['tag_id']>=0 and r['tag_id'] in decoded})
 records.append({'index':r['index'],'tag_id':r['tag_id'],'proposals':rr})
flat=[p for r in records for p in r['proposals']];accepted=[p for p in flat if p['score']>=plan['threshold']];tp=sum(p['label'] for p in accepted);fp=len(accepted)-tp
report={'complete':True,'frames':len(rows),'proposals':len(flat),'accepted':len(accepted),'tp':tp,'fp':fp,'precision':tp/len(accepted) if accepted else None,'frames_with_readable_proposal':sum(any(p['label'] for p in r['proposals']) for r in records),'frames_with_accepted_readable_proposal':sum(any(p['label'] and p['score']>=plan['threshold'] for p in r['proposals']) for r in records),'negative_frames_with_acceptance':sum(r['tag_id']<0 and any(p['score']>=plan['threshold'] for p in r['proposals']) for r in records),'threshold':plan['threshold'],'model_sha256':plan['model_sha256'],'runtime_deployed':False,'scope':'Fresh IDs/seeds/off-center rendered backgrounds; same renderer, not mission/hardware validation. Image-only Otsu quadrilateral proposals, IoU0.5 suppression,25% padding,replicate64. No fixture coordinates/distance/ID in proposals or features. Decoder used only afterward for evaluation labels. Proposal labels differ from original fixture crops; no direct recall comparison.','records':records}
(base/'v210_proposal_evaluation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k!='records'}))
