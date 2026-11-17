"""Image-only contour/cell feature ablation. No decoder outputs in features."""
from pathlib import Path
import json,hashlib,cv2,numpy as np,torch
base=Path(__file__).resolve().parents[3]/'outputs/r12';src=base/'tag_render_v201';out=base/'tag_quality_v207';out.mkdir(exist_ok=False)
torch.set_num_threads(2);torch.manual_seed(207)
def features(im):
 threshold,binary=cv2.threshold(im,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
 contours,_=cv2.findContours(binary,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE);quads=[]
 for c in contours:
  q=cv2.approxPolyDP(c,.03*cv2.arcLength(c,True),True)
  if len(q)==4 and cv2.isContourConvex(q) and cv2.contourArea(q)>=100:quads.append(q.reshape(4,2).astype(np.float32))
 if not quads:return np.zeros(133,np.float32),False
 q=max(quads,key=cv2.contourArea);center=q.mean(0);q=q[np.argsort(np.arctan2(q[:,1]-center[1],q[:,0]-center[0]))];q=np.roll(q,-np.argmin(q.sum(1)),axis=0)
 target=np.array([[0,0],[63,0],[63,63],[0,63]],np.float32);warp=cv2.warpPerspective(im,cv2.getPerspectiveTransform(q,target),(64,64))
 cells=np.array([warp[y*8+2:y*8+6,x*8+2:x*8+6] for y in range(8) for x in range(8)])
 means=cells.mean((1,2))/255;std=cells.std((1,2))/255
 return np.r_[1,cv2.contourArea(q)/4096,im.std()/255,threshold/255,np.min(np.linalg.norm(q-np.roll(q,1,axis=0),axis=1))/64,means,std].astype(np.float32),True
rows=[r for r in json.loads((src/'manifest.json').read_text())['rows'] if r['split']!='test'];features_list=[];found=[]
for r in rows:
 im=cv2.imread(str(src/f'replicate_{r["index"]:03d}.png'),0);assert im is not None
 f,ok=features(im);features_list.append(f);found.append(ok)
x=np.array(features_list);y=np.array([r['tag_id'] in r['decoder_variants']['input64_replicate'] for r in rows],np.float32)
ix={s:np.array([i for i,r in enumerate(rows) if r['split']==s]) for s in ['train','validation']}
mean=x[ix['train']].mean(0);scale=np.maximum(x[ix['train']].std(0),.01);X=torch.tensor((x-mean)/scale);Y=torch.tensor(y)
net=torch.nn.Sequential(torch.nn.Linear(133,16),torch.nn.ReLU(),torch.nn.Linear(16,1));opt=torch.optim.AdamW(net.parameters(),lr=.001,weight_decay=.01)
def predict(i):
 with torch.no_grad():return net(X[i]).flatten().sigmoid().numpy()
best=float('inf');trace=[];stale=0
for step in range(601):
 loss=torch.nn.functional.binary_cross_entropy_with_logits(net(X[ix['train']]).flatten(),Y[ix['train']]);opt.zero_grad();loss.backward();opt.step()
 if step%10==0:
  p=predict(ix['validation']);brier=float(np.mean((p-y[ix['validation']])**2));trace.append({'step':step,'validation_brier':brier});print(step,brier,flush=True)
  if brier<best:
   best=brier;stale=0;torch.save({'weights':net.state_dict(),'mean':mean,'scale':scale,'step':step,'input_contract':'v197_replicate64_image_only_contour_cells'},out/'model.pt')
  else:stale+=1
  if stale>=10:break
saved=torch.load(out/'model.pt',weights_only=False);net.load_state_dict(saved['weights']);v=predict(ix['validation']);choices=[float(t) for t in np.linspace(.5,.99,50) if (v>=t).sum()>=10 and y[ix['validation']][v>=t].mean()>=.95];threshold=min(choices) if choices else None
metrics={}
for s,i in ix.items():
 p=predict(i);mask=p>=threshold if threshold is not None else np.zeros(len(i),bool)
 metrics[s]={'n':len(i),'positive_labels':int(y[i].sum()),'quad_found':int(np.array(found)[i].sum()),'brier':float(np.mean((p-y[i])**2)),'predicted_positive':int(mask.sum()),'precision':float(y[i][mask].mean()) if mask.any() else None,'recall':float(y[i][mask].sum()/y[i].sum())}
report={'complete':True,'completed_steps':step+1,'selected_step':saved['step'],'threshold':threshold,'metrics':metrics,'test_images_read':False,'runtime_deployed':False,'features':'Otsu contours, largest convex quadrilateral; image-only warp and 8x8 cell means/std. No dictionary, ID, distance, fixture corners, decoder output or rejected candidates in features. Labels only use saved decoder success.','limitations':'Reused development train/validation, small controlled fixture dataset. Largest quad is not guaranteed true tag. Fresh holdout and runtime proposals required before qualification.','manifest_sha256':hashlib.sha256((src/'manifest.json').read_bytes()).hexdigest()}
(out/'evaluation.json').write_text(json.dumps(report,indent=2)+'\n');(out/'trace.json').write_text(json.dumps(trace,indent=2)+'\n');print('COMPLETE',json.dumps(report),flush=True)
