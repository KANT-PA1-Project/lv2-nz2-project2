"""Mount-independent synthetic tag-readability prediction; never identity inference."""
import json,time
from pathlib import Path
import cv2,numpy as np,torch
root=Path(__file__).resolve().parents[1];out=root.parent.parent/'outputs/r12/tag_quality_v180';out.mkdir(exist_ok=False);rng=np.random.default_rng(180);torch.manual_seed(180);torch.set_num_threads(2)
detector=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11));textures=[cv2.imread(str(root/f'models/navigation/textures/tag36h11-{i}.png'),0) for i in range(12)];assert all(x is not None for x in textures)
(out/'experiment.json').write_text(json.dumps(dict(seed=180,task='Predict probability standard AprilTag decoder returns correct ID from a 64x64 crop',train_tag_ids=list(range(8)),validation_tag_ids=[8,9],test_tag_ids=[10,11],samples_per_id=600,scope='Synthetic camera-space blur/resolution/contrast/perspective/noise/JPEG perturbations only. Independent of arm mounting, not validated on real camera crops. Quality score never supplies object identity, pose or control authorization.',existing_yolo_centerpose_changed=False),indent=2)+'\n')
images=[];ys=[];groups=[];sizes=[]
for tag,tex in enumerate(textures):
 for k in range(600):
  canvas=np.full((64,64),rng.integers(150,256),np.uint8);size=int(rng.integers(6,53));x=int(rng.integers(2,63-size));y=int(rng.integers(2,63-size));src=np.float32([[0,0],[tex.shape[1]-1,0],[tex.shape[1]-1,tex.shape[0]-1],[0,tex.shape[0]-1]]);dst=np.float32([[x,y],[x+size,y],[x+size,y+size],[x,y+size]])+rng.uniform(-size*.12,size*.12,(4,2)).astype(np.float32);M=cv2.getPerspectiveTransform(src,dst);warped=cv2.warpPerspective(tex,M,(64,64),borderValue=int(canvas[0,0]));image=warped.astype(float);contrast=rng.uniform(.2,1.);image=127+(image-127)*contrast+rng.uniform(-45,45);sigma=rng.uniform(0,1.8)
  if sigma>.15:image=cv2.GaussianBlur(image,(7,7),sigma)
  image=np.clip(image+rng.normal(0,rng.uniform(0,12),image.shape),0,255).astype(np.uint8);ok,encoded=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,int(rng.integers(30,101))]);image=cv2.imdecode(encoded,0)
  _,ids,_=detector.detectMarkers(image);label=ids is not None and tag in ids.flatten();images.append(image);ys.append(int(label));groups.append(tag);sizes.append(size)
 print('generated_tag',tag,'samples',len(images),'readable',sum(ys),flush=True)
images=np.array(images);ys=np.array(ys);groups=np.array(groups);sizes=np.array(sizes);train=np.flatnonzero(groups<8);val=np.flatnonzero((groups==8)|(groups==9));test=np.flatnonzero(groups>=10);assert all(len(np.unique(ys[ix]))==2 for ix in [train,val,test]);np.savez_compressed(out/'dataset.npz',images=images,readable=ys,tag_id=groups,size_px=sizes,train=train,validation=val,test=test)
X=torch.tensor(images[:,None]/255.,dtype=torch.float32);Y=torch.tensor(ys,dtype=torch.float32);net=torch.nn.Sequential(torch.nn.Conv2d(1,8,5,stride=2,padding=2),torch.nn.ReLU(),torch.nn.Conv2d(8,16,3,stride=2,padding=1),torch.nn.ReLU(),torch.nn.Conv2d(16,24,3,stride=2,padding=1),torch.nn.ReLU(),torch.nn.Flatten(),torch.nn.Linear(24*8*8,32),torch.nn.ReLU(),torch.nn.Linear(32,1));opt=torch.optim.Adam(net.parameters(),lr=.001);best=float('inf')
def predict(ix):
 with torch.no_grad():return torch.cat([net(X[batch]).flatten().sigmoid() for batch in np.array_split(ix,max(1,len(ix)//128))]).numpy()
for step in range(1201):
 ix=rng.choice(train,64);logits=net(X[ix]).flatten();loss=torch.nn.functional.binary_cross_entropy_with_logits(logits,Y[ix]);opt.zero_grad();loss.backward();opt.step()
 if step%100==0:
  pred=predict(val);brier=float(np.mean((pred-ys[val])**2));print('step',step,'validation_brier',brier,flush=True)
  if brier<best:best=brier;torch.save(dict(weights=net.state_dict(),step=step,task='readability not identity'),out/'model.pt')
d=torch.load(out/'model.pt',weights_only=False);net.load_state_dict(d['weights']);pv=predict(val);pt=predict(test)
# Set threshold on validation only: prioritize precision at minimum 30 predicted positives.
choices=[]
for threshold in np.linspace(.5,.99,50):
 selected=pv>=threshold
 if selected.sum()>=30 and ys[val][selected].mean()>=.95:choices.append(float(threshold))
threshold=min(choices) if choices else None
selected=pt>=threshold if threshold is not None else np.zeros(len(test),bool)
r={'complete':True,'selected_step':d['step'],'train_count':len(train),'validation_count':len(val),'test_count':len(test),'test_readable_fraction':float(ys[test].mean()),'test_brier':float(np.mean((pt-ys[test])**2)),'constant_training_prior_test_brier':float(np.mean((ys[train].mean()-ys[test])**2)),'validation_selected_threshold':threshold,'test_predicted_readable_count':int(selected.sum()),'test_precision':float(ys[test][selected].mean()) if selected.any() else None,'test_recall':float((ys[test]*selected).sum()/ys[test].sum()),'runtime_deployed':False,'scope':'Held-out tag identities10/11, synthetic transformations. Does not establish real-camera performance, geometric reachability or grasp success. Do not use predictions as decoded IDs.'}
(out/'evaluation.json').write_text(json.dumps(r,indent=2)+'\n');print('COMPLETE',json.dumps(r),flush=True)
