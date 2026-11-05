"""Mount-independent synthetic tag-readability prediction; never identity inference."""
import json,time
from pathlib import Path
import cv2,numpy as np,torch
root=Path(__file__).resolve().parents[1];out=root.parent.parent/'outputs/r12/tag_quality_v184';out.mkdir(exist_ok=False);rng=np.random.default_rng(184);torch.manual_seed(184);torch.set_num_threads(2)
detector=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11));textures=[cv2.imread(str(root/f'models/navigation/textures/tag36h11-{i}.png'),0) for i in range(12)];assert all(x is not None for x in textures)
(out/'experiment.json').write_text(json.dumps(dict(seed=184,task='Predict probability standard AprilTag decoder returns correct ID from a 64x64 crop',train_tag_ids=list(range(8)),validation_tag_ids=[8,9],test_tag_ids=[10,11],samples_per_id=600,negative_samples_per_split_group=600,negative_seed_formula="184000+group",audit_images_used=False,scope='Synthetic transformed tags plus procedurally generated tag-absent stripes, edges and rectangles. Independent of arm mounting, not validated on real camera crops. Quality score never supplies object identity, pose or control authorization.',existing_yolo_centerpose_changed=False),indent=2)+'\n')
# Reuse immutable synthetic tag samples; never load archived audit images.
with np.load(root.parent.parent/'outputs/r12/tag_quality_v180/dataset.npz') as data:
 images=list(data['images']);ys=list(data['readable']);groups=list(data['tag_id']);sizes=list(data['size_px'])
negative_types=[]
for group in range(12):
 nrng=np.random.default_rng(184000+group)
 for k in range(600):
  kind=k%3
  image=np.full((64,64),int(nrng.integers(20,240)),np.uint8)
  if kind==0:
   # Linear stripe patches emulate tag-absent barcode-like distractors.
   image[:]=int(nrng.integers(180,256));x=int(nrng.integers(0,10));top=int(nrng.integers(0,20));bottom=int(nrng.integers(40,65))
   while x<64:
    width=int(nrng.integers(1,5));image[top:bottom,x:x+width]=int(nrng.integers(0,80));x+=width+int(nrng.integers(1,5))
  elif kind==1:
   for _ in range(int(nrng.integers(1,8))):
    a=tuple(int(v) for v in nrng.integers(-20,85,2));bb=tuple(int(v) for v in nrng.integers(-20,85,2));cv2.line(image,a,bb,int(nrng.integers(0,256)),int(nrng.integers(1,10)))
  else:
   for _ in range(int(nrng.integers(1,8))):
    a=tuple(int(v) for v in nrng.integers(0,64,2));bb=tuple(int(v) for v in nrng.integers(0,64,2));cv2.rectangle(image,a,bb,int(nrng.integers(0,256)),-1)
  sigma=float(nrng.uniform(0,2))
  if sigma>.15:image=cv2.GaussianBlur(image,(7,7),sigma)
  image=np.clip(image.astype(float)+nrng.normal(0,nrng.uniform(0,15),image.shape),0,255).astype(np.uint8)
  _,decoded,_=detector.detectMarkers(image)
  if decoded is not None:raise RuntimeError('Unexpected decodable generated negative; inspect instead of mislabeling')
  images.append(image);ys.append(0);groups.append(group);sizes.append(0);negative_types.append(kind)
 print('negative_group',group,'samples',len(images),flush=True)
images=np.array(images);ys=np.array(ys);groups=np.array(groups);sizes=np.array(sizes);train=np.flatnonzero(groups<8);val=np.flatnonzero((groups==8)|(groups==9));test=np.flatnonzero(groups>=10)
np.savez_compressed(out/'dataset.npz',images=images,readable=ys,tag_id=groups,size_px=sizes,train=train,validation=val,test=test)
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
r={'complete':True,'selected_step':d['step'],'train_count':len(train),'validation_count':len(val),'test_count':len(test),'test_readable_fraction':float(ys[test].mean()),'test_brier':float(np.mean((pt-ys[test])**2)),'constant_training_prior_test_brier':float(np.mean((ys[train].mean()-ys[test])**2)),'validation_selected_threshold':threshold,'test_predicted_readable_count':int(selected.sum()),'test_precision':float(ys[test][selected].mean()) if selected.any() else None,'test_recall':float((ys[test]*selected).sum()/ys[test].sum()),'negative_test_count':int((sizes[test]==0).sum()),'negative_test_false_positive_count':int(selected[sizes[test]==0].sum()),'runtime_deployed':False,'scope':'Held-out tag identities10/11, synthetic transformations. Does not establish real-camera performance, geometric reachability or grasp success. Do not use predictions as decoded IDs.'}
(out/'evaluation.json').write_text(json.dumps(r,indent=2)+'\n');print('COMPLETE',json.dumps(r),flush=True)
