"""Evaluate frozen readability model on timestamp-matched archived render crops."""
import json,gzip
from pathlib import Path
import cv2,numpy as np,torch
root=Path(__file__).resolve().parents[1];base=root.parent.parent/'outputs/r12';run=root/'reports/local/terminal_observation_v166';bank=base/'tag_quality_v184';torch.set_num_threads(2)
net=torch.nn.Sequential(torch.nn.Conv2d(1,8,5,stride=2,padding=2),torch.nn.ReLU(),torch.nn.Conv2d(8,16,3,stride=2,padding=1),torch.nn.ReLU(),torch.nn.Conv2d(16,24,3,stride=2,padding=1),torch.nn.ReLU(),torch.nn.Flatten(),torch.nn.Linear(24*8*8,32),torch.nn.ReLU(),torch.nn.Linear(32,1));net.load_state_dict(torch.load(bank/'model.pt',weights_only=False)['weights']);net.eval();threshold=json.loads((bank/'evaluation.json').read_text())['validation_selected_threshold'];detector=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11));perception={}
for line in (run/'perception.jsonl').open():
 p=json.loads(line);perception[(p['stamp_ns'],p['camera_frame'])]=p
rows=[];matched=0
for file in sorted((run/'observation_history').glob('*.npz.gz')):
 with gzip.open(file,'rb') as stream, np.load(stream) as d:
  stamp=int(d['stamp_ns'])
  for camera in ['r07_camera1','r07_camera2','r07_camera3','arm_camera']:
   p=perception.get((stamp,camera+'_left_optical'))
   if p is None:continue
   matched+=1;rgb=d['rgb'][0] if camera=='r07_camera1' else d[camera+'_left_optical'];gray=cv2.cvtColor(rgb[:,:,:3],cv2.COLOR_RGB2GRAY)
   for det in p['detections']:
    if det['class_name']!='apriltag':continue
    x1,y1,x2,y2=det['bbox'];pad=.25*max(x2-x1,y2-y1);left=max(0,int(x1-pad));right=min(gray.shape[1],int(np.ceil(x2+pad)));top=max(0,int(y1-pad));bottom=min(gray.shape[0],int(np.ceil(y2+pad)));crop=gray[top:bottom,left:right]
    if min(crop.shape)<2:continue
    _,raw_ids,_=detector.detectMarkers(crop)
    factor=min(1.,60/max(crop.shape));small=cv2.resize(crop,(max(1,round(crop.shape[1]*factor)),max(1,round(crop.shape[0]*factor))),interpolation=cv2.INTER_AREA);image=np.full((64,64),255,np.uint8);yy=(64-small.shape[0])//2;xx=(64-small.shape[1])//2;image[yy:yy+small.shape[0],xx:xx+small.shape[1]]=small;_,ids,_=detector.detectMarkers(image)
    with torch.no_grad():score=float(net(torch.tensor(image[None,None]/255,dtype=torch.float32)).sigmoid().item())
    rows.append(dict(stamp_ns=stamp,camera=camera,box=det['bbox'],native_crop_shape=list(crop.shape),score=score,predicted_readable=score>=threshold,decoder_readable_64=ids is not None,native_decoder_readable=raw_ids is not None))
y=np.array([r['decoder_readable_64'] for r in rows]);pr=np.array([r['score'] for r in rows]);selected=pr>=threshold
result=dict(complete=True,source_run=run.name,matched_camera_frames=matched,crops=len(rows),threshold=threshold,decoder_positive=int(y.sum()),native_decoder_positive=sum(r['native_decoder_readable'] for r in rows),predicted_positive=int(selected.sum()),false_positive=int((selected & ~y).sum()),precision=float(y[selected].mean()) if selected.any() else None,recall=float(selected[y].mean()) if y.any() else None,brier=float(np.mean((pr-y)**2)) if len(rows) else None,scope='Previously unused V166 run, same simulated scene; temporal holdout, not independent scene or real hardware validation. Detector crop proposals timestamp-matched to saved frames. Label is standard decoder output on model input, not verified true ID. Native decode separately reported. No training on this audit set, no runtime deployment.',rows=rows)
(base/'v185_tag_quality_archive_evaluation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='rows'},indent=2))
