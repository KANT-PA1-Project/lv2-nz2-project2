"""Paired decoder diagnostics only; original training labels remain unchanged."""
from pathlib import Path
import json,cv2,numpy as np
base=Path(__file__).resolve().parents[3]/'outputs/r12';src=base/'tag_render_v201'
rows=[r for r in json.loads((src/'manifest.json').read_text())['rows'] if r['split']!='test']
det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),cv2.aruco.DetectorParameters())
records=[]
for r in rows:
 im=cv2.imread(str(src/f'replicate_{r["index"]:03d}.png'),0);assert im is not None
 variants={'replicate64':im,'replicate128_nearest':cv2.resize(im,(128,128),interpolation=cv2.INTER_NEAREST),'replicate128_linear':cv2.resize(im,(128,128),interpolation=cv2.INTER_LINEAR),'replicate256_linear':cv2.resize(im,(256,256),interpolation=cv2.INTER_LINEAR)}
 decoded={}
 for name,img in variants.items():
  _,ids,_=det.detectMarkers(img);decoded[name]=[] if ids is None else ids.flatten().tolist()
 assert decoded['replicate64']==r['decoder_variants']['input64_replicate']
 decoded['native_frame']=r['decoder_variants']['full_frame_native'];decoded['native_crop']=r['decoder_variants']['crop_native']
 records.append({'index':r['index'],'split':r['split'],'tag_id':r['tag_id'],'distance_m':r['distance_m'],'decoded':decoded})
summary={}
for split in ['train','validation']:
 rr=[r for r in records if r['split']==split];summary[split]={}
 for name in records[0]['decoded']:
  positives=[r for r in rr if r['tag_id']>=0];neg=[r for r in rr if r['tag_id']<0]
  summary[split][name]={'correct':sum(r['tag_id'] in r['decoded'][name] for r in positives),'positive_samples':len(positives),'negative_false_decodes':sum(bool(r['decoded'][name]) for r in neg),'negative_samples':len(neg),'recovered_from_64':sum(r['tag_id'] not in r['decoded']['replicate64'] and r['tag_id'] in r['decoded'][name] for r in positives),'lost_from_64':sum(r['tag_id'] in r['decoded']['replicate64'] and r['tag_id'] not in r['decoded'][name] for r in positives)}
out={'complete':True,'samples':len(rows),'test_images_read':False,'summary':summary,'records':records,'scope':'Posthoc diagnostic on reused train/validation. Upsampling adds no optical information; target labels, trained models and runtime unchanged. Fixture crop is dataset-only, not control observation.'}
(base/'v205_resolution_diagnosis.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(summary,indent=2))
