"""Train/validation-only rejected-quadrilateral diagnostic; no relabeling."""
from pathlib import Path
import json,cv2,numpy as np
base=Path(__file__).resolve().parents[3]/'outputs/r12';src=base/'tag_render_v201'
rows=[r for r in json.loads((src/'manifest.json').read_text())['rows'] if r['split']!='test']
det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),cv2.aruco.DetectorParameters())
records=[]
for r in rows:
 im=cv2.imread(str(src/f'replicate_{r["index"]:03d}.png'),0);assert im is not None
 corners,ids,rejected=det.detectMarkers(im);decoded=[] if ids is None else ids.flatten().tolist();assert decoded==r['decoder_variants']['input64_replicate']
 # Center/area are image-space proxies, not ground-truth tag corner matching.
 candidates=[]
 for c in rejected:
  p=c.reshape(4,2);area=abs(float(cv2.contourArea(p)));center=p.mean(0)
  candidates.append({'area_px2':area,'center_offset_px':float(np.linalg.norm(center-np.array([31.5,31.5])))})
 central=[c for c in candidates if c['area_px2']>=100 and c['center_offset_px']<=8]
 records.append({'index':r['index'],'split':r['split'],'tag_id':r['tag_id'],'decoder_success':r['tag_id'] in decoded,'rejected_count':len(rejected),'central_large_rejected_count':len(central),'max_rejected_area_px2':max([c['area_px2'] for c in candidates],default=0),'laplacian_variance':float(cv2.Laplacian(im,cv2.CV_64F).var()),'contrast_std':float(im.std())})
summary={}
for split in ['train','validation']:
 summary[split]={}
 for group in ['success','tag_failure','tag_absent']:
  rr=[r for r in records if r['split']==split and (r['tag_id']<0 if group=='tag_absent' else r['tag_id']>=0 and r['decoder_success']==(group=='success'))]
  summary[split][group]={'n':len(rr),'any_rejected':sum(r['rejected_count']>0 for r in rr),'central_large_rejected':sum(r['central_large_rejected_count']>0 for r in rr),'laplacian_q25_median_q75':np.quantile([r['laplacian_variance'] for r in rr],[.25,.5,.75]).tolist(),'contrast_q25_median_q75':np.quantile([r['contrast_std'] for r in rr],[.25,.5,.75]).tolist()}
report={'complete':True,'samples':len(rows),'test_images_read':False,'summary':summary,'records':records,'scope':'Rejected quadrilaterals are not proven tag boundaries. Central means offset<=8px, large means area>=100px2; diagnostic proxies fixed before analysis. No changes to models, labels or runtime.'}
(base/'v206_candidate_diagnosis.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(summary,indent=2))
