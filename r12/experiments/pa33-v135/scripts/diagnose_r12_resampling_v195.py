"""Paired input-stage audit; diagnostic variants do not change deployed labels."""
from pathlib import Path
import json,cv2,numpy as np
root=Path(__file__).resolve().parents[1];base=root.parent.parent/'outputs/r12';src=base/'tag_render_v194';rows=json.loads((src/'manifest.json').read_text())['rows']
d=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11));results=[]
for r in rows:
 gray=cv2.cvtColor(cv2.imread(str(src/f'frame_{r["index"]:03d}.png'),cv2.IMREAD_COLOR),cv2.COLOR_BGR2GRAY);half=int(np.ceil(400*.01/r['distance_m']*1.5));crop=gray[240-half:240+half,320-half:320+half]
 variants={'native_crop':crop};scale=min(1.,60/max(crop.shape));shape=(round(crop.shape[1]*scale),round(crop.shape[0]*scale))
 for name,method in [('area',cv2.INTER_AREA),('linear',cv2.INTER_LINEAR),('nearest',cv2.INTER_NEAREST),('cubic',cv2.INTER_CUBIC)]:
  small=cv2.resize(crop,shape,interpolation=method);variants[name+'_no_pad']=small
  im=np.full((64,64),255,np.uint8);h,w=small.shape;im[(64-h)//2:(64-h)//2+h,(64-w)//2:(64-w)//2+w]=small;variants[name+'_pad64']=im
 native128=np.full((128,128),255,np.uint8);h,w=crop.shape;assert max(h,w)<=128;native128[(128-h)//2:(128-h)//2+h,(128-w)//2:(128-w)//2+w]=crop;variants['native_pad128']=native128
 values={}
 for name,img in variants.items():
  corners,ids,_=d.detectMarkers(img);ids=[] if ids is None else ids.flatten().tolist();values[name]={'correct':r['tag_id'] in ids,'ids':ids,'shape':list(img.shape)}
  if name=='native_crop' and r['tag_id'] in ids:
   points=corners[ids.index(r['tag_id'])][0];values[name]['black_border_side_px']=float(np.mean(np.linalg.norm(points-np.roll(points,1,axis=0),axis=1)))
 baseline=cv2.imread(str(src/f'crop_{r["index"]:03d}.png'),0)
 assert np.array_equal(variants['area_pad64'],baseline), 'Preprocess reproduction mismatch'
 results.append({'index':r['index'],'distance_m':r['distance_m'],'camera':r['camera'],'variants':values})
summary={}
for distance in sorted({x['distance_m'] for x in results}):
 group=[x for x in results if x['distance_m']==distance];summary[str(distance)]={name:sum(x['variants'][name]['correct'] for x in group) for name in group[0]['variants']}
report={'samples':len(results),'baseline_pixel_exact':True,'by_distance_correct_counts':summary,'rows':results,'scope':'Posthoc paired diagnostic on controlled static renders. Variants are not trained, deployed, or held-out acceptance results. Native64 target is unchanged.'}
(base/'v195_resampling_diagnostic.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k!='rows'},indent=2))
