"""Paired padding-only audit of preserved V194 RGB; no training or runtime edit."""
from pathlib import Path
import json,cv2,numpy as np
root=Path(__file__).resolve().parents[1];base=root.parent.parent/'outputs/r12';src=base/'tag_render_v194';rows=json.loads((src/'manifest.json').read_text())['rows'];dic=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11);d=cv2.aruco.ArucoDetector(dic);results=[]
for row in rows:
 gray=cv2.cvtColor(cv2.imread(str(src/f'frame_{row["index"]:03d}.png')),cv2.COLOR_BGR2GRAY);half=int(np.ceil(400*.01/row['distance_m']*1.5));crop=gray[240-half:240+half,320-half:320+half];scale=min(1.,60/max(crop.shape));small=cv2.resize(crop,(round(crop.shape[1]*scale),round(crop.shape[0]*scale)),interpolation=cv2.INTER_AREA);h,w=small.shape;top=(64-h)//2;left=(64-w)//2;border=np.concatenate([small[0],small[-1],small[:,0],small[:,-1]]);median=int(np.median(border));variants={}
 for name,color in [('white',255),('black',0),('gray127',127),('edge_median',median)]:
  variants[name]=cv2.copyMakeBorder(small,top,64-h-top,left,64-w-left,cv2.BORDER_CONSTANT,value=color)
 for name,mode in [('replicate',cv2.BORDER_REPLICATE),('reflect',cv2.BORDER_REFLECT_101)]:variants[name]=cv2.copyMakeBorder(small,top,64-h-top,left,64-w-left,mode)
 assert np.array_equal(variants['white'],cv2.imread(str(src/f'crop_{row["index"]:03d}.png'),0))
 values={}
 for name,im in variants.items():
  _,ids,rejected=d.detectMarkers(im);ids=[] if ids is None else ids.flatten().tolist();values[name]={'ids':ids,'correct':row['tag_id'] in ids,'rejected_quad_count':len(rejected)}
 results.append(dict(index=row['index'],distance_m=row['distance_m'],border_median=median,variants=values))
summary={}
for distance in sorted({r['distance_m'] for r in results}):
 group=[r for r in results if r['distance_m']==distance];summary[str(distance)]={name:sum(r['variants'][name]['correct'] for r in group) for name in group[0]['variants']}
r={'samples':len(results),'baseline_pixel_exact':True,'by_distance_correct':summary,'rows':results,'scope':'Known controlled pilot only. Correct IDs assessed against generated texture solely for diagnostic labels. No change to native64 contract, model, runtime, or threshold. Same centered pixels and canvas size for every variant.'};(base/'v196_padding_diagnostic.json').write_text(json.dumps(r,indent=2));print(json.dumps({k:v for k,v in r.items() if k!='rows'},indent=2))
