from pathlib import Path
import ast,json,cv2,numpy as np
root=Path(__file__).resolve().parents[1];b=root.parent.parent/'outputs/r12';src=b/'tag_render_v218'
for filename,fn in [('train_r12_tag_quality_v207.py','features'),('evaluate_r12_proposals_v209.py','proposals')]:
 t=ast.parse((root/'scripts'/filename).read_text());f=next(n for n in t.body if isinstance(n,ast.FunctionDef) and n.name==fn);exec(compile(ast.Module(body=[f],type_ignores=[]),filename,'exec'))
det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11));records=[]
for r in json.loads((src/'manifest.json').read_text())['rows']:
 im=cv2.imread(str(src/f'frame_{r["index"]:03d}.png'),0);boxes,crops=proposals(im)
 for box,crop in zip(boxes,crops):
  f,ok=features(crop);_,ids,_=det.detectMarkers(crop);label=bool(r['tag_id']>=0 and ids is not None and r['tag_id'] in ids.flatten())
  x,y,w,h=box;cx,cy=np.array([320,240])+r['offset_px'];center_inside=bool(r['tag_id']>=0 and x<=cx<=x+w and y<=cy<=y+h)
  means=f[5:69].reshape(8,8)[1:7,1:7];std=f[69:].reshape(8,8)[1:7,1:7];margins=np.abs(means-f[3]);records.append({'frame':r['index'],'split':r['split'],'group':'readable' if label else 'target_center_unreadable' if center_inside else 'other_unreadable','feature_quad_found':ok,'short_box_px':min(w,h),'quad_short_side_px':float(f[4]*64),'cell_margin_p10':float(np.quantile(margins,.1)) if ok else None,'cell_margin_median':float(np.median(margins)) if ok else None,'cell_std_mean':float(std.mean()) if ok else None})
summary={}
for s in ['train','validation']:
 summary[s]={}
 for g in ['readable','target_center_unreadable','other_unreadable']:
  rr=[r for r in records if r['split']==s and r['group']==g];v=[r for r in rr if r['feature_quad_found']];q={}
  for key in ['short_box_px','quad_short_side_px','cell_margin_p10','cell_margin_median','cell_std_mean']:
   values=[r[key] for r in v];q[key]=np.quantile(values,[.25,.5,.75]).tolist() if values else []
  summary[s][g]={'n':len(rr),'valid_feature_quad':len(v),'q25_median_q75':q}
report={'complete':True,'proposals':len(records),'summary':summary,'scope':'V218 train/validation only. Fixture center used for diagnostic grouping only, not features/control; center inclusion is a proxy, not exact tag boundary matching. Quad/cell metrics are image-derived and depend on correct contour selection. No threshold tuning or label changes.'};(b/'v228_cell_diagnosis.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(summary,indent=2))
