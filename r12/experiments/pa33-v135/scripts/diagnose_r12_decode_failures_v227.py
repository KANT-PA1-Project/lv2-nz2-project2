from pathlib import Path
import ast,json,cv2,numpy as np
root=Path(__file__).resolve().parents[1];b=root.parent.parent/'outputs/r12';src=b/'tag_render_v226'
t=ast.parse((root/'scripts/evaluate_r12_proposals_v209.py').read_text());f=next(n for n in t.body if isinstance(n,ast.FunctionDef) and n.name=='proposals');exec(compile(ast.Module(body=[f],type_ignores=[]),'proposals','exec'))
det=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11));fail=json.loads((b/'v227_failure_locations.json').read_text())
for r in fail:
 im=cv2.imread(str(src/f'frame_{r["frame"]:03d}.png'),0);boxes,crops=proposals(im);crop=crops[boxes.index(tuple(r['box']))]
 variants={'native_full_frame':im,'original_proposal64':crop,'proposal128_nearest':cv2.resize(crop,(128,128),interpolation=cv2.INTER_NEAREST),'proposal128_linear':cv2.resize(crop,(128,128),interpolation=cv2.INTER_LINEAR),'proposal256_linear':cv2.resize(crop,(256,256),interpolation=cv2.INTER_LINEAR)}
 r['diagnostic_decodes']={}
 for name,img in variants.items():
  _,ids,rejected=det.detectMarkers(img);decoded=[] if ids is None else ids.flatten().tolist();r['diagnostic_decodes'][name]={'ids':decoded,'correct_id':r['tag_id'] in decoded,'rejected_quads':len(rejected)}
report={'complete':True,'failures':fail,'scope':'Posthoc consumed V226 diagnosis. Fixture center only for interpretation, never proposal/model input. Upsampling is diagnostic, adds no optical information; original labels and frozen evaluation remain unchanged.'};(b/'v227_decode_diagnosis.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
