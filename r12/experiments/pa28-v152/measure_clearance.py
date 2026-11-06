from pathlib import Path
import json,sys,numpy as np
root=Path(__file__).parent;exec(compile((root/'path.py').read_text().split('center=(tg@TCP)')[0],'geometry','exec'));o=Path(sys.argv[1]);rows=json.loads((o/'per_step.json').read_text());init=json.loads((o/'initialization.json').read_text());cc=np.array(init['center']);height=cc[2]-.0301;sp=(np.r_[cc[:2],height/2-.008],np.eye(3),np.array([.03,.03,height]));out=[]
for r in rows[::96]:
 _,items=boxes_at(r['arm_q'],r['aperture_m']);pc=np.array(r['puck_position_evaluator'])-[0,0,.008];quat=r['puck_quat_evaluator'];rot=R.from_quat(np.r_[quat[1:],quat[0]]).as_matrix();ob=(pc,rot,np.array([.03,.03,.06]));nonpads=[];pads=[];env=[]
 for name,body,box,*_ in items:
  if name.endswith('/pad_collision'):pads.append((distance(box,ob),name))
  elif overlap(box,ob,.05):nonpads.append((distance(box,ob),name))
  if overlap(box,sp,.05):env.append((distance(box,sp),name))
 out.append(dict(step=r['step'],phase=r['phase'],nonpad_min=min(nonpads,default=(None,None)),pad_min=min(pads),env_min=min(env,default=(None,None))))
summary={}
for ph in dict.fromkeys(r['phase']for r in out):
 x=[r for r in out if r['phase']==ph];summary[ph]=dict(samples=len(x),nonpad_min_m=min([r['nonpad_min'][0]for r in x if r['nonpad_min'][0]is not None],default=None),pad_min_m=min(r['pad_min'][0]for r in x),environment_min_m=min([r['env_min'][0]for r in x if r['env_min'][0]is not None],default=None))
res=dict(scope='Recorded actual pose and joint geometry sampled10Hz; exact cuboid OBB distances, no continuous certificate',summary=summary,samples=out);(o/'measured_clearance.json').write_text(json.dumps(res,indent=2));print(json.dumps(summary,indent=2))
