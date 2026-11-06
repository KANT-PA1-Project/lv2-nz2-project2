"""Same-home IK labels followed by yaw-aware USD self/path filtering.
This offline pilot cannot certify unseen environment or continuous clearance.
"""
import json,time
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from audit_r12_usd_arm_path import CollisionModel
from arm_practice.learned_ik import ArmFK
root=Path(__file__).resolve().parents[1];out=root.parent.parent/'outputs/r12/collision_pilot_v179';out.mkdir(exist_ok=False);cfg=json.loads((root/'configs/r12_robot.json').read_text());fk=ArmFK(root/'models/r12/r12_arm.urdf',cfg['joint_names'],[.12475,0,0]);model=CollisionModel();rng=np.random.default_rng(179);home=np.zeros(5);phi=np.arctan2(.1854,.024);lo=np.array([-np.pi,-np.pi,-np.pi,-np.pi]);hi=np.array([np.pi,np.pi,phi,np.pi]);rows=[]
assert not model.hits(home)
def write(name,data):
 p=out/name;t=p.with_suffix(p.suffix+'.tmp');t.write_text(json.dumps(data,indent=2)+'\n');t.replace(p)
def path_check(waypoints):
 count=0
 for start,end in zip(waypoints,waypoints[1:]):
  for q in np.linspace(start,end,int(np.ceil(np.max(abs(end-start))/.01))+1):
   count+=1;hits=model.hits(q)
   if hits:return {'passed':False,'checked_samples':count,'collision_pairs':hits[:6]}
 return {'passed':True,'checked_samples':count}
write('experiment.json',dict(seed=179,targets=240,label_start_q=home.tolist(),search_lower=lo.tolist(),search_upper=hi.tolist(),search_scope='Geometric branch, not physical limit',path_joint_spacing_rad=.01,external_clearance_verified=False,execution_authorized=False))
for i in range(240):
 yaw=rng.uniform(-np.pi,np.pi);radius=rng.uniform(.18,.40);height=rng.uniform(.08,.22);target=np.array([radius*np.cos(yaw),radius*np.sin(yaw),height])
 fit=least_squares(lambda a:np.r_[fk(np.r_[a,0])-target,.1*(sum(a[1:])+np.pi/2)],home[:4],bounds=(lo,hi),max_nfev=200,ftol=1e-10,xtol=1e-10,gtol=1e-10);q=np.r_[fit.x,0];mm=float(np.linalg.norm(fk(q)-target)*1000);deg=float(abs(sum(q[1:4])+np.pi/2)*180/np.pi)
 row=dict(index=i,target_base_m=target.tolist(),q=q.tolist(),label_start_q=home.tolist(),endpoint_error_mm=mm,orientation_error_deg=deg,endpoint_pass=mm<=1 and deg<=1,path_pass=False)
 if row['endpoint_pass']:
  hits=model.hits(q);row['self_collision_pairs']=hits
  if not hits:
   direct=path_check([home,q]);row['direct_path']=direct
   if direct['passed']:row.update(path_pass=True,path_waypoints=[home.tolist(),q.tolist()])
   else:
    rotate=home.copy();rotate[0]=q[0];path=path_check([home,rotate,q]);row['rotate_then_pitch_path']=path
    if path['passed']:row.update(path_pass=True,path_waypoints=[home.tolist(),rotate.tolist(),q.tolist()])
 row['execution_authorized']=False;rows.append(row)
 if i%10==0:
  progress=dict(processed=len(rows),total=240,endpoint_pass=sum(x['endpoint_pass'] for x in rows),self_clear=sum(x['endpoint_pass'] and not x.get('self_collision_pairs',[]) for x in rows),sampled_path_pass=sum(x['path_pass'] for x in rows));write('progress.json',progress);write('partial_results.json',rows);print(json.dumps(progress),flush=True)
report=dict(complete=True,targets=len(rows),endpoint_pass=sum(x['endpoint_pass'] for x in rows),self_clear=sum(x['endpoint_pass'] and not x.get('self_collision_pairs',[]) for x in rows),sampled_path_pass=sum(x['path_pass'] for x in rows),scope='Original home for every solver label; target includes yaw; configured cube collider intersection checks with <=.01rad joint spacing. Does not prove continuous path or CAD+3cm external clearance. No environment observations used, so no executable grasp labels yet.',execution_authorized=False,rows=rows)
write('evaluation.json',report);print('PILOT_COMPLETE',json.dumps({k:v for k,v in report.items() if k!='rows'}),flush=True)
