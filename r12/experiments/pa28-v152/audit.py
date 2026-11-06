import json,hashlib,time,platform
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R
p=Path(__file__).parent;g=json.loads((p/'geometry.json').read_text());data=json.loads((p/'candidates.json').read_text());rng=np.random.default_rng(2408)
import xml.etree.ElementTree as E
urdf=E.parse(p/'r12_arm.urdf').getroot();j={x.get('name'):x for x in urdf.findall('joint')};names=['arm_joint'+str(i) for i in range(1,6)];low=np.array([float(j[n].find('limit').get('lower'))for n in names]);high=np.array([float(j[n].find('limit').get('upper'))for n in names])
def tf(q):
 vals=dict(zip(names,q));vals.update(gripper_left_joint=.02768613,gripper_right_joint=-.02768613);ts={'base_link':np.eye(4)};pending=g['joints'].copy()
 while pending:
  for e in pending[:]:
   n,a,b,l0,l1,axis,prism=e
   if a not in ts:continue
   m=np.eye(4);v=vals.get(n,0);ax=np.eye(3)['XYZ'.index(axis)]
   if prism:m[:3,3]=ax*v
   else:m[:3,:3]=R.from_rotvec(ax*v).as_matrix()
   ts[b]=ts[a]@l0@m@np.linalg.inv(l1);pending.remove(e)
 return ts
TCP=np.array([.12475,0,0,1.])
def solve(target,axis,seed):
 def fun(q):
  t=tf(q)['arm_link5'];return np.r_[(t@TCP)[:3]-target,.15*(t[:3,0]-axis)]
 sols=[least_squares(fun,s,bounds=(low+1e-7,high-1e-7),max_nfev=120,ftol=1e-9,xtol=1e-9,gtol=1e-9)for s in [seed,np.zeros(5),*rng.uniform(np.maximum(low,-1.3),np.minimum(high,1.3),size=(6,5))]]
 best=min(sols,key=lambda s:np.linalg.norm(s.fun));t=tf(best.x)['arm_link5'];return best.x,float(np.linalg.norm((t@TCP)[:3]-target)),float(np.degrees(np.arccos(np.clip(t[:3,0]@axis,-1,1))))
def overlap(a,b,margin=0):
 ca,ra,sa=a;cb,rb,sb=b;sa=sa/2+margin/2;sb=sb/2+margin/2
 axes=[*ra.T,*rb.T,*[np.cross(x,y)for x in ra.T for y in rb.T]];d=cb-ca
 return all(abs(d@x)<=sum(sa*abs(ra.T@x))+sum(sb*abs(rb.T@x))+1e-10 for x in axes if np.linalg.norm(x)>1e-9)
skip={frozenset(x)for x in g['skip']}
def collisions(q,center):
 ts=tf(q);boxes=[]
 for n,b,l,size in g['boxes']:
  t=ts[b]@l;boxes.append((n,b,(t[:3,3],t[:3,:3],np.array(size))))
 height=center[2]-.0301;support=(np.array([center[0],center[1],height/2]),np.eye(3),np.array([.14,.14,height]));obj=(center,np.eye(3),np.full(3,.06));hits=[]
 for i,(n,b,o) in enumerate(boxes):
  if overlap(o,support,.03):hits.append([b,'support_margin3cm'])
  if b not in ['gripper_left','gripper_right'] and overlap(o,obj):hits.append([b,'puck_envelope'])
  for n2,b2,o2 in boxes[i+1:]:
   if b!=b2 and frozenset([b,b2])not in skip and overlap(o,o2):hits.append([b,b2])
 return hits
rows=[]
for ix in [45,54]:
 c=data['candidates'][ix];center=np.array(c['fixture_center_base_m']);radial=center.copy();radial[2]=0;radial/=np.linalg.norm(radial)
 for mode,axis in [('top',np.array([0.,0.,-1.])),('side_radial',radial)]:
  q=np.array(c['observe_q']);stages=[]
  for name,target in [('pregrasp',center-.05*axis),('grasp',center),('lift',center+[0,0,.03])]:
   nxt,err,ang=solve(target,axis,q);samples=max(2,int(np.ceil(np.max(abs(nxt-q))/.025))+1);hits=[]
   for k,point in enumerate(np.linspace(q,nxt,samples)):
    h=collisions(point,center)
    if h:hits.append({'sample':k,'pairs':h[:6]})
   stages.append({'stage':name,'q':nxt.tolist(),'position_residual_m':err,'axis_error_deg':ang,'sample_count':samples,'collision_sample_count':len(hits),'examples':hits[:2]});q=nxt
  rows.append({'candidate':ix,'mode':mode,'stages':stages,'pose_feasible':all(s['position_residual_m']<=.005 and s['axis_error_deg']<=5 for s in stages),'sampled_collision_clear':all(s['collision_sample_count']==0 for s in stages)})
result={'host':platform.node(),'seed':2408,'scope':'CPU setup/evaluator only; +X approach axis constrained with free roll. USD cube colliders/SAT sampled .025rad; 6cm puck bounding cube, 14cm support; 3cm support margin only. Finger-puck intended contact excluded, actual contact/closure and continuous collision NOT verified. No GT control.','motion_authorized':False,'native_smoke_ready':False,'rows':rows,'hashes':{x.name:hashlib.sha256(x.read_bytes()).hexdigest()for x in p.iterdir()if x.is_file() and x.name!='result.json'}};(p/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
