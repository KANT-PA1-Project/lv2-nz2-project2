from pathlib import Path
exec(compile((Path(__file__).parent/'audit.py').read_text().split('rows=[]')[0],'audit.py','exec'))
from scipy.optimize import lsq_linear
import time
started=time.monotonic();fixture=json.loads((p/'search_result.json').read_text())['survivors'][0];near=json.loads((p/'near_result.json').read_text())['rows'][0];pre=np.array(near['pregrasp_q']);grasp=np.array(fixture['q_grasp']);lift=np.array(fixture['stages'][1]['q']);center=np.array(fixture['center']);height=center[2]-.0301;support=(np.r_[center[:2],height/2],np.eye(3),np.array([.03,.03,height]));tg=tf(grasp)['arm_link5'];relative=np.linalg.inv(tg)@np.r_[center,1];rows=[]
def distance(a,b):
 ca,ra,sa=a;cb,rb,sb=b;A=np.c_[ra*sa/2,-rb*sb/2];sol=lsq_linear(A,cb-ca,bounds=(-np.ones(6),np.ones(6)),tol=1e-12,max_iter=100,method="bvls");return float(np.linalg.norm(A@sol.x-(cb-ca)))
def boxes_at(q,width):
 ts=tf(q);items=[]
 for n,b,l,size in g['boxes']:
  t=ts[b]@l
  if b=='gripper_left':t[:3,3]+=ts['arm_link5'][:3,1]*(width-.075)/2
  if b=='gripper_right':t[:3,3]-=ts['arm_link5'][:3,1]*(width-.075)/2
  size=np.array(size);half=abs(t[:3,:3])@size/2;items.append((n,b,(t[:3,3],t[:3,:3],size),t[:3,3]-half,t[:3,3]+half))
 return ts,items
# Historical function name retained for call compatibility; geometry is exact cuboid OBB.
def cylinder_distance(o,cc,cr):
 return distance(o,(cc,cr,np.array([.03,.03,.06]))),True

center=(tg@TCP)[:3]+.0065*tg[:3,0]-[0,0,.012]
relative=np.linalg.inv(tg)@np.r_[center,1]
height=center[2]-.0301
support=(np.r_[center[:2],height/2],np.eye(3),np.array([.03,.03,height]))

placeq=np.array([0.,.15327346123403265,-.27269988318568805,.11942642195218953,0.])
enc=json.loads((p/'cpu_encoder_fixture.json').read_text());qlevel=placeq+enc['level'];qnear=qlevel+enc['near'];measured=np.array(enc['encoder']);target=(tf(measured)['arm_link5']@TCP)[:3];target[2]+=.010
def retreat_error(x):
 qt=measured.copy();qt[1:4]=x;z=tf(qt)['arm_link5'];return np.r_[(z@TCP)[[0,2]]-target[[0,2]],.15*z[2,0]]
sol=least_squares(retreat_error,measured[1:4],bounds=(low[1:4],high[1:4]),gtol=1e-12,xtol=1e-12,ftol=1e-12);delta=np.zeros(5);delta[1:4]=sol.x-measured[1:4];qretreat=np.array(json.loads((p/'unloaded_retreat_cpu_input.json').read_text())['reference_goal'])
(p/'retreat_cpu_plan.json').write_text(json.dumps(dict(qnear=qnear.tolist(),qretreat=qretreat.tolist(),retreat_delta=delta.tolist(),retreat_min_s=float(max(abs(delta))/.005),return_min_s=float(max(abs(qretreat-grasp))/.005),residual=float(np.linalg.norm(sol.fun))),indent=2))
segments=[('close',grasp,grasp,.075,.027,False),('lift',grasp,lift,.027,.027,True),('lower',lift,placeq,.027,.027,True),('level',placeq,qlevel,.027,.027,True),('near_lower',qlevel,qnear,.027,.027,True),('release_airborne',qnear,qnear,.027,.075,True),('release_supported',qnear,qnear,.027,.075,False),('retreat',qnear,qretreat,.075,.075,False),('arm_return_initial_fixture',qretreat,grasp,.075,.075,False)]

for phase,qa,qb,wa,wb,carried in segments:
 count=max(2,int(np.ceil(max(abs(qb-qa))/.005))+1,int(np.ceil(abs(wb-wa)/.002))+1)
 for k,t in enumerate(np.linspace(0,1,count)):
  q=qa+(qb-qa)*t;width=wa+(wb-wa)*t;ts,items=boxes_at(q,width);objcenter=(ts['arm_link5']@relative)[:3]if carried else center.copy();objrot=ts['arm_link5'][:3,:3]@tg[:3,:3].T if carried else np.eye(3);obj=(objcenter,objrot,np.array([.03,.03,.06]));pads=[];nonpad=[];env=[]
  for n,b,o,*_ in items:
   if n.endswith('/pad_collision'):pads.append({'name':n,'distance_to_cylinder_m':cylinder_distance(o,objcenter,objrot)[0],'nominal_cylinder_intersection':cylinder_distance(o,objcenter,objrot)[0]<1e-8,'conservative_3cm_overlap':overlap(o,obj,.03)})
   elif overlap(o,obj,.03):nonpad.append({'name':n,'distance_m':cylinder_distance(o,objcenter,objrot)[0],'nominal_overlap':cylinder_distance(o,objcenter,objrot)[0]<1e-8})
   if overlap(o,support,.03):env.append({'name':n,'distance_m':distance(o,support),'nominal_overlap':overlap(o,support)})
  mins=np.array([x[3]for x in items]);maxs=np.array([x[4]for x in items]);pairs=np.argwhere(np.triu(np.all(mins[:,None,:]<=maxs[None,:,:],axis=2)&np.all(maxs[:,None,:]>=mins[None,:,:],axis=2),1));selfhits=[]
  for i,j in pairs:
   a,b=items[i],items[j]
   if a[1]!=b[1]and frozenset([a[1],b[1]])not in skip and overlap(a[2],b[2]):selfhits.append([a[0],b[0]])
  rows.append({'phase':phase,'fraction':float(t),'q':q.tolist(),'aperture_m':float(width),'puck_center_base_m':objcenter.tolist(),'tcp_to_object_m':float(np.linalg.norm((ts['arm_link5']@TCP)[:3]-objcenter)),'pad_object':pads,'nonpad_object_margin_candidates':nonpad,'environment_margin_candidates':env,'self_collisions':selfhits,'clearance_allowed':False})
summary=[]
for phase,*_ in segments:
 r=[x for x in rows if x['phase']==phase];summary.append({'phase':phase,'samples':len(r),'pad_distance_min_m':min(a['distance_to_cylinder_m']for x in r for a in x['pad_object']),'pad_distance_max_m':max(a['distance_to_cylinder_m']for x in r for a in x['pad_object']),'nominal_pad_overlap_samples':sum(any(a['nominal_cylinder_intersection']for a in x['pad_object'])for x in r),'nonpad_under3cm_samples':sum(any(a['distance_m']<.03-1e-7 for a in x['nonpad_object_margin_candidates'])for x in r),'environment_under3cm_samples':sum(any(a['distance_m']<.03-1e-7 for a in x['environment_margin_candidates'])for x in r),'selfcollision_samples':sum(bool(x['self_collisions'])for x in r)})
result={'scope':'CPU hypothetical geometry only. Actual green cuboid30x30x60mm; exact OBB distance via bounded linear least squares. +X2mmoffset, TCP12mmaboveobject and27mmclosure fixed. During lift/place only, evaluator assumes perfect rigid retention to produce swept geometry, NOT verified physical attachment or control observation. Open→27mm closure bounded by existinggripper limits. Environment3cm and nonpad3cm retained; no approach pad exemption.','sampling':'jointincrements<=.005rad, apertureincrements<=.002m, endpoints included. No continuous interval certificate; unsampled collisions remain possible.','distance':'Finitecylinder-to-OBB convex distance for object; BVLS OBB distance for support.','summary':summary,'rows':rows,'clearance_allowed':False,'native_execution':False,'wall_s':time.monotonic()-started};(p/'path_result.json').write_text(json.dumps(result,indent=2));print(json.dumps(summary))
