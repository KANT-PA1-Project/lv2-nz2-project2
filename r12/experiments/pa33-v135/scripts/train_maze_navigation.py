#!/usr/bin/env python3
import sys,json,time
from pathlib import Path
import numpy as np,torch
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'src'))
from arm_practice.navigation import Grid,Policy
# Behavior cloning of a bounded local waypoint controller; CPU avoids other GPU jobs.
torch.set_num_threads(4);torch.manual_seed(20261030);rng=np.random.default_rng(20261030)
x=rng.uniform(-.7,.7,(40000,2)).astype('float32');y=x*1.5;y*=np.minimum(1,.20/np.maximum(np.linalg.norm(y,axis=1,keepdims=True),1e-9))
x=torch.from_numpy(x);y=torch.from_numpy(y);policy=Policy();opt=torch.optim.Adam(policy.parameters(),lr=.002)
losses=[]
for epoch in range(80):
 perm=torch.randperm(len(x))
 for ix in perm.split(512):
  loss=((policy(x[ix])-y[ix])**2).mean();opt.zero_grad();loss.backward();opt.step()
 losses.append(float(loss.detach()))
print('TRAIN_LOSS',losses[-1],flush=True)
torch.save(policy.state_dict(),root/'models/navigation/waypoint_policy.pt')
torch.jit.trace(policy,torch.zeros(1,2)).save(str(root/'models/navigation/waypoint_policy.ts'))
grid=Grid(root/'models/navigation/maze_grid.npz');cfg=json.loads((root/'models/navigation/mission_navigation.json').read_text())
points={'START':cfg['start']}
for name,center in list(zip([f'S{i+1}' for i in range(6)],cfg['starts']))+list(zip(['L1','L2','L3'],cfg['destinations'])):
 # Staging pose only, not a proven manipulator reach/grasp pose.
 options=[np.array(center)+.55*np.array([np.cos(a),np.sin(a)]) for a in np.linspace(0,2*np.pi,16,endpoint=False)]
 for p in options:
  if grid.valid(p):
   try:grid.path(cfg['start'],p)
   except ValueError:continue
   points[name]=p.tolist();break
 if name not in points:raise RuntimeError(('no staging point',name))
route_order=['START','S1','S2','S3','S4','S5','S6','L1','L2','L3','START']
routes=[]
for a,b in zip(route_order,route_order[1:]):routes.append({'from':a,'to':b,'path':grid.path(points[a],points[b]).tolist()})
(root/'models/navigation/routes.json').write_text(json.dumps({'points':points,'routes':routes,'scope':'Visit all six source staging poses and three destination staging poses, return. Not puck transport schedule.'},indent=2))
# Held-out randomized starts/goals: learned local controller + known-map A*, kinematic integration.
rng=np.random.default_rng(20261104);free=np.argwhere(grid.free);cases=[]
for i in range(100):
 for attempt in range(100):
  a,b=(free[rng.integers(len(free))][::-1]*grid.res for _ in range(2))
  try:path=grid.path(a,b);break
  except ValueError:continue
 pos=a.copy();stops=0;minimum=10.;distance=0
 for step in range(6000):
  v,stopped=grid.command(policy,pos,path);stops+=int(stopped);nextpos=pos+.1*v
  distance+=np.linalg.norm(nextpos-pos);pos=nextpos;y0,x0=grid.cell(pos);minimum=min(minimum,grid.clear[y0,x0]-grid.radius)
  if np.linalg.norm(pos-b)<.04:break
 cases.append({'case':i,'success':bool(np.linalg.norm(pos-b)<.04),'steps':step+1,'safety_stops':stops,'minimum_margin_m':minimum,'distance_m':distance})
report={'method':'Behavior cloning local waypoint velocity policy; known-map A* global planner; no lidar/camera learning; kinematic evaluation only','train_samples':40000,'epochs':80,'seed':20261030,'evaluation_seed':20261104,'final_batch_mse':losses[-1],'successes':sum(c['success'] for c in cases),'episodes':100,'cases':cases}
(root/'reports/local/navigation/learning.json').write_text(json.dumps(report,indent=2));print('LEARNING',report['successes'],flush=True)
