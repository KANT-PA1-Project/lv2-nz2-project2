"""Known-map conservative navigation. Does not claim camera perception or grasping."""
import heapq,numpy as np,torch
from torch import nn
class Policy(nn.Module):
 def __init__(self):
  super().__init__();self.net=nn.Sequential(nn.Linear(2,64),nn.Tanh(),nn.Linear(64,64),nn.Tanh(),nn.Linear(64,2),nn.Tanh())
 def forward(self,x):return (self.net(x)-self.net(-x))*.10
class Grid:
 def __init__(self,path):
  d=np.load(path);self.clear=d['clearance'];self.res=float(d['resolution']);self.radius=float(d['radius']);self.free=self.clear>self.radius
 def cell(self,p):return tuple(np.rint(np.asarray(p)/self.res).astype(int)[::-1])
 def valid(self,p):
  y,x=self.cell(p);return 0<=y<self.free.shape[0] and 0<=x<self.free.shape[1] and self.free[y,x]
 def path(self,a,b):
  a=self.cell(a);b=self.cell(b)
  if not self.free[a] or not self.free[b]:raise ValueError(('blocked endpoint',a,b))
  todo=[(0,a)];cost={a:0};prev={}
  while todo:
   _,p=heapq.heappop(todo)
   if p==b:break
   for dy,dx in [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]:
    n=(p[0]+dy,p[1]+dx)
    if not (0<=n[0]<self.free.shape[0] and 0<=n[1]<self.free.shape[1] and self.free[n]):continue
    if dy and dx and not(self.free[p[0]+dy,p[1]] and self.free[p[0],p[1]+dx]):continue
    c=cost[p]+np.hypot(dy,dx)*(1+.08/max(self.clear[n]-self.radius,.01))
    if c<cost.get(n,1e20):cost[n]=c;prev[n]=p;heapq.heappush(todo,(c+np.linalg.norm(np.subtract(n,b)),n))
  if b not in cost:raise ValueError('unreachable')
  route=[b]
  while route[-1]!=a:route.append(prev[route[-1]])
  return np.array(route[::-1])[:,::-1]*self.res
 def command(self,policy,pos,path):
  nearest=np.argmin(np.linalg.norm(path-pos,axis=1));index=min(nearest+6,len(path)-1)
  while index>nearest+1 and not all(self.valid(p) for p in np.linspace(pos,path[index],15)):index-=1
  goal=path[index]
  with torch.no_grad():v=policy(torch.tensor(goal-pos,dtype=torch.float32)).numpy()
  norm=np.linalg.norm(v)
  if norm>.20:v*=.20/norm
  # Conservative occupancy safety filter; no hidden teacher substitution.
  if not all(self.valid(pos+v*t) for t in np.linspace(0,.1,6)):return np.zeros(2),True
  return v,False
