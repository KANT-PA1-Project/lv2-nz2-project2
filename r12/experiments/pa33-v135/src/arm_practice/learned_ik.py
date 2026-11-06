"""Position-only neural IK proposal, bounded solver finish, conservative cloud gate.
No trajectory execution or collision certification is implied by endpoint acceptance.
"""
import xml.etree.ElementTree as ET
import numpy as np
import torch
from scipy.optimize import least_squares
from scipy.spatial import Delaunay, cKDTree
from .kinematics import rotation

class ArmFK:
    def __init__(self, urdf, names, offset):
        root=ET.parse(urdf).getroot(); by_child={j.find('child').get('link'):j for j in root.findall('joint')}
        chain=[];link='arm_link5'
        while link in by_child:
            j=by_child[link];chain.append(j);link=j.find('parent').get('link')
        self.chain=[];self.offset=np.r_[offset,1.]
        for j in reversed(chain):
            o=j.find('origin');xyz=np.fromstring(o.get('xyz','0 0 0'),sep=' ');rpy=np.fromstring(o.get('rpy','0 0 0'),sep=' ')
            T=np.eye(4);T[:3,3]=xyz;T[:3,:3]=rotation([0,0,1],rpy[2])@rotation([0,1,0],rpy[1])@rotation([1,0,0],rpy[0])
            idx=names.index(j.get('name')) if j.get('name') in names else None
            if idx is None and j.get('type')!='fixed':raise ValueError('Unmodeled moving joint')
            axis=np.fromstring(j.find('axis').get('xyz'),sep=' ') if idx is not None else None
            self.chain.append((T,idx,axis))
    def __call__(self,q):
        T=np.eye(4)
        for origin,idx,axis in self.chain:
            T=T@origin
            if idx is not None:
                R=np.eye(4);R[:3,:3]=rotation(axis,q[idx]);T=T@R
        return (T@self.offset)[:3]

class CloudGate:
    def __init__(self,points,nearest_m=.025,simplex_diameter_m=.08):
        self.points=np.asarray(points);self.hull=Delaunay(self.points);self.tree=cKDTree(self.points)
        self.nearest_m=nearest_m;self.diameter_m=simplex_diameter_m
    def contains(self,target):
        x=np.asarray(target,dtype=float)
        if x.shape!=(3,) or not np.isfinite(x).all():return False
        i=int(self.hull.find_simplex(x))
        if i<0 or self.tree.query(x)[0]>self.nearest_m:return False
        p=self.points[self.hull.simplices[i]]
        return bool(np.linalg.norm(p[:,None]-p[None,:],axis=-1).max()<=self.diameter_m)

def network():
    return torch.nn.Sequential(torch.nn.Linear(3,64),torch.nn.Tanh(),torch.nn.Linear(64,64),torch.nn.Tanh(),torch.nn.Linear(64,4))

def finish(fk,target,seed,lower,upper,home,max_nfev=100):
    # Joint5 rotates about TCP's own X axis: position alone cannot identify it.
    def full(q):return np.r_[q,home[4]]
    fit=least_squares(lambda q:fk(full(q))-target,np.clip(seed[:4],lower[:4],upper[:4]),bounds=(lower[:4],upper[:4]),max_nfev=max_nfev,ftol=1e-11,xtol=1e-11,gtol=1e-11)
    q=full(fit.x);return q,float(np.linalg.norm(fk(q)-target)*1000),int(fit.nfev)

class LearnedIK:
    def __init__(self,artifact,fk):
        d=torch.load(artifact,weights_only=False,map_location='cpu');self.net=network();self.net.load_state_dict(d['weights']);self.net.eval()
        self.d=d;self.fk=fk;self.gate=CloudGate(d['train_xyz'])
    def solve(self,target):
        target=np.asarray(target,dtype=float)
        if not self.gate.contains(target):return {'accepted':False,'reason':'outside_supported_training_cloud'}
        d=self.d
        with torch.no_grad():p=self.net(torch.tensor((target-d['mean'])/d['std'],dtype=torch.float32)).numpy()
        seed=np.r_[np.clip(p*d['scale'][:4],d['lower'][:4],d['upper'][:4]),d['home'][4]]
        q,error,n=finish(self.fk,target,seed,d['lower'],d['upper'],d['home'])
        return {'accepted':bool(error<=d['tolerance_mm']),'reason':'endpoint_only','q':q.tolist(),'proposal_error_mm':float(np.linalg.norm(self.fk(seed)-target)*1000),'error_mm':error,'solver_nfev':n,'trajectory_validated':False}
