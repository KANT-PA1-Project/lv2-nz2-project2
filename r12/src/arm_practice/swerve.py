"""Learned wheel-plane velocity allocation with explicit actuator constraints."""
import numpy as np,torch
from torch import nn
OFFSETS=np.array([[.1,.1],[.1,-.1],[-.1,.1],[-.1,-.1]])
BASE_DIRECTIONS=np.array([-np.pi/2,-np.pi/2,0,np.pi])
RADIUS=.03068719889482
class SwervePolicy(nn.Module):
 def __init__(self):
  super().__init__();self.net=nn.Sequential(nn.Linear(3,32,bias=False),nn.Tanh(),nn.Linear(32,8,bias=False))
 def forward(self,x):return self.net(x)
def teacher(twist):
 t=np.asarray(twist);return (t[...,:2,None].swapaxes(-1,-2)+t[...,2,None,None]*np.column_stack([-OFFSETS[:,1],OFFSETS[:,0]])).reshape(*t.shape[:-1],8)
def actuate(policy,twist,current,dt=.05,previous_target=None):
 twist=np.asarray(twist,dtype=float).copy()
 if not np.isfinite(twist).all() or not np.isfinite(current).all():raise ValueError('nonfinite command/state')
 norm=np.linalg.norm(twist[:2]);twist[:2]*=min(1,.20/max(norm,1e-9));twist[2]=np.clip(twist[2],-.4,.4)
 if np.linalg.norm(twist)<1e-6:return np.asarray(current).copy(),np.zeros(4)
 with torch.no_grad():vectors=policy(torch.tensor(twist,dtype=torch.float32)).numpy().reshape(4,2)
 heading=np.arctan2(vectors[:,1],vectors[:,0]);angle=(heading-BASE_DIRECTIONS+np.pi)%(2*np.pi)-np.pi
 speed=np.linalg.norm(vectors,axis=1);reverse=np.abs(angle)>np.pi/2;angle[reverse]-=np.sign(angle[reverse])*np.pi;speed[reverse]*=-1
 previous=np.asarray(current if previous_target is None else previous_target)
 # Keep the selected end-stop branch across small +/-90 degree noise.
 hold=(np.abs(np.abs(angle)-np.pi/2)<.06)&(np.abs(previous)>.7)
 angle[hold]=np.sign(previous[hold])*np.pi/2
 speed=np.sum(vectors*np.column_stack([np.cos(BASE_DIRECTIONS+angle),np.sin(BASE_DIRECTIONS+angle)]),axis=1)
 target=previous+np.clip(angle-previous,-dt,dt)
 # Wait for steering alignment: avoid lateral scrub during large direction changes.
 speed*=np.clip(np.cos(angle-current),0,1);
 if np.max(np.abs(angle-current))>.25:speed[:]=0
 return np.clip(target,-np.pi/2,np.pi/2),np.clip(speed/RADIUS,-10,10)

def wheel_odometry(steering, wheel_velocity):
 """All four steering angles + four wheel encoders, least-squares body twist.

 Residual is a consistency indicator, not a complete slip detector. Common-mode
 slip needs IMU/visual comparison. Encoder velocities must be rad/s.
 """
 steering=np.asarray(steering);wheel_velocity=np.asarray(wheel_velocity)
 if steering.shape!=(4,) or wheel_velocity.shape!=(4,) or not np.isfinite(np.r_[steering,wheel_velocity]).all():raise ValueError('Need all 8 finite encoder measurements')
 heading=BASE_DIRECTIONS+steering
 vectors=RADIUS*wheel_velocity[:,None]*np.column_stack([np.cos(heading),np.sin(heading)])
 A=np.zeros((8,3));A[::2,0]=1;A[1::2,1]=1;A[::2,2]=-OFFSETS[:,1];A[1::2,2]=OFFSETS[:,0]
 twist=np.linalg.lstsq(A,vectors.ravel(),rcond=None)[0]
 return twist,float(np.sqrt(np.mean((A@twist-vectors.ravel())**2)))
