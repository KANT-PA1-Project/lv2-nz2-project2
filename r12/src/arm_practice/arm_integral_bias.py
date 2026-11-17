"""Experimental encoder-only position-reference integral bias; no torque override."""
import numpy as np
class ArmIntegralBias:
    def __init__(self,lower,upper,ki=1.5,cap=.025,rate=.03,deadband=0.):
        self.lower=np.asarray(lower);self.upper=np.asarray(upper);self.ki=ki;self.cap=cap;self.rate=rate;self.bias=np.zeros(len(lower));self.deadband=deadband
        if not np.isfinite(deadband) or deadband<0:raise ValueError('Invalid deadband')
    def step(self,reference,measured,dt,enabled=True):
        r=np.asarray(reference,dtype=float);q=np.asarray(measured,dtype=float)
        if r.shape!=self.bias.shape or q.shape!=self.bias.shape or not np.isfinite(np.r_[r,q,dt]).all():
            self.bias[:]=0;raise ValueError('Invalid feedback')
        if not enabled or not 0<dt<=.1:
            self.bias[:]=0;return np.clip(r,self.lower,self.upper)
        error=r-q
        error=np.where(np.abs(error)<=self.deadband,0.,error)
        delta=np.clip(self.ki*error*dt,-self.rate*dt,self.rate*dt)
        # Projection prevents windup against bias/joint bounds; drive's 2Nm cap remains independent.
        self.bias=np.clip(self.bias+delta,np.maximum(-self.cap,self.lower-r),np.minimum(self.cap,self.upper-r))
        return np.clip(r+self.bias,self.lower,self.upper)
