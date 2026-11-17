"""Optional encoder-feedback integral torque, bounded and reset on stop/fault."""
import numpy as np

class WheelIntegral:
    def __init__(self, ki=.04, limit=.04):
        self.ki=ki;self.limit=limit;self.value=np.zeros(4)
    def step(self,target,measured,dt,kd,enabled=True):
        target=np.asarray(target,float);measured=np.asarray(measured,float)
        if not enabled or target.shape!=(4,) or measured.shape!=(4,) or not np.isfinite(np.r_[target,measured,dt,kd]).all() or not 0<dt<=.1:
            self.value[:]=0;return self.value.copy()
        error=target-measured
        candidate=np.clip(self.value+self.ki*error*dt,-self.limit,self.limit)
        upper=np.minimum(.108,.54*(1-measured/29.321531434));lower=np.maximum(-.108,.54*(-1-measured/29.321531434))
        demand=kd*error+candidate
        accept=((demand<=upper)&(demand>=lower))|((demand>upper)&(error<0))|((demand<lower)&(error>0))
        self.value=np.where(accept,candidate,self.value)
        self.value=np.where(np.abs(target)<1e-6,0.,self.value)
        return self.value.copy()
