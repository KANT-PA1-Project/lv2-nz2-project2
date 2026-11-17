"""Two isolated CUDA actuator graphs audited against the pinned eager model.

The bench still applies eager torque. This checks formula/capture equivalence,
not the complete mission CUDA executor or graph scheduling integration.
"""
import numpy as np
import torch
from arm_practice.latest_policy import dc_effort


class TorqueParity:
    def __init__(self):
        self.graphs=[];self.inputs=[];self.outputs=[];self.calls=0;self.max_error=0.
        for steering in (True,False):
            x=torch.zeros((4,4),device='cuda:0',dtype=torch.float32)
            def fn():
                target,position,velocity,ff=x.unbind()
                nl,stall,continuous=(7.330382858,2.7,.54) if steering else (29.321531434,.54,.108)
                demand=5*(target-position)-.2*velocity if steering else .04*(target-velocity)+ff
                upper=torch.minimum(torch.full_like(velocity,continuous),stall*(1-velocity/nl))
                lower=torch.maximum(torch.full_like(velocity,-continuous),stall*(-1-velocity/nl))
                return torch.minimum(torch.maximum(demand,lower),upper)
            stream=torch.cuda.Stream();stream.wait_stream(torch.cuda.current_stream())
            with torch.cuda.stream(stream):
                for _ in range(3):fn()
            torch.cuda.current_stream().wait_stream(stream)
            g=torch.cuda.CUDAGraph()
            with torch.cuda.graph(g):y=fn()
            self.graphs.append(g);self.inputs.append(x);self.outputs.append(y)
        rng=np.random.default_rng(3311)
        for steering in (True,False):
            for _ in range(64):
                scale=1.5 if steering else 29.
                self.check(steering,rng.uniform(-scale,scale,4),rng.uniform(-1.5,1.5,4),rng.uniform(-scale,scale,4),rng.uniform(-.04,.04,4))

    def check(self,steering,target,position,velocity,ff=None):
        ff=np.zeros(4) if ff is None else np.asarray(ff)
        index=0 if steering else 1
        self.inputs[index].copy_(torch.as_tensor(np.stack([target,position,velocity,ff]),device='cuda:0',dtype=torch.float32))
        self.graphs[index].replay();actual=self.outputs[index].cpu().numpy()
        expected=dc_effort(target,position,velocity,steering=steering,wheel_kd=.04,wheel_feedforward=ff)
        error=float(np.max(abs(actual-expected)));self.max_error=max(self.max_error,error);self.calls+=1
        if error>3e-6:raise RuntimeError('CUDA/eager torque mismatch '+str(error))

    def result(self):
        return {'graphs':2,'calls':self.calls,'maximum_absolute_error_Nm':self.max_error,'tolerance_Nm':3e-6,'passed':True,'applied_backend':'existing_numpy_eager','scope':'Two graph formula parity, not full mission CUDA executor integration'}
