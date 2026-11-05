"""Encoder-based fold/settle interlock. No environment or simulator truth input."""
import numpy as np

class TransportPosture:
    def __init__(self,config,*,require_clearance=False):
        # Kinematic fixtures may exercise slew alone; mission callers must require clearance.
        self.require_clearance=require_clearance
        self.goal=np.array(config['arm_target'],float);self.tolerance=config['joint_tolerance_rad'];self.rate=config['joint_velocity_limit_rad_s'];self.settle=config['settle_seconds'];self.command=None;self.since=None;self.last=None;self.ready=False;self.state='await_feedback'
    def step(self,now,position,velocity,base_stopped,valid=True):
        position=np.asarray(position);velocity=np.asarray(velocity)
        if not valid or position.shape!=self.goal.shape or velocity.shape!=self.goal.shape or not np.isfinite(np.r_[now,position,velocity]).all():
            self.ready=False;self.state='invalid_feedback';self.last=None;return None
        dt=0 if self.last is None else now-self.last;self.last=now
        if not 0<=dt<=.1:self.ready=False;self.since=None;self.state='clock_fault';return None
        if self.require_clearance:
            # No authenticated swept-volume verifier is connected to this legacy
            # transport controller yet. Encoder settling is not path clearance.
            self.ready=False;self.since=None;self.command=position.copy()
            self.state='await_swept_volume_clearance';return None
        close=np.max(np.abs(position-self.goal))<=self.tolerance
        if self.ready and close:self.state='transport_ready';return self.goal.copy()
        self.ready=False
        if not base_stopped:self.since=None;self.state='await_base_stop';return None
        if self.command is None:self.command=position.copy()
        self.command+=np.clip(self.goal-self.command,-self.rate*dt,self.rate*dt)
        if close and np.max(np.abs(velocity))<.02 and np.max(np.abs(self.command-self.goal))<1e-6:
            if self.since is None:self.since=now
            self.ready=now-self.since>=self.settle
            self.state='transport_ready' if self.ready else 'settling'
        else:self.since=None;self.state='folding'
        return self.command.copy()
