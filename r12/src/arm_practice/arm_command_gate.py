"""Lesson p61: exclusive arm command ownership and finite/fresh feedback.

All stamps use the same simulation/ROS clock. This is a command interlock,
not collision checking, a planner, or evidence of grasp success.
"""
import math

class ArmCommandGate:
    MODES=('hold','transport','planner','servo','policy')
    def __init__(self, names, lower, upper, velocity, timeout=.25):
        self.names=list(names);self.lower=lower;self.upper=upper;self.velocity=velocity
        self.timeout=timeout;self.mode='hold';self.target=None;self.stamp=None;self.last=None;self.reason='hold';self.hold_target=None;self.reference=None
    def select(self, mode, measured_velocity):
        if mode not in self.MODES:raise ValueError('Unknown arm control mode')
        if len(measured_velocity)!=len(self.names) or not all(math.isfinite(v) and abs(v)<.02 for v in measured_velocity):
            raise ValueError('Mode changes require stationary measured joints')
        self.mode=mode;self.target=None;self.stamp=None;self.last=None;self.hold_target=None;self.reference=None
    def submit(self, mode, names, positions, stamp):
        if mode!=self.mode or mode=='hold':return False
        if list(names)!=self.names or len(positions)!=len(self.names):return False
        if not math.isfinite(stamp) or not all(math.isfinite(q) and lo<=q<=hi for q,lo,hi in zip(positions,self.lower,self.upper)):return False
        if self.stamp is not None and stamp<=self.stamp:return False
        self.target=list(positions);self.stamp=stamp;return True
    def step(self, now, measured, state_stamp):
        if len(measured)!=len(self.names) or not all(math.isfinite(q) for q in measured):
            self.reason='invalid_feedback';return None
        if not all(math.isfinite(t) for t in [now,state_stamp]) or not 0<=now-state_stamp<=self.timeout:
            self.reason='stale_feedback';self.last=None;return None
        if self.mode=='hold' or self.target is None or not 0<=now-self.stamp<=self.timeout:
            self.reason='hold' if self.mode=='hold' else 'stale_command';self.last=now
            if self.hold_target is None:self.hold_target=list(measured)
            self.reference=None
            return list(self.hold_target)
        dt=0 if self.last is None else now-self.last;self.last=now
        if not 0<dt<=self.timeout:
            self.reason='clock_reset';self.reference=list(measured);return list(measured)
        self.reason='tracking';self.hold_target=None
        if self.reference is None:self.reference=list(measured)
        self.reference=[max(lo,min(hi,q+max(-v*dt,min(v*dt,t-q)))) for q,t,v,lo,hi in zip(self.reference,self.target,self.velocity,self.lower,self.upper)]
        return list(self.reference)
