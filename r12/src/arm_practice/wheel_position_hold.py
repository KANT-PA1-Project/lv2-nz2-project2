"""Encoder-only stationary wheel hold; final actuator torque limits still apply."""
import numpy as np

class WheelPositionHold:
    def __init__(self, gain=.3, limit=.04):
        self.gain,self.limit=gain,limit
        self.anchor=None
    def step(self, positions, enabled, retain_anchor=False):
        q=np.asarray(positions,float)
        if not enabled or q.shape!=(4,) or not np.isfinite(q).all():
            if not retain_anchor:self.anchor=None
            return np.zeros(4)
        if self.anchor is None:self.anchor=q.copy()
        return np.clip(self.gain*(self.anchor-q),-self.limit,self.limit)
