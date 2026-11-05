"""Contiguous physics-step settling diagnostic, not a motion certificate.

Inputs must come from the same pre/post physics step. Identity, freshness and
capture association remain separate authenticated producer responsibilities.
"""
import math


class SettlingWindow:
    def __init__(self, physics_hz=960, dwell_s=.5, speed_limit=.02):
        if (type(physics_hz) is not int or physics_hz <= 0
                or type(dwell_s) not in (int, float) or not math.isfinite(dwell_s)
                or dwell_s <= 0 or type(speed_limit) not in (int, float)
                or not math.isfinite(speed_limit) or speed_limit <= 0):
            raise ValueError('invalid settling window')
        self.required = math.ceil(physics_hz * dwell_s)
        self.limit = speed_limit
        self.previous = None
        self.count = 0

    def update(self, step, native_qd, same_step_fd):
        valid_step = type(step) is int and step >= 0
        consecutive = valid_step and (self.previous is None or step == self.previous + 1)
        self.previous = step if valid_step else None
        try:
            valid = all(len(v) == 5 and all(type(x) in (int, float)
                        and math.isfinite(x) and abs(x) < self.limit for x in v)
                        for v in (native_qd, same_step_fd))
        except TypeError:
            valid = False
        # A discontinuous record cannot count as a validated interval.
        self.count = self.count + 1 if consecutive and valid else 0
        return {'continuous_steps': self.count, 'required_steps': self.required,
                'settled_interval': self.count >= self.required,
                'motion_authorized': False}
