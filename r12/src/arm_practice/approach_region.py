"""Coarse navigation arrival, explicitly separate from arm reach/pose acceptance."""
import math
from .goal_stop_hold import GoalStopHold


class ApproachRegion(GoalStopHold):
    def __init__(self, radius_m=.10):
        if not math.isfinite(radius_m) or not .05<=radius_m<=.20:
            raise ValueError('Coarse navigation region must be within5..20cm')
        self.radius_m=radius_m
        super().__init__(capture_radius_m=radius_m)

    def status(self):
        return dict(super().status(),scope='Coarse navigation stop only; wrist/arm must separately verify reach, collision clearance and fine pose.')


def region_arrival_ready(distance_m,speed_m_s,hold):
    if not isinstance(hold,ApproachRegion):
        raise ValueError('Explicit coarse navigation region required')
    return (all(isinstance(x,(int,float)) and math.isfinite(x) and x>=0 for x in (distance_m,speed_m_s))
            and distance_m<hold.radius_m and speed_m_s<hold.capture_speed_m_s
            and hold.active and hold.valid and not hold.failed)
