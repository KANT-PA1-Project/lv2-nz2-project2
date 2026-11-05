"""Latch a sensor-based goal stop; this never certifies arrival or observation."""
import math


class GoalStopHold:
    radius_m = .02
    capture_speed_m_s = .015

    def __init__(self, timeout_s=12., capture_radius_m=None):
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError('Invalid stop timeout')
        self.capture_radius_m = self.radius_m if capture_radius_m is None else capture_radius_m
        if not math.isfinite(self.capture_radius_m) or not 0 < self.capture_radius_m <= self.radius_m:
            raise ValueError('Stop capture must be within the unchanged arrival radius')
        self.timeout_ns = int(timeout_s * 1e9)
        self.reset()

    def reset(self):
        self.goal = None
        self.active = self.valid = self.failed = False
        self.started_ns = self.previous_ns = None
        self.reason = 'approach'

    @property
    def braking_stop_distance_m(self):
        # The follower's zero-speed endpoint must be inside strict capture.
        return min(.01, self.capture_radius_m / 2)

    def update(self, stamp_ns, goal, distance_m, speed_m_s, stationary, valid):
        if goal != self.goal:
            self.reset()
            self.goal = goal
        clock_ok = (type(stamp_ns) is int and stamp_ns >= 0 and
                    (self.previous_ns is None or stamp_ns > self.previous_ns))
        measurement_ok = all(isinstance(x, (float, int)) and math.isfinite(x) and x >= 0
                             for x in (distance_m, speed_m_s))
        self.valid = bool(valid and clock_ok and measurement_ok)
        if not self.valid:
            self.reason = 'await_fresh_sensors'
            return self.status()
        self.previous_ns = stamp_ns
        if self.failed:
            self.reason = 'stop_timeout'
            return self.status()
        if self.active:
            # A stopped base outside the original tolerance must approach again.
            # While braking, keep a zero command instead of restarting the follower.
            if distance_m >= self.radius_m and stationary:
                self.active = False
                self.started_ns = None
                self.reason = 'reapproach'
            elif stamp_ns - self.started_ns > self.timeout_ns:
                self.failed = True
                self.reason = 'stop_timeout'
            else:
                self.reason = 'holding' if stationary else 'braking'
        elif distance_m < self.capture_radius_m and speed_m_s < self.capture_speed_m_s:
            self.active = True
            self.started_ns = stamp_ns
            self.reason = 'holding' if stationary else 'braking'
        else:
            self.reason = 'approach'
        return self.status()

    def status(self):
        return dict(active=self.active, feedback_valid=self.valid, failed=self.failed,
                    reason=self.reason, force_zero_command=self.active,
                    position_hold_allowed=self.active and self.valid,
                    arrival_radius_m=self.radius_m,
                    capture_radius_m=self.capture_radius_m,
                    scope='Stop stabilization only; original arrival/dwell checks still required')


def goal_arrival_ready(distance_m, speed_m_s, stop_hold=None):
    """Route into dwell only after capture; preserve the original acceptance bounds."""
    if not all(isinstance(v, (float, int)) and math.isfinite(v) and v >= 0
               for v in (distance_m, speed_m_s)):
        return False
    within_bounds = distance_m < GoalStopHold.radius_m and speed_m_s < GoalStopHold.capture_speed_m_s
    return within_bounds and (stop_hold is None or
                              (stop_hold.active and stop_hold.valid and not stop_hold.failed))
