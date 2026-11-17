"""Stationary, bounded arm motion for a diagnostic camera view, not a grasp."""
import numpy as np


class ObservationPosture:
    def __init__(self, goal, lower, upper, waypoints=None):
        self.goal = np.asarray(goal, float)
        if self.goal.shape != (5,) or not np.isfinite(self.goal).all() or np.any(self.goal < lower) or np.any(self.goal > upper):
            raise ValueError('Invalid observation posture')
        self.waypoints = [np.asarray(x,float) for x in (waypoints if waypoints is not None else [goal])]
        if not self.waypoints or any(x.shape!=(5,) or not np.isfinite(x).all() or np.any(x<lower) or np.any(x>upper) for x in self.waypoints) or not np.allclose(self.waypoints[-1],self.goal):
            raise ValueError('Invalid observation waypoints')
        self.waypoint_index=0
        self.goal=self.waypoints[0]
        self.state = 'waiting'
        self.reason = None
        self.last = self.stable = self.exposure = None
        self.reference = self.anchor = None
        self.pause_since = None

    @property
    def done(self):
        return self.state in ('complete', 'failed')

    def step(self, now, q, velocity, pose, stopped, valid):
        q = np.asarray(q, float)
        velocity = np.asarray(velocity, float)
        pose = np.asarray(pose, float) if pose is not None else np.array([])
        good = valid and q.shape == (5,) and velocity.shape == (5,) and pose.shape == (3,) and np.isfinite(np.r_[now,q,velocity,pose]).all()
        if self.done:
            return None
        if not good:
            if self.state != 'waiting':
                if self.pause_since is None:
                    self.pause_since = now
                self.state, self.reason, self.last = 'paused', 'await_fresh_sensors', now
                if not np.isfinite(now) or now-self.pause_since > 1.:
                    self.state, self.reason = 'failed', 'sensor_timeout'
            self.stable = None
            return None
        if not stopped:
            if self.state != 'waiting':
                self.state, self.reason = 'failed', 'base_moved'
            self.stable = None
            return None
        if self.state == 'waiting':
            if np.max(np.abs(velocity)) >= .02:
                self.stable = None
                return None
            if self.stable is None:
                self.stable = now
            if now-self.stable < .5:
                return None
            self.state, self.reference, self.anchor, self.last = 'moving', q.copy(), pose.copy(), now
            return q.copy()
        dyaw = np.arctan2(np.sin(pose[2]-self.anchor[2]),np.cos(pose[2]-self.anchor[2]))
        if np.linalg.norm(pose[:2]-self.anchor[:2]) > .015 or abs(dyaw) > .03:
            self.state, self.reason = 'failed', 'base_pose_drift'
            return None
        if self.state == 'paused':
            self.reference, self.last, self.exposure = q.copy(), now, None
            self.pause_since, self.reason, self.state = None, None, 'moving'
            return q.copy()
        dt = now-self.last
        self.last = now
        if not 0 < dt <= .1:
            self.state, self.reason = 'failed', 'clock_gap'
            return None
        self.reference += np.clip(self.goal-self.reference, -.10*dt, .10*dt)
        settled = np.max(np.abs(q-self.goal)) < .025 and np.max(np.abs(velocity)) < .02
        if settled:
            if self.exposure is None:
                self.exposure = now
            if self.waypoint_index < len(self.waypoints)-1:
                if now-self.exposure >= .5:
                    self.waypoint_index+=1;self.goal=self.waypoints[self.waypoint_index];self.exposure=None
                self.state='moving'
                return self.reference.copy()
            self.state = 'observing'
            if now-self.exposure >= 5.:
                self.state = 'complete'
        else:
            self.exposure = None
            self.state = 'moving'
        return self.reference.copy()
