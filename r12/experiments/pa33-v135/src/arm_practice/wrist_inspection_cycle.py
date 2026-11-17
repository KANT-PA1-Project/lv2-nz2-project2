"""One stationary view-and-refold cycle. This is not a collision checker.

The caller supplies a current sensor-based sweep checker for each proposed
measured-position-to-reference interval. Missing or rejected checks cannot
produce an arm command. A completed cycle can still have no confirmed identity.
"""
import copy
import numpy as np


class WristInspectionCycle:
    def __init__(self, goal, lower, upper, *, clearance_check=None,
                 exposure_seconds=5., timeout_seconds=40.):
        self.goal = np.asarray(goal, float)
        self.lower, self.upper = np.asarray(lower, float), np.asarray(upper, float)
        if (any(x.shape != (5,) for x in (self.goal, self.lower, self.upper))
                or not np.isfinite(np.r_[self.goal, self.lower, self.upper]).all()
                or np.any(self.lower > self.upper) or np.any(self.goal < self.lower)
                or np.any(self.goal > self.upper)):
            raise ValueError('Invalid wrist posture or limits')
        if (not np.isfinite([exposure_seconds, timeout_seconds]).all()
                or exposure_seconds <= 0 or timeout_seconds <= exposure_seconds + 1):
            raise ValueError('Invalid observation duration')
        if clearance_check is not None and not callable(clearance_check):
            raise ValueError('A live interval checker is required')
        self.clearance_check = clearance_check
        self.exposure_seconds, self.timeout_seconds = exposure_seconds, timeout_seconds
        self.state, self.reason = 'waiting', 'await_stationary_feedback'
        self.last = self.started = self.stable = self.exposure = self.clearance_since = None
        self.home = self.anchor = self.reference = None
        self.confirmed = {}
        self.run_id = None
        self.waypoint_index = 0

    @property
    def done(self):
        return self.state in ('complete', 'failed')

    @property
    def view_ready(self):
        return self.state == 'observing'

    @property
    def inspection_passed(self):
        return self.state == 'complete' and bool(self.confirmed)

    def _fail(self, reason):
        self.state, self.reason = 'failed', reason
        return None

    def _checked(self, q, target, direction):
        if self.clearance_check is None:
            return False
        try:
            # Only an explicit bool True is accepted; no truthy report/dict.
            return self.clearance_check(q.copy(), target.copy(), direction) is True
        except Exception:
            return False

    def step(self, now, q, velocity, pose, stopped, valid, *, run_id=None,
             confirmations=(), invalidated_ids=()):
        if self.done:
            return None
        for pid in invalidated_ids:
            self.confirmed.pop(pid, None)
        q, velocity, pose = (np.asarray(x, float) for x in (q, velocity, pose))
        good = (valid is True and type(run_id) is str and bool(run_id)
                and q.shape == velocity.shape == (5,) and pose.shape == (3,)
                and np.isfinite(np.r_[now, q, velocity, pose]).all()
                and np.all(q >= self.lower) and np.all(q <= self.upper))
        if not good or stopped is not True:
            self.stable = None
            if self.home is not None:
                return self._fail('sensor_invalid' if not good else 'base_moved')
            self.reason = 'await_stationary_feedback'
            return None
        if self.started is None:
            self.started = now
            self.run_id = run_id
        if run_id != self.run_id:
            return self._fail('capture_run_changed')
        if now - self.started > self.timeout_seconds:
            return self._fail('cycle_timeout')
        dt = None if self.last is None else now - self.last
        self.last = now
        if dt is not None and not 0 < dt <= .100000001:
            return self._fail('clock_gap')
        if self.home is None:
            if np.max(np.abs(velocity)) >= .02:
                self.stable = None
                return None
            if self.stable is None:
                self.stable = now
            if now - self.stable < .5:
                return None
            self.home, self.reference, self.anchor = q.copy(), q.copy(), pose.copy()
            self.stable = None
            self.state, self.reason = 'waiting_clearance', 'await_live_clearance'
            self.clearance_since = now
        dyaw = np.arctan2(np.sin(pose[2]-self.anchor[2]), np.cos(pose[2]-self.anchor[2]))
        if np.linalg.norm(pose[:2]-self.anchor[:2]) > .015 or abs(dyaw) > .03:
            return self._fail('base_pose_drift')
        if np.max(np.abs(q-self.reference)) > .025:
            return self._fail('arm_tracking_error')
        returning = self.state == 'returning'
        endpoint = self.home if returning else self.goal
        target = self.reference + np.clip(endpoint-self.reference, -.10*dt, .10*dt)
        direction = 'return' if returning else 'approach'
        if not self._checked(q, target, direction):
            if self.state != 'waiting_clearance':
                return self._fail('clearance_lost')
            if now - self.clearance_since >= 2.:
                return self._fail('clearance_unavailable')
            return None
        if self.state == 'waiting_clearance':
            self.state, self.reason = 'moving', None
        self.reference = target
        settled = (np.max(np.abs(q-endpoint)) < .025
                   and np.max(np.abs(velocity)) < .02
                   and np.max(np.abs(self.reference-endpoint)) < 1e-9)
        if not settled:
            self.stable = self.exposure = None
            if self.state == 'observing':
                self.state = 'moving'
                self.confirmed.clear()
            return target.copy()
        if self.stable is None:
            self.stable = now
        if now - self.stable < .5:
            return target.copy()
        if returning:
            self.state = 'complete'
            self.reason = 'identified_and_refolded' if self.confirmed else 'refolded_without_identity'
            return target.copy()
        if self.state != 'observing':
            self.state, self.exposure = 'observing', now
        else:
            # Only same-frame results produced by WristIdentity may count.
            for item in confirmations:
                if (item.get('camera') == 'arm_camera'
                        and item.get('camera_frame') == 'arm_camera_left_optical'
                        and item.get('evidence_role') == 'wrist_confirmed_identity_with_pose_estimate'
                        and item.get('run_id') == self.run_id
                        and item.get('wrist_consistent_frames', 0) >= 3
                        and type(item.get('stamp_ns')) is int
                        and abs(item['stamp_ns']-round(now*1e9)) <= 1
                        and type(item.get('id')) is int and 0 <= item['id'] < 12):
                    self.confirmed[item['id']] = copy.deepcopy(item)
        if now-self.exposure >= self.exposure_seconds:
            self.state, self.reason = 'returning', None
            self.waypoint_index = 1
            self.stable = self.exposure = None
        return target.copy()

    def status(self):
        return dict(state=self.state, reason=self.reason,
                    view_ready=self.view_ready, confirmed_ids=sorted(self.confirmed),
                    refold_settled=self.state == 'complete',
                    inspection_passed=self.inspection_passed,
                    grasp_started=False, grasp_pose_accepted=False)
