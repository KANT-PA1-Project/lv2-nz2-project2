"""Sensor-verified dwell and camera coverage; never certifies object identity."""
import math


class StationaryObservationDwell:
    cameras = frozenset(('r07_camera1', 'r07_camera2', 'r07_camera3', 'arm_camera'))

    def __init__(self, seconds=4.5, max_gap_ns=100_000_000):
        if not math.isfinite(seconds) or not .5 <= seconds <= 30:
            raise ValueError('Observation dwell must be finite and between 0.5 and 30 seconds')
        if not isinstance(max_gap_ns, int) or max_gap_ns <= 0:
            raise ValueError('Invalid observation gap')
        self.duration_ns = int(seconds * 1e9)
        self.max_gap_ns = max_gap_ns
        self.reset()

    def reset(self):
        self.started_ns = None
        self.previous_ns = None
        self.seen = set()

    def update(self, stamp_ns, stationary, camera=None):
        """Caller supplies current fresh sensor standstill and a fresh detection camera.

        Invalid localization, movement, lost transport posture, or missing/current-
        stamp perception must not be promoted to stationary/covered by the caller.
        A camera with zero detections still counts as inspected, not recognized.
        """
        if not isinstance(stamp_ns, int) or isinstance(stamp_ns, bool) or stamp_ns < 0:
            self.reset()
            return self.status('invalid_stamp')
        if not stationary:
            self.reset()
            return self.status('await_standstill')
        if self.previous_ns is not None and stamp_ns <= self.previous_ns:
            self.reset()
            return self.status('non_increasing_stamp')
        if self.previous_ns is not None and stamp_ns - self.previous_ns > self.max_gap_ns:
            self.reset()
        if self.started_ns is None:
            self.started_ns = stamp_ns
        self.previous_ns = stamp_ns
        if camera in self.cameras:
            self.seen.add(camera)
        return self.status('observing')

    def status(self, reason):
        elapsed = 0 if self.started_ns is None else self.previous_ns - self.started_ns
        complete = elapsed >= self.duration_ns and self.seen == self.cameras
        return dict(complete=complete, reason='coverage_complete' if complete else reason,
                    elapsed_s=elapsed / 1e9, required_s=self.duration_ns / 1e9,
                    inspected_cameras=sorted(self.seen),
                    scope='Stationary camera inspection only; not object identity or pose acceptance')
