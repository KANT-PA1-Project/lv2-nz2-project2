"""Sensor-only settling gate for terminal archives, not motion authorization."""
import numpy as np

class TerminalObservation:
    def __init__(self):
        self.since = None
        self.previous = None

    def update(self, packet, now_ns):
        stamp = packet.get('stamp_ns')
        b = packet.get('base_sensor_feedback') or {}
        v = np.asarray(packet.get('arm_measured_velocity'), dtype=float)
        twist = np.asarray(b.get('twist'), dtype=float)
        capture = packet.get('frame_capture_monotonic_ns')
        valid = (isinstance(stamp, int) and isinstance(capture, int)
                 and 0 <= now_ns-capture <= 250_000_000
                 and packet.get('finished', False) and packet.get('arm_feedback_valid', False)
                 and b.get('valid', False) and b.get('stamp_ns') == stamp
                 and packet.get('arm_feedback_stamp_ns') == stamp
                 and b.get('run_id') == packet.get('capture_run_id') != None
                 and b.get('capture_monotonic_ns') == capture
                 and v.shape == (5,) and twist.shape == (3,)
                 and np.isfinite(v).all() and np.isfinite(twist).all()
                 and np.max(np.abs(v)) < .02 and np.linalg.norm(twist[:2]) < .01
                 and abs(twist[2]) < .02)
        contiguous = self.previous is not None and isinstance(stamp, int) and stamp-self.previous == 50_000_000
        if not valid:
            self.since = None
        elif self.since is None or not contiguous:
            self.since = stamp
        self.previous = stamp
        settled = bool(valid and stamp-self.since >= 500_000_000)
        return {'settled': settled, 'execution_authorized': False,
                'checked_monotonic_ns': now_ns,
                'capture_age_ns': now_ns-capture if isinstance(capture, int) else None,
                'reason': 'stationary_fresh_0.5s' if settled else 'await_stationary_fresh_feedback'}
