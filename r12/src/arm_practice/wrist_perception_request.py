"""Use the last fresh settled response to schedule wrist inference work only.

The next response must independently validate current feedback and identity.
Scheduling extra inference never authorizes motion or confirms an object.
"""
import numpy as np


def wrist_schedule_feedback(response, *, run_id, stamp_ns, now_ns):
    no = dict(wrist_view_ready=False, feedback_fresh=False,
              base_stopped=False, arm_settled=False)
    try:
        previous = response['stamp_ns']
        capture = response['frame_capture_monotonic_ns']
        arm = np.asarray(response['arm_measured_velocity'], float)
        base = response['base_sensor_feedback']
        twist = np.asarray(base['twist'], float)
        command = np.asarray(response['command'], float)
        valid = (type(stamp_ns) is int and type(previous) is int
                 and type(now_ns) is int and type(capture) is int
                 and type(run_id) is str and bool(run_id)
                 and stamp_ns-previous == 50_000_000 and 0 <= now_ns-capture <= 500_000_000
                 and response['capture_run_id'] == base['run_id'] == run_id
                 and base['stamp_ns'] == response['arm_feedback_stamp_ns'] == previous
                 and base['capture_monotonic_ns'] == capture
                 and base['valid'] is True and response['arm_feedback_valid'] is True
                 and response['base_stopped'] is True
                 and response['observation_state'] == 'observing'
                 and response['observation_sensor_checks']['fresh'] is True
                 and response['stationary_wheel_hold_enabled'] is True
                 and arm.shape == (5,) and twist.shape == command.shape == (3,)
                 and np.isfinite(np.r_[arm, twist, command]).all()
                 and np.max(np.abs(arm)) < .02 and np.linalg.norm(twist[:2]) < .01
                 and abs(twist[2]) < .02 and np.max(np.abs(command)) < 1e-9)
    except (KeyError, TypeError, ValueError):
        valid = False
    return {key: bool(valid) for key in no}
