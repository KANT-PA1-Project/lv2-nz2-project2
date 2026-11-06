"""Authorize existing encoder position hold during a safe planner stop.

No desired pose, simulator truth or extra torque is introduced. This prevents
sub-encoder-quantum creep while a healthy planner waits for a clear reference.
"""
import numpy as np


def planner_stop_hold_allowed(command, path_state, *, healthy, transport_ok,
                              base_stopped, guard, visual_tilt, motion_state):
    if not healthy or not transport_ok or not base_stopped or guard is not None:
        return False
    if not isinstance(path_state, dict) or not isinstance(motion_state, dict):
        return False
    if motion_state.get('latched') or motion_state.get('reason') not in (None, 'limited'):
        return False
    cmd, tilt = np.asarray(command, float), np.asarray(visual_tilt, float)
    if cmd.shape != (3,) or tilt.shape != (2,) or not np.isfinite(np.r_[cmd,tilt]).all():
        return False
    if np.linalg.norm(cmd) >= 1e-6 or np.max(abs(tilt)) >= np.deg2rad(4):
        return False
    if path_state.get('reason') == 'oriented_reference':
        # A live-map zero-motion footprint check is required in addition to
        # every existing healthy/transport/stopped/tilt/torque gate above.
        return (path_state.get('oriented_hold_eligible') is True
                and path_state.get('oriented_phase') == 'await_stop')
    return (path_state.get('reason') == 'continuous_reference'
            and path_state.get('preview_scale') == 0.
            and path_state.get('preview_reason') in
            ('filtered_rollout_clear', 'no_filtered_rollout_clear'))
