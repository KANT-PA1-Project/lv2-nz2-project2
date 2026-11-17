import math
from arm_practice.goal_stop_hold import GoalStopHold
from arm_practice.stationary_observation_dwell import StationaryObservationDwell


def test_capture_requires_original_tolerance_and_low_speed():
    h = GoalStopHold()
    assert not h.update(0, 0, .021, .01, False, True)['active']
    assert not h.update(1, 0, .019, .015, False, True)['active']
    assert h.update(2, 0, .019, .014, False, True)['force_zero_command']


def test_braking_does_not_restart_follower_on_boundary_crossing():
    h = GoalStopHold()
    h.update(0, 0, .019, .01, False, True)
    assert h.update(50_000_000, 0, .021, .02, False, True)['active']
    assert h.update(100_000_000, 0, .021, 0., True, True)['reason'] == 'reapproach'
    assert not h.active


def test_sensor_loss_keeps_stop_but_revokes_hold_torque():
    h = GoalStopHold()
    h.update(0, 0, .01, .01, False, True)
    s = h.update(50_000_000, 0, math.nan, .01, False, False)
    assert s['force_zero_command'] and not s['position_hold_allowed']
    assert h.update(100_000_000, 0, .01, 0., True, True)['position_hold_allowed']


def test_goal_change_and_explicit_reset_release_anchor_authorization():
    h = GoalStopHold()
    h.update(0, 0, .01, .01, False, True)
    assert not h.update(50_000_000, 1, 1., .01, False, True)['active']
    h.update(100_000_000, 1, .01, .01, True, True)
    h.reset()
    assert not h.active


def test_repeated_stamp_cannot_authorize_hold():
    h = GoalStopHold()
    h.update(0, 0, .01, .01, False, True)
    assert not h.update(0, 0, .01, .01, False, True)['position_hold_allowed']


def test_timeout_fails_closed_and_still_stops():
    h = GoalStopHold(timeout_s=1.)
    h.update(0, 0, .01, .01, False, True)
    s = h.update(1_100_000_000, 0, .01, .01, False, True)
    assert s['failed'] and s['force_zero_command']


def test_stop_latch_is_not_dwell_acceptance():
    h = GoalStopHold()
    dwell = StationaryObservationDwell(4.5)
    for i in range(100):
        h.update(i * 50_000_000, 0, .01, .01, False, True)
        state = dwell.update(i * 50_000_000, False, 'arm_camera')
    assert h.active and not state['complete'] and state['elapsed_s'] == 0


def test_inner_capture_keeps_arrival_radius_and_reapproach_threshold():
    h = GoalStopHold(capture_radius_m=.01)
    assert not h.update(0, 0, .019, .01, False, True)['active']
    assert h.update(50_000_000, 0, .009, .01, False, True)['active']
    state = h.update(100_000_000, 0, .016, 0., True, True)
    assert state['active'] and state['arrival_radius_m'] == .02
    assert state['capture_radius_m'] == .01
    assert not h.update(150_000_000, 0, .021, 0., True, True)['active']


def test_inner_capture_rejects_invalid_or_relaxed_threshold():
    import pytest
    for radius in [0., -.01, .021, float('nan'), float('inf')]:
        with pytest.raises(ValueError):
            GoalStopHold(capture_radius_m=radius)
