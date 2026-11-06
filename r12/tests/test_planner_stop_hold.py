import numpy as np
from arm_practice.planner_stop_hold import planner_stop_hold_allowed
from arm_practice.wheel_position_hold import WheelPositionHold


def allowed(**changes):
    args=dict(command=[0.,0.,0.], path_state=dict(reason='continuous_reference',preview_scale=0.,preview_reason='filtered_rollout_clear'),
              healthy=True, transport_ok=True, base_stopped=True, guard=None,
              visual_tilt=[0.,0.], motion_state=dict(reason='limited',latched=False))
    args.update(changes)
    return planner_stop_hold_allowed(**args)


def test_safe_planner_stop_holds_encoder_anchor_without_velocity_command():
    hold=WheelPositionHold()
    np.testing.assert_array_equal(hold.step([1.]*4,allowed()),[0.]*4)
    np.testing.assert_allclose(hold.step([1.01]*4,allowed()),[-.003]*4)


def test_no_hold_after_safety_veto_or_sensor_failure():
    for change in [dict(guard='rotation_sweep_blocked'),dict(guard='invalid_sensor'),dict(healthy=False),
                   dict(motion_state={'latched':True}),dict(transport_ok=False),dict(base_stopped=False)]:
        assert not allowed(**change)


def test_does_not_hold_during_drive_or_yaw_command():
    for command in [[.01,0,0],[0,.01,0],[0,0,.01]]:
        assert not allowed(command=command)


def test_invalid_tilt_and_commands_fail_closed():
    for change in [dict(visual_tilt=None),dict(visual_tilt=[np.nan,0]),dict(visual_tilt=[np.deg2rad(4),0]),
                   dict(command=[np.nan,0,0]),dict(command=[0,0])]:
        assert not allowed(**change)


def test_no_implicit_hold_without_current_navigation_reference():
    assert not allowed(path_state=None)
    assert not allowed(path_state={'reason':'continuous_reference','preview_scale':1.,'preview_reason':'filtered_rollout_clear'})


def test_oriented_wait_requires_live_clearance_and_retains_safety_interlocks():
    state=dict(reason='oriented_reference',oriented_hold_eligible=True,oriented_phase='await_stop')
    assert allowed(path_state=state)
    for change in [dict(healthy=False),dict(transport_ok=False),dict(base_stopped=False),
                   dict(guard='oriented_live_sweep_blocked'),dict(command=[0,0,.01]),
                   dict(visual_tilt=[np.deg2rad(4),0]),dict(motion_state={'latched':True})]:
        assert not allowed(path_state=state,**change)
    for phase in ['prepare_heading','tracking',None]:
        assert not allowed(path_state=dict(state,oriented_phase=phase))
    for clearance in [False,None,1,'true']:
        assert not allowed(path_state=dict(state,oriented_hold_eligible=clearance))
