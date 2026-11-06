"""Schedule-to-identity regression; synthetic evidence is not runtime acceptance."""
import pytest

from arm_practice.perception_schedule import select_perception_work
from arm_practice.wrist_identity import WristIdentity


READY = dict(wrist_view_ready=True, feedback_fresh=True,
             base_stopped=True, arm_settled=True)


def observe(gate, frame):
    candidate = dict(id=2, color='R', shape='cylinder',
                     center_map_m=[1., 2., .1], depth_surface_m=.3,
                     depth_consistent=True,
                     pose_camera=dict(center_camera_m=[0., 0., .3],
                                      translation_valid=True, confidence=.9))
    return gate.observe(candidate, camera='arm_camera',
                        camera_frame='arm_camera_left_optical', run_id='test',
                        stamp_ns=1_000_000_000 + frame * 50_000_000,
                        fresh=True, view_ready=True, stationary=True,
                        arm_velocity=[0.] * 5)


def test_survey_rotation_cannot_satisfy_repeated_wrist_gate():
    gate = WristIdentity()
    wrist_frames = [f for f in range(240)
                    if (w := select_perception_work(f))['detect']
                    and w['camera'] == 'arm_camera']
    assert wrist_frames == [60, 140, 220]
    assert all(observe(gate, f) is None for f in wrist_frames)


def test_ready_burst_can_supply_distinct_close_source_frames():
    gate = WristIdentity()
    for f in (60, 61, 62):
        work = select_perception_work(f, **READY)
        assert work['camera'] == 'arm_camera' and work['detect']
        result = observe(gate, f)
        if f < 62:
            assert result is None
    assert result['wrist_consistent_frames'] == 3
    assert result['grasp_pose_accepted'] is False


@pytest.mark.parametrize('missing', list(READY))
def test_readiness_loss_stops_burst_immediately(missing):
    assert select_perception_work(61, **READY)['mode'] == 'wrist_confirmation'
    for value in (False, None, 1, 'true'):
        invalid = dict(READY, **{missing: value})
        assert select_perception_work(62, **invalid) == select_perception_work(62)


def test_legacy_survey_schedule_is_unchanged_without_ready_view():
    cameras = ['r07_camera1', 'r07_camera2', 'r07_camera3', 'arm_camera']
    for f in range(240):
        work = select_perception_work(f)
        assert work['camera'] == cameras[(f // 20) % 4]
        assert work['detect'] == (f % 20 == 0)


@pytest.mark.parametrize('invalid', [-1, 1.0, True, None])
def test_bad_frame_is_rejected(invalid):
    with pytest.raises(ValueError):
        select_perception_work(invalid, **READY)
