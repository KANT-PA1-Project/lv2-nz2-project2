"""Camera work selection for a future per-station wrist inspection integration.

This selector does not authorize arm motion or accept identity. Callers must
derive readiness from current, validated sensor feedback and the collision-
checked observation state. Existing capture provenance/age checks still apply
to every response. Source-frame cadence is separate from wall-clock latency.
"""


def select_perception_work(frame_index, *, wrist_view_ready=False,
                           feedback_fresh=False, base_stopped=False,
                           arm_settled=False):
    if type(frame_index) is not int or frame_index < 0:
        raise ValueError('frame_index must be a nonnegative integer')
    # Missing/invalid readiness preserves survey behavior; it must never
    # manufacture an inspection opportunity from a route arrival alone.
    wrist_burst = all(flag is True for flag in (
        wrist_view_ready, feedback_fresh, base_stopped, arm_settled))
    if wrist_burst:
        return dict(camera='arm_camera', detect=True, mode='wrist_confirmation')
    return dict(camera=('r07_camera1', 'r07_camera2', 'r07_camera3', 'arm_camera')[
                    (frame_index // 20) % 4],
                detect=frame_index % 20 == 0, mode='survey')
