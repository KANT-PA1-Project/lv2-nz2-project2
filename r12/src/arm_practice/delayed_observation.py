"""Isolated historical response binding; not integrated with live consumers."""
from copy import deepcopy
from .perception_response_identity import check_perception_response


def bind_historical_response(request, response, *, received_ns, max_age_ns):
    """Validate at receipt against the original request, never a newer packet.

    This snapshot records provenance at receipt only. It cannot establish current
    freshness, capture-pose association, or permission to move. No pose is
    inferred from fixture geometry, a latest pose, or an optional visual_pose.
    """
    verdict = check_perception_response(request, response, now_ns=received_ns,
                                        max_age_ns=max_age_ns)
    if not verdict['matched']:
        raise ValueError(verdict['reason'])
    return dict(request=deepcopy(request), response=deepcopy(response),
                received_monotonic_ns=received_ns,
                provenance_at_receipt=deepcopy(verdict),
                current_observation_eligible=False, pose_associated=False,
                execution_authorized=False,
                scope='historical_response_only_requires_time_aligned_sensor_pose')
