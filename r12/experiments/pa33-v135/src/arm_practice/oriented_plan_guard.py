"""Freshness/version boundary for asynchronous oriented-path candidates.

Acceptance only permits handing a candidate to the existing heading, live
occupancy, motion-envelope and actuator gates. It never authorizes motion.
All time values must be from the same monotonic host clock.
"""
import math


def accept_oriented_plan(plan, request, active_request, sensor_pose, *,
                         now_s, sensor_stamp_s, max_sensor_age_s=.15,
                         max_plan_age_s=5.):
    def verdict(ok, reason):
        return {'accepted': ok, 'reason': reason, 'motion_authorized': False}

    keys = ('request_id', 'goal_index', 'map_revision', 'path_revision',
            'footprint_revision')
    if not isinstance(request, dict) or not isinstance(active_request, dict):
        return verdict(False, 'missing_request')
    if any(k not in request or request[k] is None or request[k] == '' for k in keys):
        return verdict(False, 'incomplete_request_identity')
    if any(request[k] != active_request.get(k) for k in keys):
        return verdict(False, 'superseded_request')
    try:
        created = float(request['created_monotonic_s'])
        times = [float(x) for x in (now_s, sensor_stamp_s, max_sensor_age_s, max_plan_age_s)]
        now, stamp, sensor_limit, plan_limit = times
        if not all(math.isfinite(x) for x in [created, *times]) or min(sensor_limit, plan_limit) <= 0:
            return verdict(False, 'invalid_clock')
        if not 0 <= now-stamp <= sensor_limit:
            return verdict(False, 'sensor_age_invalid')
        if not 0 <= now-created <= plan_limit:
            return verdict(False, 'plan_age_invalid')
        if not isinstance(plan, dict) or plan.get('ok') is not True:
            return verdict(False, 'invalid_plan')
        origin = [float(x) for x in plan['preparation_origin']]
        pose = [float(x) for x in sensor_pose]
        source_pose = [float(x) for x in request['sensor_pose']]
        margin = float(plan['position_margin_m'])
        yaw_limit = float(plan['yaw_error_rad'])
        if len(origin) != 2 or len(pose) != 3 or len(source_pose) != 3:
            return verdict(False, 'invalid_pose_shape')
        if not all(math.isfinite(x) for x in [*origin, *pose, *source_pose, margin, yaw_limit]):
            return verdict(False, 'nonfinite_pose_or_margin')
        if margin <= 0 or not 0 < yaw_limit <= math.pi/4:
            return verdict(False, 'invalid_margin')
        if math.hypot(origin[0]-source_pose[0], origin[1]-source_pose[1]) > 1e-8:
            return verdict(False, 'plan_origin_mismatch')
        if math.hypot(pose[0]-origin[0], pose[1]-origin[1]) > margin:
            return verdict(False, 'preparation_origin_drift')
        dyaw = math.atan2(math.sin(pose[2]-source_pose[2]), math.cos(pose[2]-source_pose[2]))
        if abs(dyaw) > yaw_limit:
            return verdict(False, 'preparation_heading_drift')
    except (KeyError, TypeError, ValueError, OverflowError):
        return verdict(False, 'malformed_candidate')
    return verdict(True, 'candidate_current_for_live_gate_checks')
