"""Same-host perception response validation; never motion authorization."""
CAMERAS = frozenset(('r07_camera1', 'r07_camera2', 'r07_camera3', 'arm_camera'))


def check_perception_response(request, response, *, now_ns, max_age_ns):
    """Request identity must originate from the loaded frame, not just its filename.

    Monotonic times must come from the same host/boot. Call again at consumption;
    a successful check does not keep a cached response fresh indefinitely.
    """
    result = {'matched': False, 'live_freshness_verified': False,
              'execution_authorized': False}
    try:
        if type(now_ns) is not int or type(max_age_ns) is not int or max_age_ns <= 0:
            return dict(result, reason='invalid_clock')
        for obj in (request, response):
            if not isinstance(obj, dict):
                return dict(result, reason='invalid_metadata')
            if type(obj['run_id']) is not str or not obj['run_id']:
                return dict(result, reason='invalid_run_id')
            if obj['clock_domain'] != 'synthetic_packet_20hz':
                return dict(result, reason='unsupported_clock')
            if type(obj['camera']) is not str or obj['camera'] not in CAMERAS:
                return dict(result, reason='invalid_camera')
            for key in ('frame_index', 'stamp_ns', 'capture_monotonic_ns'):
                if type(obj[key]) is not int or obj[key] < 0:
                    return dict(result, reason='invalid_metadata')
            if obj['stamp_ns'] != 1_000_000_000 + obj['frame_index'] * 50_000_000:
                return dict(result, reason='invalid_frame_stamp')
        keys = ('run_id', 'clock_domain', 'camera', 'frame_index', 'stamp_ns', 'capture_monotonic_ns')
        if any(request[k] != response[k] for k in keys):
            return dict(result, reason='response_identity_mismatch')
        start = response['inference_started_monotonic_ns']
        done = response['inference_completed_monotonic_ns']
        capture = response['capture_monotonic_ns']
        if type(start) is not int or type(done) is not int:
            return dict(result, reason='invalid_clock')
        if not 0 <= capture <= start <= done <= now_ns:
            return dict(result, reason='clock_order_mismatch')
        age = now_ns - capture
        timing = {'age_ns': age, 'inference_duration_ns': done - start,
                  'capture_to_completion_ns': done - capture}
        if age > max_age_ns:
            return dict(result, reason='stale_response', **timing)
        return dict(result, matched=True, live_freshness_verified=True,
                    reason='same_host_response_provenance_only', **timing)
    except (KeyError, TypeError, ValueError, OverflowError):
        return dict(result, reason='missing_or_invalid_metadata')


def check_packet_perception(packet, *, now_ns, max_age_ns):
    """Validate against the current sensor packet, ignoring sender verdicts.

    Cached results from an earlier frame are not observations of the current pose.
    """
    try:
        stamp = packet['stamp_ns']
        if type(stamp) is not int or stamp < 1_000_000_000 or (stamp-1_000_000_000)%50_000_000:
            raise ValueError('invalid packet stamp')
        frame = (stamp-1_000_000_000)//50_000_000
        camera = ('r07_camera1','r07_camera2','r07_camera3','arm_camera')[(frame//20)%4]
        request = dict(run_id=packet['capture_run_id'], clock_domain='synthetic_packet_20hz',
                       camera=camera, frame_index=frame, stamp_ns=stamp,
                       capture_monotonic_ns=packet['frame_capture_monotonic_ns'])
        return check_perception_response(request, packet.get('perception',{}),
                                         now_ns=now_ns, max_age_ns=max_age_ns)
    except (KeyError, TypeError, ValueError, OverflowError):
        return dict(matched=False, live_freshness_verified=False,
                    execution_authorized=False, reason='invalid_sensor_packet_identity')
