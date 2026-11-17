"""Same-host acquisition freshness for image-derived signal colors."""

def checked_signal_color(packet, *, now_ns, max_age_ns):
    try:
        obs = packet['signal_observation']
        capture = packet['frame_capture_monotonic_ns']
        stamp = packet['stamp_ns']
        run = packet['capture_run_id']
        if type(now_ns) is not int or type(max_age_ns) is not int or max_age_ns <= 0:
            return 'UNKNOWN'
        if type(capture) is not int or not 0 <= now_ns-capture <= max_age_ns:
            return 'UNKNOWN'
        if type(stamp) is not int or stamp < 1000000000 or (stamp-1000000000)%50000000:
            return 'UNKNOWN'
        if type(run) is not str or not run or obs['run_id'] != run:
            return 'UNKNOWN'
        if type(obs['stamp_ns']) is not int or obs['stamp_ns'] != stamp:
            return 'UNKNOWN'
        if type(obs['capture_monotonic_ns']) is not int or obs['capture_monotonic_ns'] != capture:
            return 'UNKNOWN'
        if obs['camera'] != 'r07_camera1' or obs['clock_domain'] != 'synthetic_packet_20hz':
            return 'UNKNOWN'
        # ROI comes from the top camera barcode pass, even when the same worker
        # response contains detections from a different selected camera.
        roi = obs['roi_source']
        roi_stamp = roi['stamp_ns']
        roi_capture = roi['capture_monotonic_ns']
        if roi['run_id'] != run or roi['camera'] != 'r07_camera1':
            return 'UNKNOWN'
        if type(roi_stamp) is not int or not 1000000000 <= roi_stamp <= stamp or (roi_stamp-1000000000)%50000000:
            return 'UNKNOWN'
        if type(roi_capture) is not int or not 0 <= roi_capture <= capture or not 0 <= now_ns-roi_capture <= max_age_ns:
            return 'UNKNOWN'
        if roi_stamp == stamp and roi_capture != capture:
            return 'UNKNOWN'
        color = obs['color']
        return color if color in ('R','Y','G','OFF') else 'UNKNOWN'
    except (KeyError, TypeError, ValueError):
        return 'UNKNOWN'
