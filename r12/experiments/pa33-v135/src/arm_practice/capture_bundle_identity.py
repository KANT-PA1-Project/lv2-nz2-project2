"""Same-host NPZ/packet correspondence only; no SLAM or motion authorization."""
import hashlib
import numpy as np


def inspect_capture_bundle(data, packet):
    """Read identity from the opened NPZ and hash the six upper RGB arrays.

    Caller must publish these exact arrays after validation. Hashes cannot prove
    which inputs cuVSLAM consumed, source authenticity, or capture freshness.
    """
    result = dict(matched=False, source_session_verified=False,
                  capture_freshness_verified=False, pose_join_verified=False,
                  execution_authorized=False)
    try:
        def scalar(key, kind):
            a = np.asarray(data[key])
            if a.shape != () or a.dtype.kind not in kind:
                raise ValueError('invalid_metadata')
            return a.item()
        run = scalar('run_id', 'U'); clock = scalar('clock_domain', 'U')
        frame = scalar('frame_index', 'iu'); stamp = scalar('stamp_ns', 'iu')
        capture = scalar('capture_monotonic_ns', 'iu')
        if not run or clock != 'synthetic_packet_20hz' or min(frame, stamp, capture) < 0:
            raise ValueError('invalid_metadata')
        if stamp != 1_000_000_000 + frame * 50_000_000:
            raise ValueError('invalid_frame_stamp')
        for key in ('stamp_ns', 'frame_capture_monotonic_ns'):
            if type(packet[key]) is not int:
                raise ValueError('invalid_packet_metadata')
        if type(packet['capture_run_id']) is not str or (run, stamp, capture) != (packet['capture_run_id'], packet['stamp_ns'], packet['frame_capture_monotonic_ns']):
            raise ValueError('bundle_packet_mismatch')
        rgb = np.asarray(data['rgb'])
        if rgb.dtype != np.uint8 or rgb.shape != (2, 480, 640, 3):
            raise ValueError('invalid_rgb')
        arrays = {f'r07_camera1_{side}_optical': rgb[i] for i, side in enumerate(('left', 'right'))}
        arrays.update({f'r07_camera{c}_{side}_optical': np.asarray(data[f'r07_camera{c}_{side}_optical']) for c in (2, 3) for side in ('left', 'right')})
        if any(a.dtype != np.uint8 or a.shape != (480, 640, 3) for a in arrays.values()):
            raise ValueError('invalid_rgb')
        return dict(result, matched=True, reason='bundle_packet_correspondence_only',
                    run_id=run, frame_index=frame, stamp_ns=stamp, capture_monotonic_ns=capture,
                    image_sha256={k: hashlib.sha256(a.tobytes()).hexdigest() for k, a in arrays.items()})
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        return dict(result, reason=str(exc))


class CaptureSessionLedger:
    """Constant-space, single-threaded diagnostics; new instance opens a session.

    Pin run and local generation before ingestion. A rejected current-generation
    bundle latches the session closed; delayed callbacks from an old generation
    are rejected without altering the new session. Skipped frames are allowed.
    """
    def __init__(self, run_id, generation):
        if any(type(v) is not str or not v for v in (run_id, generation)):
            raise ValueError('invalid_session_identity')
        self._run_id = run_id
        self._generation = generation
        self._last = None
        self.faulted = False

    def inspect(self, data, packet, *, generation):
        denied = dict(matched=False, sequence_eligible=False,
                      source_session_verified=False, capture_freshness_verified=False,
                      pose_join_verified=False, execution_authorized=False)
        if generation != self._generation:
            return dict(denied, reason='old_callback_generation')
        if self.faulted:
            return dict(denied, reason='session_faulted_requires_new_instance')
        result = inspect_capture_bundle(data, packet)
        reason = None
        if not result['matched']:
            reason = result['reason']
        elif result['run_id'] != self._run_id:
            reason = 'capture_run_changed'
        else:
            current = (result['frame_index'], result['stamp_ns'], result['capture_monotonic_ns'])
            if self._last is not None and any(a <= b for a, b in zip(current, self._last)):
                reason = 'duplicate_or_regressing_capture'
        if reason is not None:
            self.faulted = True
            return dict(denied, reason=reason)
        self._last = current
        return dict(result, sequence_eligible=True, reason='local_capture_sequence_only')
