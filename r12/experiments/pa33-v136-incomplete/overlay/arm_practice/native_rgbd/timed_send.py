"""공유 core send_packet은 수정하지 않고 v136 쪽에서만 송신 시간을 기록한다."""
import time
from .trusted_capture_ipc import send_packet as _core_send_packet

def send_packet(sock, data, blobs=(), *, timing=None, **kwargs):
    if timing is None:
        return _core_send_packet(sock, data, blobs, **kwargs)
    timing['serialization_start_ns'] = time.monotonic_ns()
    timing['blob_bytes'] = sum(len(b) for b in blobs)
    _core_send_packet(sock, data, blobs, **kwargs)
    timing['unix_send_end_ns'] = time.monotonic_ns()