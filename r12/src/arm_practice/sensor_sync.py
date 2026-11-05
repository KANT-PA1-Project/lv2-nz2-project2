"""Bounded simulation transport wait; does not relax sensor stamp freshness."""
import time


def wait_visual_pair(stamp, status, odom, spin, timeout=1., clock=time.monotonic):
    start = clock()
    deadline = start + timeout
    while stamp not in status or stamp not in odom:
        remaining = deadline - clock()
        if remaining <= 0:
            break
        spin(min(.005, remaining))
    return {'wait_seconds': clock()-start, 'status_received': stamp in status,
            'odom_received': stamp in odom,
            'timed_out': stamp not in status or stamp not in odom}
