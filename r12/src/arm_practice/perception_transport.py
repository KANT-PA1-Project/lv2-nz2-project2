"""Bounded same-host JSON transport; no model or motion logic."""
import json
import math
import os
import selectors
import time
import numpy as np
from .perception_response_identity import check_perception_response


def frame_identity(frame, camera):
    identity = {'camera': camera}
    for key in ('run_id', 'clock_domain', 'frame_index', 'stamp_ns', 'capture_monotonic_ns'):
        value = np.asarray(frame[key])
        if value.shape != ():
            raise ValueError('Non-scalar frame identity')
        if key in ('run_id', 'clock_domain'):
            if value.dtype.kind not in 'US':
                raise ValueError('Invalid string identity')
        elif value.dtype.kind not in 'iu':
            raise ValueError('Invalid integer identity')
        identity[key] = value.item()
    return identity


class JsonPipeReader:
    def __init__(self, stream, max_bytes=1048576):
        self.fd = stream.fileno()
        self.pending = b''
        self.max_bytes = max_bytes

    def receive(self, predicate, timeout_s):
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError('Invalid response timeout')
        deadline = time.monotonic() + timeout_s
        with selectors.DefaultSelector() as selector:
            selector.register(self.fd, selectors.EVENT_READ)
            while True:
                if time.monotonic() >= deadline:
                    raise TimeoutError('Perception response deadline exceeded')
                if b'\n' in self.pending:
                    line, self.pending = self.pending.split(b'\n', 1)
                    try:
                        obj = json.loads(line)
                    except (ValueError, UnicodeDecodeError):
                        continue
                    if isinstance(obj, dict) and predicate(obj):
                        return obj
                    continue
                if not selector.select(max(0, deadline-time.monotonic())):
                    raise TimeoutError('Perception response deadline exceeded')
                chunk = os.read(self.fd, 65536)
                if not chunk:
                    raise EOFError('Perception worker closed output')
                self.pending += chunk
                if len(self.pending) > self.max_bytes:
                    raise ValueError('Perception output exceeds limit')


def require_response(request, response, max_age_ns):
    verdict = check_perception_response(request, response, now_ns=time.monotonic_ns(),
                                        max_age_ns=max_age_ns)
    if not verdict['matched']:
        raise ValueError('Rejected perception response: ' + verdict['reason'])
    return verdict


def fresh_response(request, response, max_age_ns):
    """Drop expired cached observations at each consumption point."""
    if not response:
        return {}
    try:
        verdict = require_response(request, response, max_age_ns)
    except ValueError:
        return {}
    return dict(response, provenance_check=verdict)


def stop_worker(worker, timeout_s=2):
    """Reap only this owned child, escalating if it ignores termination."""
    import subprocess
    if worker.poll() is None:
        worker.terminate()
        try:
            worker.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            worker.kill()
            worker.wait(timeout=timeout_s)
    else:
        worker.wait()
    for stream in (worker.stdin, worker.stdout, worker.stderr):
        if stream is not None:
            stream.close()
