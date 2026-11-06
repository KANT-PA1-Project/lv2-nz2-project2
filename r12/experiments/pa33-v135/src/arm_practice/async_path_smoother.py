"""Bounded path smoothing IPC, separate from the ROS sensor executor.

Prepared component: the mission bridge must explicitly integrate it. It never
publishes motion. The existing smoother and immutable map/footprint are loaded
in a dedicated subprocess, so no ROS or CUDA main module is forked/reimported.
"""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import math
import os
from pathlib import Path
import select
import subprocess
import sys
import time


def _digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _path(value):
    if not isinstance(value, (list, tuple)) or not 2 <= len(value) <= 10000:
        raise ValueError('Path must contain 2..10000 XY points')
    result = []
    for point in value:
        if len(point) != 2 or not all(math.isfinite(float(v)) for v in point):
            raise ValueError('Invalid path coordinate')
        result.append([float(v) for v in point])
    return result


class AsyncPathSmoother:
    """One in-flight request, no callback-side path computation or pipe wait."""

    def __init__(self, map_path, footprint_path, smoother_path, *, timeout_seconds=10.,
                 python_executable=None):
        if not math.isfinite(timeout_seconds) or timeout_seconds <= 0:
            raise ValueError('Invalid worker timeout')
        self.sources = {name: {'path': str(Path(path).resolve()), 'sha256': _digest(path)}
                        for name, path in [('map', map_path), ('footprint', footprint_path),
                                           ('smoother', smoother_path)]}
        self.timeout_seconds = timeout_seconds
        self._generation = 0
        self._future = None
        self._closed = False
        self._buffer = b''
        self._ready = False
        self._process = subprocess.Popen(
            [python_executable or sys.executable, str(Path(__file__).resolve()),
             '--worker', json.dumps(self.sources)], stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            env={**os.environ, 'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'})
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='path-ipc')

    @property
    def busy(self):
        return self._future is not None

    @property
    def worker_pid(self):
        return self._process.pid

    def submit(self, path, goal_index, *, yaw=None):
        if self._closed:
            raise RuntimeError('Path worker is closed')
        if self.busy:
            raise RuntimeError('One path calculation is already pending')
        if type(goal_index) is not int or goal_index < 0:
            raise ValueError('Invalid goal index')
        if yaw is not None and not math.isfinite(float(yaw)):
            raise ValueError('Finite selected chassis yaw required')
        points = _path(path)
        self._generation += 1
        request = {'ticket': self._generation, 'goal_index': goal_index,
                   'path': points, 'yaw': None if yaw is None else float(yaw), 'submitted_monotonic_ns': time.monotonic_ns()}
        self._future = self._executor.submit(self._exchange, request)
        self._request = request
        return request['ticket']

    def invalidate(self):
        """Revoke an old plan even if a later mission reuses its goal index."""
        self._generation += 1

    def poll(self, current_goal_index, *, max_age_seconds=2.):
        if not math.isfinite(max_age_seconds) or max_age_seconds < 0:
            raise ValueError('Invalid path result maximum age')
        if self._future is None or not self._future.done():
            return None
        request = self._request
        try:
            result = self._future.result()
        except Exception as error:
            result = {'error': f'{type(error).__name__}: {error}'}
        self._future = None
        age = (time.monotonic_ns()-request['submitted_monotonic_ns'])/1e9
        reason = ('stale_goal' if request['goal_index'] != current_goal_index else
                  'revoked_ticket' if request['ticket'] != self._generation else
                  'expired' if age > max_age_seconds else
                  'worker_error' if result.get('error') else 'ready')
        if reason != 'ready':
            result.pop('path', None)
        return {**result, 'ticket': request['ticket'], 'goal_index': request['goal_index'],
                'selected_yaw': request['yaw'], 'age_seconds': age, 'ready_for_goal': reason == 'ready', 'reason': reason}

    def _read(self, deadline):
        stream = self._process.stdout
        while b'\n' not in self._buffer:
            remaining = deadline-time.monotonic()
            if remaining <= 0:
                raise TimeoutError('Path worker response deadline exceeded')
            readable, _, _ = select.select([stream], [], [], min(.1, remaining))
            if readable:
                data = os.read(stream.fileno(), 65536)
                if not data:
                    raise RuntimeError('Path worker exited before its response')
                self._buffer += data
                if len(self._buffer) > 2_000_000:
                    raise ValueError('Path worker response too large')
        line, self._buffer = self._buffer.split(b'\n', 1)
        return json.loads(line)

    def _exchange(self, request):
        deadline = time.monotonic()+self.timeout_seconds
        try:
            if not self._ready:
                ready = self._read(deadline)
                if ready != {'ready': True, 'sources': self.sources}:
                    raise RuntimeError(f'Worker source verification failed: {ready}')
                self._ready = True
            self._process.stdin.write((json.dumps(request, allow_nan=False)+'\n').encode())
            self._process.stdin.flush()
            result = self._read(deadline)
            if result.get('ticket') != request['ticket']:
                raise RuntimeError('Worker response ticket mismatch')
            if not result.get('error'):
                result['path'] = _path(result['path'])
            return result
        except Exception:
            # Never allow a delayed response to be consumed by another request.
            if self._process.poll() is None:
                self._process.terminate()
            raise

    def close(self):
        if self._closed:
            return
        self._closed = True
        self.invalidate()
        if self._process.poll() is None:
            self._process.terminate()
        try:
            self._process.wait(timeout=2.)
        except subprocess.TimeoutExpired:
            self._process.kill()
            self._process.wait(timeout=2.)
        self._executor.shutdown(wait=True, cancel_futures=True)
        self._process.stdin.close()
        self._process.stdout.close()


def _worker(sources):
    # This entry point imports no simulator, ROS node or actuator controller.
    import importlib.util
    import numpy as np
    for item in sources.values():
        if _digest(item['path']) != item['sha256']:
            raise ValueError('Worker input changed before initialization')
    spec = importlib.util.spec_from_file_location('r12_original_smoother', sources['smoother']['path'])
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    with np.load(sources['map']['path']) as archive:
        occupied = archive['occupied'].copy()
        clearance = archive['clearance'].copy()
        resolution = float(archive['resolution'])
    footprint = json.loads(Path(sources['footprint']['path']).read_text())['vertices']
    occupied.flags.writeable = False
    clearance.flags.writeable = False
    print(json.dumps({'ready': True, 'sources': sources}), flush=True)
    for line in sys.stdin:
        request = json.loads(line)
        started = time.monotonic_ns()
        try:
            kwargs = {} if request.get('yaw') is None else {'yaw': request['yaw']}
            result = {'path': module.smooth_path(_path(request['path']), occupied, clearance,
                                                  resolution, footprint, **kwargs)}
        except Exception as error:
            result = {'error': f'{type(error).__name__}: {error}'}
        result.update(ticket=request['ticket'], worker_compute_ns=time.monotonic_ns()-started)
        print(json.dumps(result, allow_nan=False), flush=True)


if __name__ == '__main__':
    if len(sys.argv) != 3 or sys.argv[1] != '--worker':
        raise SystemExit('This module is an internal path worker')
    _worker(json.loads(sys.argv[2]))
