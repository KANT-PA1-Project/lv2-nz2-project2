"""Isolated IPC tests; no ROS, simulator or live mission modifications."""
import importlib.util
import json
from pathlib import Path
import tempfile
import time
import unittest

import numpy as np

from arm_practice.async_path_smoother import AsyncPathSmoother


class AsyncPathTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.map = self.root/'map.npz'
        self.footprint = self.root/'footprint.json'
        self.algorithm = self.root/'algorithm.py'
        np.savez(self.map, occupied=np.zeros((60, 60), bool),
                 clearance=np.ones((60, 60)), resolution=.05)
        self.footprint.write_text(json.dumps({'vertices': [[-.05,-.05],[.05,-.05],[.05,.05],[-.05,.05]]}))
        self.algorithm.write_text('''import time
def smooth_path(path, *args):
    if path[0][0] == 42: time.sleep(5)
    if path[0][0] == 43: raise ValueError('controlled failure')
    return path
''')

    def worker(self, **kwargs):
        worker = AsyncPathSmoother(self.map, self.footprint, self.algorithm, **kwargs)
        self.addCleanup(worker.close)
        return worker

    def result(self, worker, goal=0, **kwargs):
        deadline = time.monotonic()+10
        while time.monotonic() < deadline:
            result = worker.poll(goal, **kwargs)
            if result is not None:
                return result
            time.sleep(.005)
        self.fail('Worker result did not arrive')

    def test_valid_result_and_reuse(self):
        w = self.worker()
        for goal in [0, 1]:
            path = [[1., 1.], [2., 2.]]
            w.submit(path, goal)
            result = self.result(w, goal, max_age_seconds=10)
            self.assertTrue(result['ready_for_goal'])
            self.assertEqual(result['path'], path)
            self.assertFalse(w.busy)

    def test_busy_nonblocking_and_close_reaps_worker(self):
        w = self.worker()
        w.submit([[42., 1.], [2., 2.]], 0)
        self.assertIsNone(w.poll(0))
        with self.assertRaises(RuntimeError):
            w.submit([[1., 1.], [2., 2.]], 0)
        pid = w.worker_pid
        w.close()
        self.assertFalse(Path(f'/proc/{pid}').exists())

    def test_goal_change_discards_path(self):
        w = self.worker()
        w.submit([[1., 1.], [2., 2.]], 0)
        result = self.result(w, 1)
        self.assertEqual(result['reason'], 'stale_goal')
        self.assertNotIn('path', result)

    def test_reused_goal_after_invalidation_discards_path(self):
        w = self.worker()
        w.submit([[1., 1.], [2., 2.]], 0)
        w.invalidate()
        result = self.result(w, 0)
        self.assertEqual(result['reason'], 'revoked_ticket')
        self.assertNotIn('path', result)

    def test_expired_result_is_not_exposed(self):
        w = self.worker()
        w.submit([[1., 1.], [2., 2.]], 0)
        result = self.result(w, max_age_seconds=0)
        self.assertEqual(result['reason'], 'expired')
        self.assertNotIn('path', result)

    def test_algorithm_error_does_not_expose_path(self):
        w = self.worker()
        w.submit([[43., 1.], [2., 2.]], 0)
        result = self.result(w, max_age_seconds=10)
        self.assertEqual(result['reason'], 'worker_error')
        self.assertIn('controlled failure', result['error'])
        self.assertNotIn('path', result)

    def test_timeout_kills_worker_without_late_response_reuse(self):
        w = self.worker(timeout_seconds=.3)
        w.submit([[42., 1.], [2., 2.]], 0)
        result = self.result(w, max_age_seconds=10)
        self.assertEqual(result['reason'], 'worker_error')
        self.assertIn('TimeoutError', result['error'])
        self.assertNotIn('path', result)
        w.close()
        self.assertFalse(Path(f'/proc/{w.worker_pid}').exists())

    def test_nonfinite_or_malformed_path_rejected_before_submission(self):
        w = self.worker()
        for path in [[], [[1., 2.]], [[1., 2.], [float('nan'), 2.]], [[1., 2.], [3.]]]:
            with self.assertRaises(ValueError):
                w.submit(path, 0)
        self.assertFalse(w.busy)

    @unittest.skipUnless(importlib.util.find_spec('scipy'), 'Original smoother requires scipy')
    def test_original_smoother_output_is_preserved(self):
        source = Path(__file__).resolve().parents[1]/'scripts/sensor_runtime/path_smoothing.py'
        self.algorithm = source
        spec = importlib.util.spec_from_file_location('original', source)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        # Occupancy forces an unsafe shortcut to retain the original path.
        occupied = np.zeros((60, 60), bool)
        occupied[20:25, 20:25] = True
        np.savez(self.map, occupied=occupied, clearance=np.ones((60,60)), resolution=.05)
        path = [[.5,.5],[.7,.7],[1.,1.],[1.2,1.2],[1.5,1.5]]
        footprint = json.loads(self.footprint.read_text())['vertices']
        expected = module.smooth_path(path, occupied, np.ones((60,60)), .05, footprint)
        w = self.worker()
        w.submit(path, 0)
        actual = self.result(w, max_age_seconds=10)
        self.assertTrue(actual['ready_for_goal'])
        np.testing.assert_array_equal(actual['path'], expected)
        for selected_yaw in (0., np.deg2rad(70)):
            expected_yaw = module.smooth_path(path, occupied, np.ones((60,60)), .05, footprint, yaw=selected_yaw)
            w.submit(path, 0, yaw=selected_yaw)
            actual_yaw = self.result(w, max_age_seconds=10)
            self.assertTrue(actual_yaw['ready_for_goal'])
            self.assertEqual(actual_yaw['selected_yaw'], selected_yaw)
            np.testing.assert_array_equal(actual_yaw['path'], expected_yaw)
        with self.assertRaises(ValueError):
            w.submit(path, 0, yaw=float('nan'))


if __name__ == '__main__':
    unittest.main()
