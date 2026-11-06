"""Exercise the actual bridge callbacks without importing its ROS main loop."""
import ast
from concurrent.futures import Future
from functools import wraps
import inspect
import os
from pathlib import Path
from types import SimpleNamespace as NS
import time
import unittest
from unittest.mock import Mock


class PathCallbackTests(unittest.TestCase):
    def setUp(self):
        source = Path(__file__).resolve().parents[1]/'scripts/r12_sensor_ros.py'
        tree = ast.parse(source.read_text())
        names = {'timed_ros_callback', 'planned', 'accepted'}
        code = ast.Module(body=[n for n in tree.body if isinstance(n, ast.FunctionDef)
                                and n.name in names], type_ignores=[])
        self.worker = Mock()
        self.mission = dict(index=0, busy=True, path=[[.1,.2],[.3,.4]], plan_errors=[])
        self.scope = dict(time=time, os=os, wraps=wraps, callback_duration_rows=[],
                          mission=self.mission, path_worker=self.worker)
        exec(compile(code, str(source), 'exec'), self.scope)

    def completed_future(self, status=4):
        f = Future()
        poses = [NS(pose=NS(position=NS(x=x, y=y))) for x,y in [[1.,1.],[2.,2.]]]
        f.set_result(NS(status=status, result=NS(path=NS(poses=poses))))
        return f

    def test_async_submission_keeps_old_path_until_result(self):
        self.scope['planned'](self.completed_future(), 0)
        self.worker.submit.assert_called_once_with([[1.,1.],[2.,2.]], 0)
        self.assertEqual(self.mission['path'], [[.1,.2],[.3,.4]])
        self.assertTrue(self.mission['busy'])

    def test_old_goal_result_cannot_clear_current_busy_or_path(self):
        self.mission['index'] = 1
        self.scope['planned'](self.completed_future(), 0)
        self.worker.submit.assert_not_called()
        self.assertTrue(self.mission['busy'])
        self.assertEqual(self.mission['path'], [[.1,.2],[.3,.4]])

    def test_failed_plan_retains_previous_path(self):
        self.scope['planned'](self.completed_future(status=6), 0)
        self.worker.submit.assert_not_called()
        self.assertFalse(self.mission['busy'])
        self.assertEqual(self.mission['path'], [[.1,.2],[.3,.4]])
        self.assertEqual(self.mission['plan_errors'], [6])

    def test_worker_submit_failure_retains_previous_path(self):
        self.worker.submit.side_effect = RuntimeError('worker unavailable')
        self.scope['planned'](self.completed_future(), 0)
        self.assertFalse(self.mission['busy'])
        self.assertEqual(self.mission['path'], [[.1,.2],[.3,.4]])
        self.assertEqual(self.mission['plan_errors'], ['worker unavailable'])

    def test_old_goal_acceptance_does_not_touch_new_goal(self):
        self.mission['index'] = 1
        f = Mock()
        self.scope['accepted'](f, 0)
        f.result.assert_not_called()
        self.assertTrue(self.mission['busy'])

    def test_timing_wrapper_preserves_ros_callback_signature(self):
        def callback(message):
            return message
        wrapped = self.scope['timed_ros_callback']('test')(callback)
        self.assertEqual(inspect.signature(wrapped), inspect.signature(callback))
        self.assertEqual(wrapped('value'), 'value')


if __name__ == '__main__':
    unittest.main()
