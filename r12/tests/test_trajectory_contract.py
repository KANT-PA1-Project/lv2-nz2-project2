"""ROS-enabled unit checks; CPU-only environments skip these tests."""
import importlib.util
from pathlib import Path
import threading
import time
import unittest
try:
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    from rclpy.action import GoalResponse
    spec=importlib.util.spec_from_file_location('r12_server',Path(__file__).resolve().parents[1]/'scripts/r12_trajectory_server.py')
    mod=importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
    AVAILABLE=True
except ImportError:
    AVAILABLE=False


@unittest.skipUnless(AVAILABLE,'Run with ROS 2 Lyrical sourced')
class TrajectoryContract(unittest.TestCase):
    def setUp(self):
        # Exercise the callback contract without creating a live ROS node.
        self.server=object.__new__(mod.R12TrajectoryServer)
        self.server.lock=threading.Lock()
        self.server.names=['a','b']
        self.server.cfg={'lower':[-1,-1],'upper':[1,1],'velocity':[.4,.4]}
        self.server.busy=False;self.server.state=[0.,0.];self.server.last_state=time.monotonic()

    def goal(self, positions=(.1,0.), seconds=1):
        g=FollowJointTrajectory.Goal();g.trajectory.joint_names=['a','b']
        p=JointTrajectoryPoint();p.positions=list(positions);p.time_from_start.sec=seconds
        g.trajectory.points=[p];return g

    def test_valid_and_concurrent_goal(self):
        self.assertEqual(self.server.accept(self.goal()),GoalResponse.ACCEPT)
        self.assertEqual(self.server.accept(self.goal()),GoalResponse.REJECT)

    def test_invalid_joint_vectors(self):
        for values in ([float('nan'),0.],[2.,0.],[.1],[]):
            with self.subTest(values=values):
                self.assertEqual(self.server.accept(self.goal(values)),GoalResponse.REJECT)

    def test_velocity_and_instant_jump(self):
        self.assertEqual(self.server.accept(self.goal((.8,0.))),GoalResponse.REJECT)
        self.assertEqual(self.server.accept(self.goal(seconds=0)),GoalResponse.REJECT)

    def test_stale_state_and_joint_order(self):
        self.server.last_state=time.monotonic()-2
        self.assertEqual(self.server.accept(self.goal()),GoalResponse.REJECT)
        self.server.last_state=time.monotonic()
        g=self.goal();g.trajectory.joint_names.reverse()
        self.assertEqual(self.server.accept(g),GoalResponse.REJECT)

    def test_decreasing_time(self):
        g=self.goal(seconds=2)
        g.trajectory.points.append(self.goal(seconds=1).trajectory.points[0])
        self.assertEqual(self.server.accept(g),GoalResponse.REJECT)
