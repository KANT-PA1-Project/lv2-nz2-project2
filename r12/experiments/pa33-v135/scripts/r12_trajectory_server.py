#!/usr/bin/env python3
"""Simulation-only FollowJointTrajectory adapter for the R12 topic bridge.

Uses linear position interpolation. This is not a physical motor driver or a
replacement for a real-time ros2_control hardware implementation.
"""
import json
import os
import math
import threading
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState


class R12TrajectoryServer(Node):
    def __init__(self):
        super().__init__('r12_sim_trajectory_server')
        self.cfg = json.loads((Path(__file__).resolve().parents[1]/os.environ.get('R12_ROBOT_CONFIG','configs/r12_robot.json')).read_text())
        self.names = self.cfg['joint_names']
        self.lock = threading.Lock()
        self.busy = False
        self.state = None
        self.last_state = 0.
        self.group = ReentrantCallbackGroup()
        self.publisher = self.create_publisher(JointState, '/r12/joint_commands', 10)
        self.create_subscription(JointState, '/r12/joint_states', self.on_state, 10, callback_group=self.group)
        self.server = ActionServer(self, FollowJointTrajectory,
            '/r12/arm_controller/follow_joint_trajectory', self.execute,
            goal_callback=self.accept, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.group)

    def on_state(self, msg):
        values = dict(zip(msg.name, msg.position))
        if all(n in values and math.isfinite(values[n]) for n in self.names):
            with self.lock:
                self.state = [values[n] for n in self.names]
                self.last_state = time.monotonic()

    def accept(self, goal):
        traj = goal.trajectory
        with self.lock:
            if self.busy or self.state is None or time.monotonic()-self.last_state > 1:
                return GoalResponse.REJECT
            if list(traj.joint_names) != self.names or not traj.points:
                return GoalResponse.REJECT
            if traj.header.stamp.sec or traj.header.stamp.nanosec:
                return GoalResponse.REJECT  # immediate-start trajectories only
            previous, last_t = list(self.state), 0.
            for point in traj.points:
                q = point.positions
                t = point.time_from_start.sec + point.time_from_start.nanosec*1e-9
                if len(q) != len(self.names) or not all(math.isfinite(x) for x in q) or t < 0 or t < last_t:
                    return GoalResponse.REJECT
                if any(x < lo or x > hi for x,lo,hi in zip(q,self.cfg['lower'],self.cfg['upper'])):
                    return GoalResponse.REJECT
                delta = max(abs(a-b) for a,b in zip(q,previous))
                if t == last_t:
                    if delta > 1e-4:
                        return GoalResponse.REJECT
                elif any(abs(a-b)/(t-last_t) > v for a,b,v in zip(q,previous,self.cfg['velocity'])):
                    return GoalResponse.REJECT
                previous, last_t = list(q), t
            self.busy = True
        return GoalResponse.ACCEPT

    def send(self, q):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.names
        msg.position = q
        self.publisher.publish(msg)

    def execute(self, handle):
        result = FollowJointTrajectory.Result()
        with self.lock:
            initial = list(self.state)
        points = [(0., initial)] + [(p.time_from_start.sec+p.time_from_start.nanosec*1e-9, list(p.positions))
                                   for p in handle.request.trajectory.points]
        duration = points[-1][0]
        start, segment = time.monotonic(), 1
        try:
            while rclpy.ok():
                elapsed = time.monotonic()-start
                with self.lock:
                    actual, age = list(self.state), time.monotonic()-self.last_state
                if handle.is_cancel_requested:
                    self.send(actual)
                    handle.canceled()
                    result.error_string = 'Canceled; holding the last measured simulated position'
                    return result
                if age > 1:
                    handle.abort()
                    result.error_code = result.PATH_TOLERANCE_VIOLATED
                    result.error_string = 'Simulated joint feedback became stale'
                    return result
                while segment < len(points)-1 and elapsed > points[segment][0]:
                    segment += 1
                t0,q0 = points[segment-1]
                t1,q1 = points[segment]
                fraction = min(1.,max(0.,(elapsed-t0)/max(t1-t0,1e-9)))
                desired = [a+fraction*(b-a) for a,b in zip(q0,q1)]
                self.send(desired)
                feedback = FollowJointTrajectory.Feedback()
                feedback.joint_names = self.names
                feedback.desired.positions = desired
                feedback.actual.positions = actual
                feedback.error.positions = [a-b for a,b in zip(desired,actual)]
                handle.publish_feedback(feedback)
                if elapsed >= duration and max(abs(a-b) for a,b in zip(points[-1][1],actual)) <= .02:
                    handle.succeed()
                    result.error_code = result.SUCCESSFUL
                    return result
                if elapsed > duration+3:
                    self.send(actual)
                    handle.abort()
                    result.error_code = result.GOAL_TOLERANCE_VIOLATED
                    return result
                time.sleep(.01)
        finally:
            with self.lock:
                self.busy = False
        handle.abort()
        result.error_code = result.INVALID_GOAL
        return result


def main():
    rclpy.init()
    node = R12TrajectoryServer()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
