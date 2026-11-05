#!/usr/bin/env python3
"""Finite simulation trajectory smoke test; rejects success on an absent server."""
import json
import time
from sensor_msgs.msg import JointState
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
rclpy.init()
node=rclpy.create_node('r12_sim_trajectory_check')
client=ActionClient(node,FollowJointTrajectory,'/r12/arm_controller/follow_joint_trajectory')
if not client.wait_for_server(timeout_sec=30):
    raise RuntimeError('Trajectory server unavailable')
received=[]
node.create_subscription(JointState,'/r12/joint_states',lambda m: received.append(m),10)
start=time.monotonic()
while not received and time.monotonic()-start<30:
    rclpy.spin_once(node,timeout_sec=.1)
if not received:
    raise RuntimeError('No simulated joint feedback')
goal=FollowJointTrajectory.Goal()
goal.trajectory.joint_names=[f'arm_joint{i}' for i in range(1,6)]
point=JointTrajectoryPoint()
point.positions=[.12,0.,0.,0.,0.]
point.time_from_start.sec=2
goal.trajectory.points=[point]
future=client.send_goal_async(goal)
rclpy.spin_until_future_complete(node,future,timeout_sec=10)
handle=future.result()
if not handle or not handle.accepted:
    raise RuntimeError('Trajectory goal rejected')
future=handle.get_result_async()
rclpy.spin_until_future_complete(node,future,timeout_sec=10)
response=future.result()
if response is None:
    raise RuntimeError('Trajectory result timed out')
report={'accepted':True,'status':response.status,'error_code':response.result.error_code,
        'error_string':response.result.error_string,'simulated_hardware_only':True}
path=Path(__file__).resolve().parents[1]/'reports/local/r12_trajectory_check.json'
path.write_text(json.dumps(report,indent=2)+'\n')
print(report)
node.destroy_node();rclpy.shutdown()
if response.status != 4 or response.result.error_code != 0:
    raise SystemExit(1)
