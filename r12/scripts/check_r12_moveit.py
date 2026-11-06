#!/usr/bin/env python3
"""Plan and execute a small joint-space goal against the mock MoveIt setup."""
import json
import os
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
target=[float(os.environ.get('R12_TARGET','0.1')),0.,0.,0.,0.]
rclpy.init()
node=rclpy.create_node('r12_moveit_validation')
states=[]
node.create_subscription(JointState,os.environ.get('R12_STATE_TOPIC','/joint_states'),lambda msg: states.append(msg),10)
client=ActionClient(node,MoveGroup,'/move_action')
assert client.wait_for_server(timeout_sec=20), 'MoveIt unavailable'
goal=MoveGroup.Goal()
goal.request.group_name='arm'
goal.request.allowed_planning_time=10.
goal.request.max_velocity_scaling_factor=.25
goal.request.max_acceleration_scaling_factor=.25
goal.request.start_state.is_diff=True
constraint=Constraints()
for i,q in enumerate(target,1):
    constraint.joint_constraints.append(JointConstraint(joint_name=f'arm_joint{i}',position=q,tolerance_above=.001,tolerance_below=.001,weight=1.))
goal.request.goal_constraints=[constraint]
goal.planning_options.plan_only=False
goal.planning_options.planning_scene_diff.is_diff=True
f=client.send_goal_async(goal)
rclpy.spin_until_future_complete(node,f,timeout_sec=15)
h=f.result();assert h and h.accepted
f=h.get_result_async();rclpy.spin_until_future_complete(node,f,timeout_sec=60)
r=f.result();assert r is not None
report={'accepted':True,'status':r.status,'error_code':r.result.error_code.val,'planned_points':len(r.result.planned_trajectory.joint_trajectory.points),'executed_points':len(r.result.executed_trajectory.joint_trajectory.points)}
for _ in range(10):
    rclpy.spin_once(node,timeout_sec=.1)
if states:
    values=dict(zip(states[-1].name,states[-1].position))
    report['final_positions']=[values[f'arm_joint{i}'] for i in range(1,6)]
    report['max_final_error']=max(abs(a-b) for a,b in zip(report['final_positions'],target))
Path(os.environ.get('R12_CHECK_OUTPUT','reports/local/r12_moveit_check.json')).write_text(json.dumps(report,indent=2)+'\n')
print(report)
node.destroy_node();rclpy.shutdown()
assert report['error_code']==1, report
assert report.get('max_final_error',float('inf')) < .01, report
