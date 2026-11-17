#!/usr/bin/env python3
"""Request a non-executing joint-space plan from the R12 cuMotion server."""
import json
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from isaac_ros_cumotion_interfaces.action import MotionPlan
rclpy.init()
node=rclpy.create_node('r12_cumotion_validation')
client=ActionClient(node,MotionPlan,'/cumotion/motion_plan')
if not client.wait_for_server(timeout_sec=15):
    raise RuntimeError('cuMotion server unavailable')
goal=MotionPlan.Goal()
goal.start_state.name=[f'arm_joint{i}' for i in range(1,6)]
goal.start_state.position=[0.]*5
goal.goal_state.name=goal.start_state.name
goal.goal_state.position=[.1,0.,0.,0.,0.]
goal.plan_cspace=True
goal.time_dilation_factor=.5
goal.use_current_state=False
goal.use_planning_scene=False
goal.execute_trajectory=False
future=client.send_goal_async(goal)
rclpy.spin_until_future_complete(node,future,timeout_sec=10)
handle=future.result()
if not handle or not handle.accepted:
    raise RuntimeError('cuMotion rejected goal')
future=handle.get_result_async()
rclpy.spin_until_future_complete(node,future,timeout_sec=30)
response=future.result()
if response is None:
    raise RuntimeError('cuMotion plan timed out')
r=response.result
report={'accepted':True,'status':response.status,'success':r.success,
        'error_code':r.error_code.val,'message':r.message,'planning_time':r.planning_time,
        'trajectory_point_counts':[len(t.joint_trajectory.points) for t in r.planned_trajectory]}
path=Path(__file__).resolve().parents[1]/'reports/local/r12_cumotion_plan.json'
path.write_text(json.dumps(report,indent=2)+'\n')
print(report)
node.destroy_node();rclpy.shutdown()
if not r.success:
    raise SystemExit(1)
