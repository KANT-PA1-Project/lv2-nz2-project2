#!/usr/bin/env python3
"""Simulation output adapter for Nav2 body velocity commands. No hardware transport."""
import sys,time
from pathlib import Path
import numpy as np,torch,rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
root=Path(__file__).resolve().parents[1];sys.path.insert(0,str(root/'src'))
from arm_practice.swerve import SwervePolicy,actuate
PREFIX=['lh1','rh1','lh2','rh2'];STEER=[p+'_steer_joint' for p in PREFIX];WHEEL=[p+'_wheel_joint' for p in PREFIX]
class LearnedSwerve(Node):
 def __init__(self):
  super().__init__('r12_learned_swerve')
  self.net=SwervePolicy();self.net.load_state_dict(torch.load(root/'models/swerve/policy.pt',weights_only=True));self.net.eval();torch.set_num_threads(1)
  self.command=np.zeros(3);self.angles=np.zeros(4);self.previous_target=None;self.command_time=-1e9;self.state_time=-1e9
  self.pub=self.create_publisher(JointState,'/r12/swerve_joint_commands',10)
  self.create_subscription(TwistStamped,'/r12/cmd_vel',self.on_command,10)
  self.create_subscription(JointState,'/r12/joint_states',self.on_state,10)
  self.create_timer(.05,self.tick)
 def on_command(self,msg):
  v=np.array([msg.twist.linear.x,msg.twist.linear.y,msg.twist.angular.z])
  now=self.get_clock().now().nanoseconds*1e-9;stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
  if msg.header.frame_id not in ('base_link','') or not np.isfinite(v).all() or abs(now-stamp)>.25:
   self.command_time=-1e9;return
  self.command=v;self.command_time=time.monotonic()
 def on_state(self,msg):
  values=dict(zip(msg.name,msg.position))
  if all(n in values for n in STEER):
   a=np.array([values[n] for n in STEER])
   if np.isfinite(a).all():self.angles=a;self.state_time=time.monotonic()
 def tick(self):
  now=time.monotonic();fresh=now-self.command_time<.25 and now-self.state_time<.25
  a,w=actuate(self.net,self.command if fresh else np.zeros(3),self.angles,.05,self.previous_target)
  self.previous_target=a.copy()
  # Partial joint messages: position for steer; velocity for wheels. Bridge must support name lookup.
  if now-self.state_time<.25:
   m=JointState();m.header.stamp=self.get_clock().now().to_msg();m.name=STEER;m.position=a.tolist();self.pub.publish(m)
  m=JointState();m.header.stamp=self.get_clock().now().to_msg();m.name=WHEEL;m.velocity=w.tolist();self.pub.publish(m)
def main():
 rclpy.init();node=LearnedSwerve()
 try:rclpy.spin(node)
 finally:node.command_time=-1e9;node.tick();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
