#!/usr/bin/env python3
"""Lyrical-side simulated joint command/feedback check on a dedicated ROS domain."""
import argparse
import json
import time
from pathlib import Path
import rclpy
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output', type=Path, required=True)
args=parser.parse_args()
rclpy.init()
node=rclpy.create_node('r12_bridge_check')
samples=[]
clocks=[]
frames=set()
node.create_subscription(TFMessage, "/r12/tf", lambda m: frames.update((t.header.frame_id,t.child_frame_id) for t in m.transforms), 10)
node.create_subscription(JointState, '/r12/joint_states', lambda m: samples.append(m), 10)
node.create_subscription(Clock, '/clock', lambda m: clocks.append(m.clock.sec+m.clock.nanosec*1e-9), 10)
publisher=node.create_publisher(JointState, '/r12/joint_commands', 10)
start=time.monotonic()
while time.monotonic()-start < 40 and not samples:
    rclpy.spin_once(node, timeout_sec=.1)
if not samples:
    raise RuntimeError('No simulated R12 joint states received')
initial=dict(zip(samples[-1].name,samples[-1].position))
command=JointState()
command.name=[f'arm_joint{i}' for i in range(1,6)]
command.position=[.1,0.,0.,0.,0.]
start=time.monotonic()
while time.monotonic()-start < 5:
    command.header.stamp=node.get_clock().now().to_msg()
    publisher.publish(command)
    rclpy.spin_once(node,timeout_sec=.05)
final=dict(zip(samples[-1].name,samples[-1].position))
report={'joint_state_samples':len(samples),'clock_samples':len(clocks),
        'clock_increasing':len(clocks)>1 and clocks[-1]>clocks[0],
        'array_lengths_match':all(len(m.name)==len(m.position)==len(m.velocity) for m in samples),
        'tf_frames':sorted(frames), 'tf_received':bool(frames), 'initial':initial,'final':final,
        'arm_joint1_target_error_rad':abs(final['arm_joint1']-.1),
        'joint1_response_pass':abs(final['arm_joint1']-.1)<.02}
command.position=[0.]*5
publisher.publish(command)
args.output.parent.mkdir(parents=True,exist_ok=True)
args.output.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
node.destroy_node();rclpy.shutdown()
if not all(report[k] for k in ('clock_increasing','array_lengths_match','joint1_response_pass','tf_received')):
    raise SystemExit(1)
