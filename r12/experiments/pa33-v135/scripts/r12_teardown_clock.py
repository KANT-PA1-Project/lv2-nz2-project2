"""Bounded clock advancement only after recorded sensor evaluation has ended."""
import time,json,sys
from pathlib import Path
import rclpy
from rosgraph_msgs.msg import Clock
rclpy.init();node=rclpy.create_node('r12_teardown_clock');pub=node.create_publisher(Clock,'/clock',10)
start=time.monotonic();count=0
try:
 while time.monotonic()-start<12:
  ns=2_000_000_000+int((time.monotonic()-start)*1e9);msg=Clock();msg.clock.sec=ns//10**9;msg.clock.nanosec=ns%10**9;pub.publish(msg);count+=1;rclpy.spin_once(node,timeout_sec=.02)
finally:
 Path(sys.argv[1]).write_text(json.dumps(dict(scope='post-evaluation teardown only; not sensor performance evidence',published=count)));node.destroy_node();rclpy.shutdown()
