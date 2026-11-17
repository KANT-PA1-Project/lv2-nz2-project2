import rclpy,json,time,signal,sys
from pathlib import Path
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
rclpy.init();node=rclpy.create_node('v336_cloud_observer',enable_rosout=False);rows=[];running=True
counts={}
def stop(*args):
 global running
 running=False
signal.signal(signal.SIGTERM,stop)
def cb(topic):
 def receive(m):
  counts[topic]=counts.get(topic,0)+1
  if len(rows)<200:rows.append(dict(topic=topic,stamp_ns=m.header.stamp.sec*10**9+m.header.stamp.nanosec,frame=m.header.frame_id,points=m.width*m.height if isinstance(m,PointCloud2) else None,fields=[f.name for f in m.fields] if isinstance(m,PointCloud2) else None))
 return receive
subs=[node.create_subscription(cls,t,cb(t),10) for t,cls in [('/r12/points',PointCloud2),('/r12/points_rgb',PointCloud2),('/odometry/filtered',Odometry)]]
start=time.monotonic()
try:
 while running and time.monotonic()-start<120:rclpy.spin_once(node,timeout_sec=.1)
finally:
 Path(sys.argv[1]).write_text(json.dumps(dict(counts=counts,rows=rows),indent=2)+'\n');node.destroy_node();rclpy.shutdown()
