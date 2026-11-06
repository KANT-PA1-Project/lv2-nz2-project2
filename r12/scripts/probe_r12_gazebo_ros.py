"""Measure live ROS sensor delivery and simulation/wall time independently."""
import argparse,time,json
from pathlib import Path
import yaml,rclpy
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message
ap=argparse.ArgumentParser();ap.add_argument('out',type=Path);ap.add_argument('--seconds',type=float,default=20);args=ap.parse_args()
rclpy.init();n=rclpy.create_node('r12_gazebo_sensor_probe');out=args.out;rows={};subs=[];begin=time.monotonic()
def cb(topic,msg):
 t=time.monotonic();row=rows.setdefault(topic,dict(count=0,first_wall=t));row.update(count=row['count']+1,last_wall=t)
 if hasattr(msg,'header'):row['stamp']=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9;row['frame_id']=msg.header.frame_id
 if hasattr(msg,'clock'):row['stamp']=msg.clock.sec+msg.clock.nanosec*1e-9
 if hasattr(msg,'width'):row.update(width=msg.width,height=msg.height)
 if hasattr(msg,'encoding'):row['encoding']=msg.encoding
 if hasattr(msg,'fields'):row['fields']=[f.name for f in msg.fields]
 if hasattr(msg,'name'):row.update(names=list(msg.name),position=list(msg.position),velocity=list(msg.velocity))
 if hasattr(msg,'angular_velocity'):row['gyro']=[msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z];row['accel']=[msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z]
 if 'first_stamp' not in row and 'stamp' in row:row['first_stamp']=row['stamp']
for item in yaml.safe_load((out/'bridge.yaml').read_text()):
 topic=item['ros_topic_name'];subs.append(n.create_subscription(get_message(item['ros_type_name']),topic,lambda m,t=topic:cb(t,m),qos_profile_sensor_data))
while time.monotonic()-begin<args.seconds:rclpy.spin_once(n,timeout_sec=.05)
clock=rows.get('/clock',{});rtf=(clock.get('stamp',0)-clock.get('first_stamp',0))/max(.001,clock.get('last_wall',begin)-clock.get('first_wall',begin))
report={'duration_wall_s':time.monotonic()-begin,'measured_rtf':rtf,'received_topics':len(rows),'expected_topics':len(subs),'topics':rows,'scope':'Sensor receipt only; no navigation or inference success'}
(out/'sensor_receipt.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k!='topics'}));print('joint',rows.get('/r12/joint_states'));n.destroy_node();rclpy.shutdown()
