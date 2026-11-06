"""Exercise changed actual bridge publication block on recorded and invalid ROS inputs."""
import ast,base64,copy,json
from pathlib import Path
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
repo=Path(__file__).resolve().parents[1];out=repo.parents[1]/'outputs/r12';source=(repo/'scripts/r12_sensor_ros.py').read_text();tree=ast.parse(source)
blocks=[n for n in ast.walk(tree) if isinstance(n,ast.If) and ast.unparse(n.test)=='ns in odom' and any(isinstance(x,ast.Name) and x.id=='visual_pub' for x in ast.walk(n))];assert len(blocks)==1
code=compile(ast.fix_missing_locations(ast.Module(body=copy.deepcopy(blocks[0].body),type_ignores=[])),'<changed actual block>','exec')
class Publisher:
 def publish(self,msg):self.message=copy.deepcopy(msg)
def legacy_output(msg):
 m=copy.deepcopy(msg);m.header.frame_id='odom';m.child_frame_id='base_link'
 for k in (0,7,14):m.pose.covariance[k]=max(m.pose.covariance[k],.0025)
 for k in (21,28,35):m.pose.covariance[k]=max(m.pose.covariance[k],.01)
 return m
count=0
for v in ('v313','v314'):
 for line in (out/f'{v}_raw/raw_messages.jsonl').read_text().splitlines():
  e=json.loads(line)
  if e['type']!='Odometry':continue
  m=deserialize_message(base64.b64decode(e['cdr_base64']),Odometry);original=copy.deepcopy(m);pub=Publisher();ctx=dict(ns=1,odom={1:m},visual_pub=pub,deepcopy=copy.deepcopy)
  exec(code,ctx);assert m==original;assert pub.message==legacy_output(original);assert ctx['vo'] is not m;count+=1
report=dict(samples=count,raw_preserved=count,filter_output_matches_legacy=count,bridge_parse_valid=True,robot_or_ros_nodes_launched=False,scope='Actual changed publication block tested offline;does not validate full controller or source authentication.')
(out/'v315_bridge_verification.json').write_text(json.dumps(report,indent=2)+'\n');print(report)
