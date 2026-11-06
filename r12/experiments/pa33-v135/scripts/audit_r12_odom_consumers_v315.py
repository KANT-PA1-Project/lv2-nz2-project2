"""Offline actual bridge attitude block comparison; never starts controller."""
import ast,base64,copy,json,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
repo=Path(__file__).resolve().parents[1];out=repo.parents[1]/'outputs/r12';source=(repo/'scripts/r12_sensor_ros.py').read_text();tree=ast.parse(source)
gates=[n for n in ast.walk(tree) if isinstance(n,ast.If) and ast.unparse(n.test)=='encoder_valid and fused is not None and (status.get(ns) == 1) and (ns in odom)']
if not gates:
 gates=[n for n in ast.walk(tree) if isinstance(n,ast.If) and ast.unparse(n.test).startswith('encoder_valid and fused is not None') and 'ns in odom' in ast.unparse(n.test)]
assert len(gates)==1
code=compile(ast.fix_missing_locations(ast.Module(body=copy.deepcopy(gates[0].body),type_ignores=[])),'<actual attitude block>','exec')
def adapt(m):
 m=copy.deepcopy(m);m.header.frame_id='odom';m.child_frame_id='base_link'
 for i in (0,7,14):m.pose.covariance[i]=max(m.pose.covariance[i],.0025)
 for i in (21,28,35):m.pose.covariance[i]=max(m.pose.covariance[i],.01)
 return m
def check(m):
 env=dict(ns=1,odom={1:m},np=np,Rotation=Rotation,visual_attitude_check={},visual_tilt=None)
 exec(code,env)
 return dict(accepted=env['visual_tilt'] is not None,checks=env['checks'],tilt=None if env['visual_tilt'] is None else env['visual_tilt'].tolist())
rows=[];sample=None
for version in ('v313','v314'):
 for line in (out/f'{version}_raw/raw_messages.jsonl').read_text().splitlines():
  e=json.loads(line)
  if e['type']!='Odometry':continue
  m=deserialize_message(base64.b64decode(e['cdr_base64']),Odometry);sample=m;before=copy.deepcopy(m);raw=check(m);adapted=check(adapt(m));assert m==before
  rows.append(dict(run=version,stamp_ns=e['source_stamp_ns'],raw=raw,adapted=adapted))
assert sample is not None
cases={}
for name in ('wrong_child','negative_tilt_covariance','nan_covariance','excessive_tilt_covariance','invalid_quaternion'):
 m=copy.deepcopy(sample);m.child_frame_id='base_link';m.pose.covariance=[0.]*36;m.pose.pose.orientation.x=0.;m.pose.pose.orientation.y=0.;m.pose.pose.orientation.z=0.;m.pose.pose.orientation.w=1.
 if name=='wrong_child':m.child_frame_id='unverified_camera_frame'
 if name=='negative_tilt_covariance':m.pose.covariance[21]=-.1
 if name=='nan_covariance':m.pose.covariance[21]=float('nan')
 if name=='excessive_tilt_covariance':m.pose.covariance[21]=.1
 if name=='invalid_quaternion':m.pose.pose.orientation.w=0.
 cases[name]=dict(raw=check(m),adapted=check(adapt(m)))
assert all(not c['raw']['accepted'] for c in cases.values())
assert cases['wrong_child']['adapted']['accepted'] and cases['negative_tilt_covariance']['adapted']['accepted']
uses=[dict(line=n.lineno,expression=ast.unparse(n)) for n in ast.walk(tree) if isinstance(n,(ast.Subscript,ast.Call)) and ((isinstance(n,ast.Subscript) and isinstance(n.value,ast.Name) and n.value.id=='odom') or (isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and isinstance(n.func.value,ast.Name) and n.func.value.id=='odom'))]
report=dict(samples=len(rows),raw_accepted=sum(r['raw']['accepted'] for r in rows),adapted_accepted=sum(r['adapted']['accepted'] for r in rows),decision_differences=sum(r['raw']['accepted']!=r['adapted']['accepted'] for r in rows),tilt_differences=sum(r['raw']['tilt']!=r['adapted']['tilt'] for r in rows),synthetic_cases=cases,odom_uses=uses,source_sha256=hashlib.sha256(source.encode()).hexdigest(),runtime_changed=False,scope='Actual bridge block extracted offline. Synthetic invalid inputs are not observed field failures. Raw validity should precede filter adaptation;covariance floors are not sensor uncertainty certificates.')
(out/'v315_odom_consumer_audit.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
