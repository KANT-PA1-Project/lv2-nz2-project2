"""Offline actual bridge-block isolation with real ROS CDR; never launches bridge."""
import ast,base64,copy,json,hashlib
from pathlib import Path
from rclpy.serialization import deserialize_message,serialize_message
from nav_msgs.msg import Odometry
repo=Path(__file__).resolve().parents[1];out=repo.parents[1]/'outputs/r12';path=repo/'scripts/r12_sensor_ros.py';source=path.read_text();tree=ast.parse(source)
blocks=[n for n in ast.walk(tree) if isinstance(n,ast.If) and ast.unparse(n.test)=='ns in odom' and any(isinstance(x,ast.Name) and x.id=='visual_pub' for x in ast.walk(n))]
assert len(blocks)==1
legacy=ast.Module(body=copy.deepcopy(blocks[0].body),type_ignores=[]);candidate=copy.deepcopy(legacy)
assignment=candidate.body[0];assert isinstance(assignment,ast.Assign) and ast.unparse(assignment.value)=='odom[ns]'
assignment.value=ast.Call(func=ast.Name(id='deepcopy',ctx=ast.Load()),args=[assignment.value],keywords=[]);ast.fix_missing_locations(candidate)
class Publisher:
 def publish(self,msg):self.message=copy.deepcopy(msg)
rows=[]
for line in (out/'v313_raw/raw_messages.jsonl').read_text().splitlines():
 event=json.loads(line)
 if event['type']!='Odometry':continue
 raw=base64.b64decode(event['cdr_base64']);results={}
 for name,block in [('legacy',legacy),('copy_candidate',candidate)]:
  msg=deserialize_message(raw,Odometry);original=copy.deepcopy(msg);pub=Publisher();ctx=dict(ns=event['source_stamp_ns'],odom={event['source_stamp_ns']:msg},visual_pub=pub,deepcopy=copy.deepcopy)
  exec(compile(ast.fix_missing_locations(block),'<isolated bridge block>','exec'),ctx)
  results[name]=dict(raw_preserved=msg==original,published_message=pub.message)
 rows.append(dict(stamp_ns=event['source_stamp_ns'],legacy_raw_preserved=results['legacy']['raw_preserved'],candidate_raw_preserved=results['copy_candidate']['raw_preserved'],published_output_identical=results['legacy']['published_message']==results['copy_candidate']['published_message']))
assert rows and all(r['candidate_raw_preserved'] and r['published_output_identical'] for r in rows)
report=dict(samples=len(rows),legacy_mutated=sum(not r['legacy_raw_preserved'] for r in rows),candidate_preserved=sum(r['candidate_raw_preserved'] for r in rows),published_identical=sum(r['published_output_identical'] for r in rows),bridge_source_sha256=hashlib.sha256(source.encode()).hexdigest(),candidate_change='vo=deepcopy(odom[ns]) before existing filter publication adaptation',runtime_changed=False,scope='Offline extracted actual publication block only. Downstream bridge consumers also read odom and may depend on floored covariance;not validated for runtime. Relabelled frames and covariance floors are not raw sensor truth.',rows=rows)
(out/'v314_bridge_copy_audit.json').write_text(json.dumps(report,indent=2)+'\n');print({k:v for k,v in report.items() if k!='rows'})
