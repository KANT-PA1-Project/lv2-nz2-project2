import ast,json,tempfile,sys
from pathlib import Path
from copy import deepcopy
import numpy as np
from sensor_msgs.msg import Image,CameraInfo
from builtin_interfaces.msg import Time
repo=Path(__file__).resolve().parents[1];sys.path.insert(0,str(repo/'src'));sys.path.insert(0,str(repo/'tests'))
from arm_practice.publication_diagnostics import PublicationDiagnostics
from test_capture_bundle_identity import CaptureBundleTest
source=(repo/'scripts/r12_sensor_ros.py').read_text();tree=ast.parse(source)
blocks=[n for n in ast.walk(tree) if isinstance(n,ast.If) and ast.unparse(n.test)=="packet.get('fault') != 'camera'"]
assert len(blocks)==1
code=compile(ast.fix_missing_locations(ast.Module(body=blocks,type_ignores=[])),'actual_bridge_publication','exec')
class Pub:
 def __init__(self):self.messages=[]
 def publish(self,msg):self.messages.append(deepcopy(msg))
def execute(folder,diag,frame=2):
 pubs=[]
 def pub():
  p=Pub();pubs.append(p);return p
 images={s:pub() for s in ('left','right')};infos={s:pub() for s in images};depth=pub()
 extra={f'r07_camera{c}_{s}_optical':(pub(),pub(),pub() if s=='left' else None) for c in (2,3) for s in ('left','right')}
 packet=dict(capture_run_id='run-a',stamp_ns=1000000000+frame*50000000,frame_capture_monotonic_ns=123+frame-2)
 env=dict(np=np,Image=Image,CameraInfo=CameraInfo,out=folder,packet=packet,ts=Time(sec=packet['stamp_ns']//10**9,nanosec=packet['stamp_ns']%10**9),images=images,infos=infos,depth_pub=depth,extra_publishers=extra,publication_diagnostics=diag)
 exec(code,env);env['data'].close();return [p.messages for p in pubs]
with tempfile.TemporaryDirectory() as tmp:
 folder=Path(tmp);fixture=CaptureBundleTest();fixture.setUp();data=fixture.d;data.update(depth=np.ones((480,640),np.float32),r07_camera2_depth=np.ones((480,640),np.float32),r07_camera3_depth=np.ones((480,640),np.float32));np.savez(folder/'frame.npz',**data)
 baseline=execute(folder,None);diag=PublicationDiagnostics(folder);actual=execute(folder,diag);assert baseline==actual
 first=json.loads(diag.path.read_text());assert first['complete_correspondence'] and len(first['images'])==6
 for frame in range(3,23):
  data.update(frame_index=np.int64(frame),stamp_ns=np.int64(1000000000+frame*50000000),capture_monotonic_ns=np.int64(123+frame-2));np.savez(folder/'frame.npz',**data);execute(folder,diag,frame)
 assert diag.count==20 and len(diag.path.read_text().splitlines())==20
 mismatch=PublicationDiagnostics(folder);execute(folder,mismatch,2);assert not json.loads(mismatch.path.read_text())['complete_correspondence']
 failing=PublicationDiagnostics(folder/'missing');execute(folder,failing,22);assert failing.error and failing.count==0
 report=dict(actual_bridge_block_verified=True,publication_messages_identical=True,upper_image_records=6,record_cap=20,attempted_bundles=21,mismatched_bundle_rejected=True,write_failure_exposed=True,runtime_launched=False,execution_authorized=False)
 out=repo.parents[1]/'outputs/r12';(out/'v326_verification.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
