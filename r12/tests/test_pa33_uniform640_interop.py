"""Real common producer -> isolated PA33 transport in another local process."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import unittest
import numpy as np
from arm_practice.native_rgbd.mixed_camera_schema import MixedCameraSchema,build_config,VERSION,UNIFORM_VERSION
from arm_practice.native_rgbd.trusted_capture_ipc import send_packet

ROOT=Path(__file__).resolve().parents[1]
CHILD="""
import hashlib,json,socket,sys
from pathlib import Path
from arm_practice.native_rgbd.mixed_camera_schema import MixedCameraSchema,build_config
from arm_practice.native_rgbd.trusted_capture_ipc import receive_packet
root=Path(sys.argv[1]);schema=MixedCameraSchema(build_config(root,sys.argv[3]))
sock=socket.socket(fileno=int(sys.argv[2]));sock.settimeout(5)
packet,blobs=receive_packet(sock,wire_version=schema.wire_schema,camera_schema=schema)
hashes=[hashlib.sha256(b).hexdigest() for b in blobs]
assert hashes==packet["expected_hashes"]
print(json.dumps({"bytes":sum(map(len,blobs)),"hashes":hashes,"wire":schema.wire_schema,
                 "source":__import__("arm_practice.native_rgbd.trusted_capture_ipc",fromlist=["x"]).__file__}))
"""

class PA33Interop(unittest.TestCase):
    def test_both_profiles_to_prepared_pa33_process(self):
        spec=importlib.util.spec_from_file_location("prepare_candidate",ROOT/"scripts/prepare_r12_pa33_uniform640.py")
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory(prefix="pa24-pa33-interop-") as temp:
            candidate=module.prepare(ROOT,Path(temp)/"candidate")
            for profile,total in [(VERSION,36_864_000),(UNIFORM_VERSION,12_288_000)]:
                with self.subTest(profile=profile):
                    schema=MixedCameraSchema(build_config(ROOT,profile))
                    descriptors=[schema.descriptor(k,"rgb") for k in sorted(schema.config["cameras"])]
                    descriptors += [schema.descriptor(k,"depth") for k in sorted(schema.config["depth"])]
                    blobs=[np.full(d["shape"],i+1,dtype=d["dtype"]).tobytes() for i,d in enumerate(descriptors)]
                    hashes=[hashlib.sha256(b).hexdigest() for b in blobs]
                    packet={"kind":"capture","certificate":{"body":schema.fields()},
                            "arrays":descriptors,"expected_hashes":hashes}
                    parent,child=socket.socketpair();parent.settimeout(5)
                    env=dict(os.environ,PYTHONPATH=str(candidate/"src"))
                    process=subprocess.Popen([sys.executable,"-c",CHILD,str(candidate),str(child.fileno()),profile],
                         pass_fds=(child.fileno(),),env=env,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True)
                    child.close()
                    try:
                        send_packet(parent,packet,blobs,wire_version=schema.wire_schema,camera_schema=schema)
                        stdout,stderr=process.communicate(timeout=10)
                    finally:
                        parent.close()
                        if process.poll() is None:process.terminate();process.wait(timeout=5)
                    self.assertEqual(process.returncode,0,stderr)
                    result=json.loads(stdout)
                    self.assertEqual(result["bytes"],total);self.assertEqual(result["hashes"],hashes)
                    self.assertEqual(result["wire"],schema.wire_schema)
                    self.assertTrue(result["source"].startswith(str(candidate/"src")))
if __name__=="__main__":unittest.main()
