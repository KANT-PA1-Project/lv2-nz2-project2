"""Both wire profiles, incompatible bindings, and CAD optical direction."""
import copy
import json
import socket
import struct
import threading
import unittest
from pathlib import Path
import numpy as np
from arm_practice.native_rgbd.mixed_camera_schema import (
    MixedCameraSchema, build_config, VERSION, UNIFORM_VERSION, WIRE_V2, WIRE_V3, sha)
from arm_practice.native_rgbd.trusted_capture_ipc import send_packet, receive_packet, wire_limit

ROOT=Path(__file__).resolve().parents[1]

class UniformContract(unittest.TestCase):
    def schema(self, profile=UNIFORM_VERSION):
        return MixedCameraSchema(build_config(ROOT, profile))

    def packet(self, schema):
        descriptors=[schema.descriptor(k,"rgb") for k in sorted(schema.config["cameras"])]
        descriptors += [schema.descriptor(k,"depth") for k in sorted(schema.config["depth"])]
        # Nonconstant bytes make channel/order corruption visible.
        blobs=[np.full(d["shape"], i+1, dtype=d["dtype"]).tobytes() for i,d in enumerate(descriptors)]
        return {"kind":"capture","certificate":{"body":schema.fields()},"arrays":descriptors},blobs

    def test_both_profiles_roundtrip(self):
        for profile,total in [(VERSION,36_864_000),(UNIFORM_VERSION,12_288_000)]:
            with self.subTest(profile=profile):
                schema=self.schema(profile);packet,blobs=self.packet(schema)
                self.assertEqual(sum(map(len,blobs)),total)
                a,b=socket.socketpair();a.settimeout(5);b.settimeout(5);errors=[]
                def writer():
                    try:send_packet(a,packet,blobs,wire_version=schema.wire_schema,camera_schema=schema)
                    except BaseException as e:errors.append(e)
                t=threading.Thread(target=writer);t.start()
                try:
                    actual,raw=receive_packet(b,wire_version=schema.wire_schema,camera_schema=schema)
                    self.assertEqual(actual,packet);self.assertEqual(raw,blobs)
                finally:
                    b.close();t.join(6);a.close()
                self.assertFalse(t.is_alive());self.assertEqual(errors,[])

    def test_wrong_wire_schema_pair_rejected(self):
        for profile,wrong in [(VERSION,WIRE_V3),(UNIFORM_VERSION,WIRE_V2)]:
            with self.assertRaisesRegex(ValueError,"unsupported_wire_profile"):
                wire_limit(wrong,self.schema(profile))

    def test_uniform_budget_and_explicit_schema(self):
        self.assertEqual(wire_limit(WIRE_V3,self.schema()),16*1024*1024)
        with self.assertRaises(ValueError):wire_limit(WIRE_V3,None)

    def test_mixed_header_rejected_before_blob_read(self):
        a,b=socket.socketpair();b.settimeout(.5)
        try:
            header=json.dumps(dict(transport_version=WIRE_V2,kind="capture",blob_lengths=[36_864_000])).encode()
            a.sendall(struct.pack("!I",len(header))+header)
            with self.assertRaisesRegex(ValueError,"profile_mismatch"):
                receive_packet(b,wire_version=WIRE_V3,camera_schema=self.schema())
        finally:a.close();b.close()

    def test_profile_certificate_cannot_cross(self):
        old=self.schema(VERSION);new=self.schema()
        packet,blobs=self.packet(old)
        with self.assertRaisesRegex(ValueError,"certificate_schema"):
            new.validate_wire(packet,list(map(len,blobs)))

    def test_uniform_lengths_and_K_pinned(self):
        schema=self.schema();packet,blobs=self.packet(schema)
        lengths=list(map(len,blobs));lengths[0]-=1
        with self.assertRaisesRegex(ValueError,"lengths"):schema.validate_wire(packet,lengths)
        cfg=copy.deepcopy(schema.config);cfg["cameras"]["arm_camera_left_optical"]["intrinsics"][0]=1200.
        cfg["calibration_sha256"]=sha({k:v for k,v in cfg.items() if k!="calibration_sha256"})
        with self.assertRaisesRegex(ValueError,"shape_K"):MixedCameraSchema(cfg)

    def test_uniform_terminal_control_requires_opt_in(self):
        schema=self.schema()
        for opt_in in (False,True):
            a,b=socket.socketpair();b.settimeout(.5)
            try:
                send_packet(a,{"kind":"finish","motion_authorized":False})
                if opt_in:
                    data,blobs=receive_packet(b,wire_version=WIRE_V3,camera_schema=schema,allow_control_finish=True)
                    self.assertEqual(data["kind"],"finish");self.assertEqual(blobs,[])
                else:
                    with self.assertRaisesRegex(ValueError,"profile_mismatch"):
                        receive_packet(b,wire_version=WIRE_V3,camera_schema=schema)
            finally:a.close();b.close()

    def test_file_selects_uniform640(self):
        schema=MixedCameraSchema.from_file(ROOT/"configs/mixed_camera_schema.json")
        self.assertEqual(schema.wire_schema,WIRE_V3)
        for spec in schema.config["cameras"].values():
            self.assertEqual(spec["resolution_wh"],[640,480])
            self.assertEqual(spec["intrinsics"],[400.,400.,319.5,239.5])

    def test_CAD_down30_optical_axis_and_FK(self):
        import sys
        sys.path.insert(0,str(ROOT/"scripts"))
        from scipy.spatial.transform import Rotation
        from sensor_runtime.camera_rig import PreparedCameraRig,transforms
        from arm_practice.kinematics import forward_transform
        cfg=json.loads((ROOT/"configs/sensors/camera_rig.json").read_text())["arm_camera"]
        R=Rotation.from_euler("xyz",cfg["optical_rpy_rad"]).as_matrix()
        np.testing.assert_allclose(R[:,2],[np.sqrt(3)/2,0,-.5],atol=1e-12)
        rig=PreparedCameraRig(ROOT)
        for values in ([0]*5,[.1,-.25,.1,-.05,.1],[-.1,.2,-.2,.15,-.1]):
            q={"arm_joint"+str(i+1):v for i,v in enumerate(values)}
            fresh=transforms(ROOT,q);cached=rig.transforms(q)
            for key in fresh:np.testing.assert_allclose(fresh[key],cached[key],atol=1e-12)
            local=np.linalg.inv(forward_transform(ROOT/"models/r12/r12_arm.urdf","arm_link5",q))@cached["arm_camera_left_optical"]
            np.testing.assert_allclose(local[:3,2],R[:,2],atol=1e-12)

if __name__=="__main__":unittest.main()
