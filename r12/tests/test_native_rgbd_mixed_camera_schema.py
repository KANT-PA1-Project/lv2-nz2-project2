import copy,json,hashlib,socket,struct,threading,unittest
from pathlib import Path
import numpy as np
from arm_practice.native_rgbd.mixed_camera_schema import *
from arm_practice.native_rgbd.renderer_epoch_issuer import RendererEpochIssuer,digest_image,digest_depth
from arm_practice.native_rgbd.trusted_capture_ipc import send_packet,receive_packet,make_request,request_matches,wire_limit
ROOT=Path(__file__).resolve().parents[1]

def fixture():
 schema=MixedCameraSchema.from_file(ROOT/'configs/mixed_camera_schema.json');cams=list(schema.config['cameras']);deps=schema.config['depth']
 x=RendererEpochIssuer(run_id='mixed',clock_epoch='e',cameras=cams,joint_names=['q'],physics_hz=960,model_sha256='0'*64,source_sha256='1'*64,fk=lambda q:{k:np.eye(4) for k in cams},depth_camera_map={k:v['rgb_camera'] for k,v in deps.items()},camera_schema=schema)
 for step,start in [(96,1_000_000_000),(144,1_050_000_000)]:x.record_encoder_epoch(physics_step=step,sim_time_s=step/960,read_start_ns=start,read_end_ns=start+100,measured_encoder=[.1])
 rgb={k:np.zeros(v['rgb_shape'],np.uint8) for k,v in schema.config['cameras'].items()};depth={k:np.ones(v['shape'],np.float32) for k,v in deps.items()}
 meta={k:{'referenceTimeNumerator':1,'referenceTimeDenominator':10} for k in rgb};dm={k:dict(meta[v['rgb_camera']]) for k,v in deps.items()}
 a=dict(native_camera_metadata=meta,pixels=rgb,native_depth_metadata=dm,depths=depth,native_metadata_after={'rgb':meta,'depth':dm},current_physics_step=144,readback_return_ns=1_060_000_000,now_ns=1_060_000_001)
 return schema,x,rgb,depth,a

class MixedContract(unittest.TestCase):
 def setUp(self):self.schema=MixedCameraSchema.from_file(ROOT/'configs/mixed_camera_schema.json')
 def test_exact_profile_and_payload(self):
  s,x,r,d,a=fixture();cert=x.issue(**a);self.assertTrue(x.verify(cert,pixels=r,depths=d,now_ns=a['now_ns'])['matched']);self.assertEqual(sum(v.nbytes for v in [*r.values(),*d.values()]),36_864_000);self.assertEqual(cert['body']['camera_schema_sha256'],s.sha256)
 def test_wrong_K_even_rehashed(self):
  c=copy.deepcopy(self.schema.config);c['cameras']['arm_camera_left_optical']['intrinsics'][2]+=.5;c['calibration_sha256']=sha({k:v for k,v in c.items() if k!='calibration_sha256'})
  with self.assertRaisesRegex(ValueError,'shape_K'):MixedCameraSchema(c)
 def test_wrong_shape_even_rehashed(self):
  c=copy.deepcopy(self.schema.config);c['cameras']['arm_camera_left_optical']['rgb_shape']=[480,640,3]
  with self.assertRaisesRegex(ValueError,'shape_K'):MixedCameraSchema(c)
 def test_camera_spec_numeric_bool(self):
  c=copy.deepcopy(self.schema.config);c['cameras']['arm_camera_left_optical']['intrinsics'][0]=True
  with self.assertRaises(ValueError):MixedCameraSchema(c)
 def test_calibration_digest_mismatch(self):
  c=copy.deepcopy(self.schema.config);c['calibration_sha256']='a'*64
  with self.assertRaisesRegex(ValueError,'hash_mismatch'):MixedCameraSchema(c)
 def test_bad_depth_units(self):
  c=copy.deepcopy(self.schema.config);c['depth']['arm_camera_depth']['unit']='mm'
  with self.assertRaisesRegex(ValueError,'depth_schema'):MixedCameraSchema(c)
 def test_legacy_shape_rejection_retained(self):
  with self.assertRaises(ValueError):digest_image(np.zeros((1440,1920,3),np.uint8))
  with self.assertRaises(ValueError):digest_depth(np.zeros((1440,1920),np.float32))
 def test_mixed_array_wrong_shape(self):
  with self.assertRaisesRegex(ValueError,'shape_or_dtype'):self.schema.digest(np.zeros((480,640,3),np.uint8),'arm_camera_left_optical','rgb')
 def test_mixed_array_wrong_dtype(self):
  with self.assertRaisesRegex(ValueError,'shape_or_dtype'):self.schema.digest(np.zeros((1440,1920),np.float64),'arm_camera_depth','depth')
 def test_signed_schema_tamper(self):
  s,x,r,d,a=fixture();cert=x.issue(**a);cert['body']['camera_schema']['cameras']['arm_camera_left_optical']['intrinsics'][0]=400.;self.assertFalse(x.verify(cert,pixels=r,depths=d,now_ns=a['now_ns'])['matched'])
 def test_wrong_request_schema_hash(self):
  s,x,r,d,a=fixture();cert=x.issue(**a);req=make_request(cert,r,d,nonce='n',gid='ab'*16,camera_schema=s);wrong=dict(req,camera_schema_sha256='b'*64)
  with self.assertRaises(ValueError):request_matches(wrong,req)
 def test_explicit_profile_required(self):
  s,x,r,d,a=fixture();cert=x.issue(**a)
  with self.assertRaisesRegex(ValueError,'explicit'):make_request(cert,r,d,nonce='n',gid='ab'*16)
  with self.assertRaises(ValueError):wire_limit(WIRE_V2,None)
 def test_replay_rejected(self):
  s,x,r,d,a=fixture();cert=x.issue(**a);self.assertTrue(x.verify(cert,pixels=r,depths=d,now_ns=a['now_ns'])['matched']);self.assertFalse(x.verify(cert,pixels=r,depths=d,now_ns=a['now_ns'])['matched'])
 def test_out_of_order_rejected(self):
  s,x,r,d,a=fixture();old=x.issue(**a)
  for meta in [a['native_camera_metadata'],a['native_depth_metadata']]:
   for v in meta.values():v.update(referenceTimeNumerator=3,referenceTimeDenominator=20)
  newer=x.issue(**a);self.assertTrue(x.verify(newer,pixels=r,depths=d,now_ns=a['now_ns'])['matched']);self.assertFalse(x.verify(old,pixels=r,depths=d,now_ns=a['now_ns'])['matched'])
 def test_expired500ms_rejected(self):
  s,x,r,d,a=fixture();cert=x.issue(**a);v=x.verify(cert,pixels=r,depths=d,now_ns=1_500_000_001);self.assertFalse(v['matched']);self.assertIn('stale',v['reason'])
 def test_native_rgb_depth_epoch_mismatch(self):
  s,x,r,d,a=fixture();a['native_depth_metadata']['arm_camera_depth']['referenceTimeNumerator']=2
  with self.assertRaisesRegex(ValueError,'rgb_depth_epoch_mismatch'):x.issue(**a)
 def test_actual_camera_lens_readback(self):
  rb={k:{q:copy.deepcopy(v[q]) for q in ['resolution_wh','focal_length_mm','horizontal_aperture_mm','vertical_aperture_mm','clipping_range_m']} for k,v in self.schema.config['cameras'].items()};self.assertTrue(self.schema.check_actual_optics(rb));rb['arm_camera_left_optical']['focal_length_mm']+=.01
  with self.assertRaisesRegex(ValueError,'lens'):self.schema.check_actual_optics(rb)
 def test_mixed_archive_hash(self):
  f=self.schema.archive_fields();self.schema.verify_archive(f);f['camera_schema_sha256']=np.array('a'*64)
  with self.assertRaisesRegex(ValueError,'archive'):self.schema.verify_archive(f)
 def wire_header_reject(self,header,version,reason):
  a,b=socket.socketpair()
  try:
   data=json.dumps(header).encode();a.sendall(struct.pack('!I',len(data))+data)
   with self.assertRaisesRegex(ValueError,reason):receive_packet(b,wire_version=version,camera_schema=self.schema if version==WIRE_V2 else None)
  finally:a.close();b.close()
 def test_wire_legacy_does_not_accept_v2(self):self.wire_header_reject({'transport_version':WIRE_V2,'blob_lengths':[]},WIRE_V1,'profile_mismatch')
 def test_wire_v2_does_not_accept_v1(self):self.wire_header_reject({'blob_lengths':[]},WIRE_V2,'profile_mismatch')
 def test_v1_16MiB_limit_retained(self):self.wire_header_reject({'blob_lengths':[16*1024*1024+1]},WIRE_V1,'blob_lengths')
 def test_v2_over48MiB_rejected_before_read(self):self.wire_header_reject({'transport_version':WIRE_V2,'blob_lengths':[48*1024*1024+1]},WIRE_V2,'blob_lengths')
 def test_v2_bad_descriptor_rejected_before_read(self):
  header={'transport_version':WIRE_V2,'kind':'capture','blob_lengths':[1],'certificate':{'body':self.schema.fields()},'arrays':[{'kind':'rgb','name':'arm_camera_left_optical','dtype':'uint8','shape':[1]}]};self.wire_header_reject(header,WIRE_V2,'descriptors')
 def test_mixed_binary_roundtrip_exact_bytes(self):
  s,x,r,d,args=fixture();cert=x.issue(**args);arrays=[(k,v,'rgb') for k,v in sorted(r.items())]+[(k,v,'depth') for k,v in sorted(d.items())];descriptors=[s.descriptor(k,kind) for k,v,kind in arrays];blobs=[v.tobytes() for k,v,kind in arrays];packet={'kind':'capture','certificate':cert,'arrays':descriptors};a,b=socket.socketpair();a.settimeout(5);b.settimeout(5);errors=[]
  def writer():
   try:send_packet(a,packet,blobs,wire_version=WIRE_V2,camera_schema=s)
   except BaseException as e:errors.append(str(e))
  t=threading.Thread(target=writer);t.start()
  try:received,actual=receive_packet(b,wire_version=WIRE_V2,camera_schema=s);self.assertEqual(received,packet);self.assertEqual(actual,blobs)
  finally:t.join(6);a.close();b.close()
  self.assertFalse(t.is_alive());self.assertEqual(errors,[])
if __name__=='__main__':unittest.main()
