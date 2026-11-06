import unittest,copy,numpy as np
from arm_practice.rgbd_epoch_issuer import RendererEpochIssuer

def fk(q):
 t=np.eye(4);t[0,3]=q['joint'];return {'left':t,'right':t}
def fresh():
 x=RendererEpochIssuer(run_id='run',clock_epoch='epoch',cameras=['left','right'],joint_names=['joint'],physics_hz=960,model_sha256='0'*64,source_sha256='1'*64,fk=fk,depth_camera_map={'depth':'left'})
 for step,q,start in [(96,.1,1000000000),(144,.2,1050000000)]:x.record_encoder_epoch(physics_step=step,sim_time_s=step/960,read_start_ns=start,read_end_ns=start+1000,measured_encoder=[q])
 pixels={k:np.zeros((480,640,3),np.uint8) for k in ['left','right']};meta={k:dict(referenceTimeNumerator=100000000,referenceTimeDenominator=1000000000) for k in pixels};args=dict(native_camera_metadata=meta,pixels=pixels,current_physics_step=144,readback_return_ns=1060000000,now_ns=1060001000)
 args.update(native_depth_metadata={'depth':copy.deepcopy(meta['left'])},depths={'depth':np.ones((480,640),np.float32)},native_metadata_after={'rgb':copy.deepcopy(meta),'depth':{'depth':copy.deepcopy(meta['left'])}})
 return x,pixels,args
class Tests(unittest.TestCase):
 def test_actual_historical_q_fk_selected(self):
  x,p,a=fresh();s=x.issue(**a);v=x.verify(s,pixels=p,depths=a['depths'],now_ns=a['now_ns']);self.assertTrue(v['matched']);self.assertEqual(v['encoder_history']['encoder_q'],[.1]);self.assertEqual(v['encoder_history']['camera_fk']['left'][0][3],.1);self.assertFalse(v['motion_authorized']);self.assertEqual(v['capture_monotonic_lower_bound_ns'],1000000000)
 def test_newq_tagged_old_time_rejected(self):
  x,p,a=fresh();s=x.issue(**a);s['body']['encoder_history']['encoder_q']=[.2];self.assertFalse(x.verify(s,pixels=p,depths=a['depths'],now_ns=a['now_ns'])['matched']);self.assertEqual(x.history[96]['encoder_q'],[.1])
 def test_packet_boolean_not_certificate(self):
  x,p,a=fresh();self.assertFalse(x.verify({'image_capture_association_verified':True},pixels=p,depths=a['depths'],now_ns=a['now_ns'])['matched'])
 def test_foreign_issuer(self):
  x,p,a=fresh();y,_,_=fresh();self.assertFalse(y.verify(x.issue(**a),pixels=p,depths=a['depths'],now_ns=a['now_ns'])['matched'])
 def test_stale_historical_even_fresh_readback(self):
  x,p,a=fresh();a.update(readback_return_ns=1600000000,now_ns=1600001000)
  with self.assertRaisesRegex(ValueError,'historical_capture'):x.issue(**a)
 def test_consumer_age_expiry(self):
  x,p,a=fresh();s=x.issue(**a);self.assertFalse(x.verify(s,pixels=p,depths=a['depths'],now_ns=1500000001)['matched'])
 def test_missing_history(self):
  x,p,a=fresh();del x.history[96]
  with self.assertRaisesRegex(ValueError,'missing'):x.issue(**a)
 def test_mixed_camera_epoch(self):
  x,p,a=fresh();a['native_camera_metadata']['right']['referenceTimeNumerator']=150000000
  a['native_metadata_after']['rgb']=copy.deepcopy(a['native_camera_metadata'])
  with self.assertRaisesRegex(ValueError,'mixed'):x.issue(**a)
 def test_unaligned_renderer_time(self):
  x,p,a=fresh();a['native_camera_metadata']['right']['referenceTimeNumerator']=100100000
  a['native_metadata_after']['rgb']=copy.deepcopy(a['native_camera_metadata'])
  with self.assertRaisesRegex(ValueError,'not_physics'):x.issue(**a)
 def test_sub_ns_rounding_accepted(self):
  x,p,a=fresh()
  for k in a['native_camera_metadata']:a['native_camera_metadata'][k]['referenceTimeNumerator']+=1
  a['native_depth_metadata']['depth']['referenceTimeNumerator']+=1;a['native_metadata_after']={'rgb':copy.deepcopy(a['native_camera_metadata']),'depth':copy.deepcopy(a['native_depth_metadata'])}
  self.assertEqual(x.issue(**a)['body']['capture_physics_step'],96)
 def test_future_renderer_time(self):
  x,p,a=fresh()
  for v in a['native_camera_metadata'].values():v['referenceTimeNumerator']=200000000
  with self.assertRaises(ValueError):x.issue(**a)
 def test_missing_camera(self):
  x,p,a=fresh();del a['native_camera_metadata']['right']
  with self.assertRaisesRegex(ValueError,'camera_set'):x.issue(**a)
 def test_pixels_modified(self):
  x,p,a=fresh();s=x.issue(**a);p['left'][0,0,0]=1;self.assertFalse(x.verify(s,pixels=p,depths=a['depths'],now_ns=a['now_ns'])['matched'])
 def test_repeated_issue(self):
  x,p,a=fresh();x.issue(**a)
  with self.assertRaisesRegex(ValueError,'repeated'):x.issue(**a)
 def test_replayed_consumer(self):
  x,p,a=fresh();s=x.issue(**a);self.assertTrue(x.verify(s,pixels=p,depths=a['depths'],now_ns=a['now_ns'])['matched']);self.assertFalse(x.verify(s,pixels=p,depths=a['depths'],now_ns=a['now_ns'])['matched'])
 def test_run_mismatch(self):
  x,p,a=fresh();s=x.issue(**a);self.assertFalse(x.verify(s,pixels=p,depths=a['depths'],now_ns=a['now_ns'],expected_current_run='old')['matched'])
 def test_encoder_read_over_1ms(self):
  x,p,a=fresh()
  with self.assertRaises(ValueError):x.record_encoder_epoch(physics_step=192,sim_time_s=.2,read_start_ns=1100000000,read_end_ns=1101000001,measured_encoder=[.3])
 def test_encoder_nonfinite(self):
  x,p,a=fresh()
  with self.assertRaises(ValueError):x.record_encoder_epoch(physics_step=192,sim_time_s=.2,read_start_ns=1100000000,read_end_ns=1100001000,measured_encoder=[float('nan')])
 def test_clock_reset_requires_new_issuer(self):
  x,p,a=fresh()
  with self.assertRaises(ValueError):x.record_encoder_epoch(physics_step=48,sim_time_s=.05,read_start_ns=1100000000,read_end_ns=1100001000,measured_encoder=[.3])
 def test_wrong_sim_time(self):
  x,p,a=fresh()
  with self.assertRaises(ValueError):x.record_encoder_epoch(physics_step=192,sim_time_s=.19,read_start_ns=1100000000,read_end_ns=1100001000,measured_encoder=[.3])
 def test_no_future_receipt(self):
  x,p,a=fresh();a['readback_return_ns']=a['now_ns']+1
  with self.assertRaises(ValueError):x.issue(**a)
class DepthTests(unittest.TestCase):
 def test_wrong_depth_epoch(self):
  x,p,a=fresh();a['native_depth_metadata']['depth']['referenceTimeNumerator']=150000000;a['native_metadata_after']['depth']=copy.deepcopy(a['native_depth_metadata'])
  with self.assertRaisesRegex(ValueError,'rgb_depth_epoch_mismatch'):x.issue(**a)
 def test_depth_bit_flip(self):
  x,p,a=fresh();s=x.issue(**a);a['depths']['depth'][0,0]+=.01;self.assertFalse(x.verify(s,pixels=p,depths=a['depths'],now_ns=a['now_ns'])['matched'])
 def test_depth_dtype(self):
  x,p,a=fresh();a['depths']['depth']=a['depths']['depth'].astype(np.float64)
  with self.assertRaisesRegex(ValueError,'invalid_depth'):x.issue(**a)
 def test_missing_depth(self):
  x,p,a=fresh();a['depths']={}
  with self.assertRaisesRegex(ValueError,'depth_set'):x.issue(**a)
 def test_metadata_changed_during_read(self):
  x,p,a=fresh();a['native_metadata_after']['depth']['depth']['referenceTimeNumerator']+=1
  with self.assertRaisesRegex(ValueError,'changed_during_readback'):x.issue(**a)
 def test_depth_nan_bytes_bound_not_pose_validity(self):
  x,p,a=fresh();a['depths']['depth'][0,0]=np.nan;s=x.issue(**a);self.assertTrue(x.verify(s,pixels=p,depths=a['depths'],now_ns=a['now_ns'])['rgbd_capture_association_verified'])
 def test_depth_cam_map_tamper(self):
  x,p,a=fresh();s=x.issue(**a);s['body']['depth_camera_map']['depth']='right';self.assertFalse(x.verify(s,pixels=p,depths=a['depths'],now_ns=a['now_ns'])['matched'])
 def test_rgb_only_cannot_claim_rgbd(self):
  x,p,a=fresh();s={'body':{'schema':'native-render-encoder-association-v1'},'signature':'f'*64};self.assertFalse(x.verify(s,pixels=p,depths=a['depths'],now_ns=a['now_ns'])['matched'])
class HistoryBoundTests(unittest.TestCase):
 def make(self,n):
  return RendererEpochIssuer(run_id='run',clock_epoch='epoch',cameras=['left','right'],joint_names=['joint'],physics_hz=960,model_sha256='0'*64,source_sha256='1'*64,fk=fk,depth_camera_map={'depth':'left'},history_size=n)
 def test_history_invalid_types_and_sizes(self):
  for n in [True,False,0,-1,4097,1.0,None,'128']:
   with self.subTest(n=n),self.assertRaisesRegex(ValueError,'invalid_history_size'):self.make(n)
 def test_history_accepts_both_bounds(self):
  self.assertEqual(self.make(1).history_size,1);self.assertEqual(self.make(4096).history_size,4096)
 def test_history_pruned_epoch_fails_closed(self):
  x=self.make(1)
  for step in [96,144]:x.record_encoder_epoch(physics_step=step,sim_time_s=step/960,read_start_ns=1_000_000_000+step*1000,read_end_ns=1_000_000_001+step*1000,measured_encoder=[.1])
  self.assertEqual(list(x.history),[144]);_,_,a=fresh()
  with self.assertRaisesRegex(ValueError,'historical_encoder_epoch_missing'):x.issue(**a)
if __name__=='__main__':unittest.main()
