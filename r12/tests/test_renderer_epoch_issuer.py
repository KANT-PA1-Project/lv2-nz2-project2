import unittest,copy,numpy as np
from arm_practice.renderer_epoch_issuer import RendererEpochIssuer

def fk(q):
 t=np.eye(4);t[0,3]=q['joint'];return {'left':t,'right':t}
def fresh():
 x=RendererEpochIssuer(run_id='run',clock_epoch='epoch',cameras=['left','right'],joint_names=['joint'],physics_hz=960,model_sha256='0'*64,source_sha256='1'*64,fk=fk)
 for step,q,start in [(96,.1,1000000000),(144,.2,1050000000)]:x.record_encoder_epoch(physics_step=step,sim_time_s=step/960,read_start_ns=start,read_end_ns=start+1000,measured_encoder=[q])
 pixels={k:np.zeros((480,640,3),np.uint8) for k in ['left','right']};meta={k:dict(referenceTimeNumerator=100000000,referenceTimeDenominator=1000000000) for k in pixels};args=dict(native_camera_metadata=meta,pixels=pixels,current_physics_step=144,readback_return_ns=1060000000,now_ns=1060001000)
 return x,pixels,args
class Tests(unittest.TestCase):
 def test_actual_historical_q_fk_selected(self):
  x,p,a=fresh();s=x.issue(**a);v=x.verify(s,pixels=p,now_ns=a['now_ns']);self.assertTrue(v['matched']);self.assertEqual(v['encoder_history']['encoder_q'],[.1]);self.assertEqual(v['encoder_history']['camera_fk']['left'][0][3],.1);self.assertFalse(v['motion_authorized']);self.assertEqual(v['capture_monotonic_lower_bound_ns'],1000000000)
 def test_newq_tagged_old_time_rejected(self):
  x,p,a=fresh();s=x.issue(**a);s['body']['encoder_history']['encoder_q']=[.2];self.assertFalse(x.verify(s,pixels=p,now_ns=a['now_ns'])['matched']);self.assertEqual(x.history[96]['encoder_q'],[.1])
 def test_packet_boolean_not_certificate(self):
  x,p,a=fresh();self.assertFalse(x.verify({'image_capture_association_verified':True},pixels=p,now_ns=a['now_ns'])['matched'])
 def test_foreign_issuer(self):
  x,p,a=fresh();y,_,_=fresh();self.assertFalse(y.verify(x.issue(**a),pixels=p,now_ns=a['now_ns'])['matched'])
 def test_stale_historical_even_fresh_readback(self):
  x,p,a=fresh();a.update(readback_return_ns=1600000000,now_ns=1600001000)
  with self.assertRaisesRegex(ValueError,'historical_capture'):x.issue(**a)
 def test_consumer_age_expiry(self):
  x,p,a=fresh();s=x.issue(**a);self.assertFalse(x.verify(s,pixels=p,now_ns=1500000001)['matched'])
 def test_missing_history(self):
  x,p,a=fresh();del x.history[96]
  with self.assertRaisesRegex(ValueError,'missing'):x.issue(**a)
 def test_mixed_camera_epoch(self):
  x,p,a=fresh();a['native_camera_metadata']['right']['referenceTimeNumerator']=150000000
  with self.assertRaisesRegex(ValueError,'mixed'):x.issue(**a)
 def test_unaligned_renderer_time(self):
  x,p,a=fresh();a['native_camera_metadata']['right']['referenceTimeNumerator']=100100000
  with self.assertRaisesRegex(ValueError,'not_physics'):x.issue(**a)
 def test_sub_ns_rounding_accepted(self):
  x,p,a=fresh()
  for k in a['native_camera_metadata']:a['native_camera_metadata'][k]['referenceTimeNumerator']+=1
  self.assertEqual(x.issue(**a)['body']['capture_physics_step'],96)
 def test_future_renderer_time(self):
  x,p,a=fresh()
  for v in a['native_camera_metadata'].values():v['referenceTimeNumerator']=200000000
  with self.assertRaises(ValueError):x.issue(**a)
 def test_missing_camera(self):
  x,p,a=fresh();del a['native_camera_metadata']['right']
  with self.assertRaisesRegex(ValueError,'camera_set'):x.issue(**a)
 def test_pixels_modified(self):
  x,p,a=fresh();s=x.issue(**a);p['left'][0,0,0]=1;self.assertFalse(x.verify(s,pixels=p,now_ns=a['now_ns'])['matched'])
 def test_repeated_issue(self):
  x,p,a=fresh();x.issue(**a)
  with self.assertRaisesRegex(ValueError,'repeated'):x.issue(**a)
 def test_replayed_consumer(self):
  x,p,a=fresh();s=x.issue(**a);self.assertTrue(x.verify(s,pixels=p,now_ns=a['now_ns'])['matched']);self.assertFalse(x.verify(s,pixels=p,now_ns=a['now_ns'])['matched'])
 def test_run_mismatch(self):
  x,p,a=fresh();s=x.issue(**a);self.assertFalse(x.verify(s,pixels=p,now_ns=a['now_ns'],expected_current_run='old')['matched'])
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
if __name__=='__main__':unittest.main()

class ScopeTests(unittest.TestCase):
 def test_rgb_proof_does_not_certify_depth(self):
  issuer,pixels,args=fresh();proof=issuer.issue(**args);result=issuer.verify(proof,pixels=pixels,now_ns=args['now_ns'])
  self.assertTrue(result['image_capture_association_verified']);self.assertFalse(result['depth_capture_association_verified']);self.assertFalse(result['rgbd_capture_association_verified']);self.assertFalse(result['motion_authorized'])
