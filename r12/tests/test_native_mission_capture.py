import copy,unittest
from pathlib import Path
import numpy as np
from arm_practice.native_rgbd.mission_capture import NativeMissionCapture
from arm_practice.native_rgbd.renderer_epoch_issuer import RendererEpochIssuer
from arm_practice.native_rgbd.mixed_camera_schema import MixedCameraSchema
ROOT=Path(__file__).resolve().parents[1]
class Ann:
 def __init__(self,value):self.value=value
 def get_data(self,**kwargs):return self.value()if callable(self.value)else self.value
class CaptureTests(unittest.TestCase):
 def setUp(self):
  self.schema=MixedCameraSchema.from_file(ROOT/'configs/mixed_camera_schema.json');self.now=1_000_000_000;self.step=96;self.q=np.zeros(7);self.renders=0;self.meta_step=96;self.render_delta=1_000_000;self.change_q=False;self.change_step=False
  self.rgb={k:Ann(np.zeros(v['rgb_shape'][:2]+[4],np.uint8))for k,v in self.schema.config['cameras'].items()}
  self.dep={k:Ann(np.ones(v['shape'],np.float32))for k,v in self.schema.config['depth'].items()}
  self.rr={k:Ann(lambda:self.meta())for k in self.rgb};self.dr={k:Ann(lambda:self.meta())for k in self.dep};self.joints=[f'arm_joint{i}'for i in range(1,6)]
  self.issuer=RendererEpochIssuer(run_id='fixture',clock_epoch='fixture',cameras=list(self.rgb),joint_names=self.joints,physics_hz=960,source_sha256='0'*64,model_sha256='1'*64,fk=lambda q:{k:np.eye(4)for k in self.rgb},depth_camera_map={k:v['rgb_camera']for k,v in self.schema.config['depth'].items()},camera_schema=self.schema)
  self.make()
 def meta(self):return {'referenceTimeNumerator':self.meta_step,'referenceTimeDenominator':960}
 def clock(self):self.now+=10;return self.now
 def render(self):
  self.renders+=1;self.now+=self.render_delta
  if self.change_q:self.q[0]+=.01
  if self.change_step:self.step+=1
 def make(self):self.capture=NativeMissionCapture(issuer=self.issuer,rgb=self.rgb,depth=self.dep,rgb_reference=self.rr,depth_reference=self.dr,read_epoch=lambda:(self.step,self.step/960),read_joint_positions=lambda:self.q.copy(),arm_indices=[0,1,2,3,4],joint_names=self.joints,render=self.render,clock=self.clock)
 def test_current_raw_owned_certificate_not_receipt(self):
  r=self.capture.acquire();self.assertTrue(r['captured']);self.assertFalse(r['motion_authorized']);self.assertFalse(r['IPC_receipts_created']);self.assertEqual(self.renders,1)
  for a in list(r['pixels'].values())+list(r['depths'].values()):self.assertTrue(a.flags.owndata);self.assertFalse(a.flags.writeable)
  first=next(iter(self.rgb));self.rgb[first].value[:]=255;self.assertEqual(r['pixels'][first].max(),0)
 def test_stale_exhausts_four_renders_without_retag(self):
  self.meta_step=48;r=self.capture.acquire();self.assertFalse(r['captured']);self.assertEqual(self.renders,4);self.assertEqual(r['reasons'],['native_ready_stale_epoch']*4);self.assertIsNone(r['pixels'])
 def test_future_rejected(self):
  self.meta_step=144;r=self.capture.acquire();self.assertFalse(r['captured']);self.assertIn('native_ready_future_epoch',r['reasons'])
 def test_partial_depth_epoch_rejected(self):
  self.dr[next(iter(self.dr))]=Ann({'referenceTimeNumerator':48,'referenceTimeDenominator':960});self.make();r=self.capture.acquire();self.assertFalse(r['captured']);self.assertIn('native_ready_partial_epoch_update',r['reasons'])
 def test_200ms_preserved(self):
  self.render_delta=201_000_000;r=self.capture.acquire();self.assertFalse(r['captured']);self.assertEqual(self.renders,1);self.assertIn('native_ready_wall_timeout',r['reasons'])
 def test_physics_step_change_rejected(self):
  self.change_step=True;r=self.capture.acquire();self.assertFalse(r['captured']);self.assertIn('physics_or_joint_changed_during_native_wait',r['reasons'])
 def test_q_change_rejected(self):
  self.change_q=True;r=self.capture.acquire();self.assertFalse(r['captured']);self.assertIn('physics_or_joint_changed_during_native_wait',r['reasons'])
 def test_bad_dtype_rejected(self):
  self.dep[next(iter(self.dep))].value=self.dep[next(iter(self.dep))].value.astype(np.float64);r=self.capture.acquire();self.assertFalse(r['captured']);self.assertIsNone(r['certificate'])
 def test_repeated_epoch_rejected(self):
  self.capture.acquire()
  with self.assertRaisesRegex(ValueError,'repeated_or_reset_physics_step'):self.capture.acquire()
 def test_current_feedback_is_independent(self):
  r=self.capture.acquire();old=copy.deepcopy(r['certificate']['body']['encoder_history']['encoder_q']);self.q[:5]=.2;f=self.capture.read_current_feedback();self.assertNotEqual(f['q'],old);self.assertEqual(r['certificate']['body']['encoder_history']['encoder_q'],old)
 def test_current_feedback_rejects_invalid_clock_without_issuer(self):
  for stamps in [(100,99),(-1,0),(True,1),(1,1.5)]:
   with self.subTest(stamps=stamps):
    values=iter(stamps);self.capture.clock=lambda:next(values)
    with self.assertRaisesRegex(ValueError,'invalid_current_feedback_clock'):
     self.capture.read_current_feedback()
 def test_each_physics_step_history_reused_without_reset_on_capture(self):
  for n in range(480):
   self.step=n;self.meta_step=n
   result=self.capture.record_physics_step(native_qd=[.001]*5,same_step_fd=[0.]*5)
  self.assertTrue(result['settled_interval'])
  captured=self.capture.acquire()
  self.assertTrue(captured['captured'])
  self.assertTrue(captured['certificate']['body']['encoder_history']['capture_settling']['settled_interval'])
 def test_motion_after_history_record_rejects_capture(self):
  self.capture.record_physics_step(native_qd=[0.]*5,same_step_fd=[0.]*5)
  self.q[0]=.01
  with self.assertRaisesRegex(ValueError,'encoder_changed_after_physics_record'):self.capture.acquire()
 def test_sparse_capture_resets_previous_contiguous_settling(self):
  for n in range(480):
   self.step=n
   self.capture.record_physics_step(native_qd=[0.]*5,same_step_fd=[0.]*5)
  self.step=500;self.meta_step=500
  r=self.capture.acquire()
  self.assertTrue(r['captured'])
  self.assertFalse(r['certificate']['body']['encoder_history']['capture_settling']['settled_interval'])
 def test_noncurrent_metadata_skips_pixel_reads(self):
  self.meta_step=48
  def forbidden():raise AssertionError('stale CPU pixel read')
  for ann in list(self.rgb.values())+list(self.dep.values()):ann.value=forbidden
  r=self.capture.acquire();self.assertFalse(r['captured']);self.assertEqual(self.renders,4)
 def test_current_epoch_after_skipped_frames_reads_once(self):
  self.meta_step=48;original=self.render;calls=[]
  def advance():
   original()
   if self.renders==3:self.meta_step=96
  self.capture.render=advance
  for name,ann in self.rgb.items():
   value=ann.value
   def read(name=name,value=value):calls.append(name);return value
   ann.value=read
  r=self.capture.acquire();self.assertTrue(r['captured']);self.assertEqual(len(calls),8)
 def test_epoch_mutation_during_pixels_still_rejected(self):
  ann=next(iter(self.rgb.values()));value=ann.value
  def mutate():self.meta_step=48;return value
  ann.value=mutate;r=self.capture.acquire()
  self.assertFalse(r['captured']);self.assertIn('native_ready_changed_during_readback',r['reasons'])
 def test_annotator_set_missing_rejected(self):
  self.rr.pop(next(iter(self.rr)))
  with self.assertRaisesRegex(ValueError,'set_mismatch'):self.make()
if __name__=='__main__':unittest.main()
