"""Producer-owned native-render time / historical encoder association.

No simulator truth input. Epoch history is recorded by the sensor producer, never
imported from ROS packets. A process-local HMAC binds pixels, actual historical
encoder/FK, run/clock, model/source hashes and native per-camera ReferenceTime.
The HMAC key is not serialized; archives alone cannot authorize live control.
This binds RGB only: depth and cross-process trust are not verified.
This is a sensor association proof, never collision/grasp/motion authorization.
"""
from collections import OrderedDict
from fractions import Fraction
import hashlib,hmac,json,math,secrets
import numpy as np


def canonical(x):return json.dumps(x,sort_keys=True,separators=(',',':'),allow_nan=False).encode()
def digest_image(x):
 a=np.asarray(x)
 if a.dtype!=np.uint8 or a.shape!=(480,640,3):raise ValueError('invalid_rgb_pixels')
 return hashlib.sha256(a.tobytes(order='C')).hexdigest()
def integer(x):return type(x) is int and x>=0

def rigid(x):
 a=np.asarray(x,float)
 if a.shape!=(4,4) or not np.isfinite(a).all() or not np.allclose(a[3],[0,0,0,1],atol=1e-9) or not np.allclose(a[:3,:3].T@a[:3,:3],np.eye(3),atol=1e-6) or abs(np.linalg.det(a[:3,:3])-1)>1e-6:raise ValueError('invalid_encoder_fk')
 return a.tolist()

class RendererEpochIssuer:
 def __init__(self,*,run_id,clock_epoch,cameras,joint_names,physics_hz,model_sha256,source_sha256,fk,history_size=128):
  if type(history_size) is not int or not 1<=history_size<=4096:raise ValueError('invalid_history_size')
  if not run_id or not clock_epoch:raise ValueError('missing_session')
  if type(physics_hz) is not int or physics_hz<=0:raise ValueError('invalid_physics_rate')
  if len(cameras)!=len(set(cameras)) or not cameras or len(joint_names)!=len(set(joint_names)) or not joint_names:raise ValueError('duplicate_or_missing_names')
  for s in [model_sha256,source_sha256]:
   if type(s) is not str or len(s)!=64 or any(c not in '0123456789abcdef' for c in s):raise ValueError('missing_pinned_hash')
  self.run_id=run_id;self.epoch=clock_epoch;self.cameras=tuple(cameras);self.joints=tuple(joint_names);self.hz=physics_hz;self.model=model_sha256;self.source=source_sha256;self.fk=fk;self.history=OrderedDict();self.history_size=history_size;self.last_step=-1;self.last_issue=-1;self.last_wall=-1;self.last_verified=-1;self.secret=secrets.token_bytes(32);self.issuer_id=secrets.token_hex(16)
 def record_encoder_epoch(self,*,physics_step,sim_time_s,read_start_ns,read_end_ns,measured_encoder):
  if not all(integer(v) for v in [physics_step,read_start_ns,read_end_ns]) or not self.last_wall<read_start_ns<=read_end_ns or read_end_ns-read_start_ns>1_000_000:raise ValueError('invalid_encoder_read_interval')
  if physics_step<=self.last_step:raise ValueError('repeated_or_reset_physics_step')
  if not math.isfinite(sim_time_s) or abs(sim_time_s-physics_step/self.hz)>1/(4*self.hz):raise ValueError('physics_time_step_mismatch')
  q=np.asarray(measured_encoder,float)
  if q.shape!=(len(self.joints),) or not np.isfinite(q).all():raise ValueError('invalid_measured_encoder')
  tf=self.fk(dict(zip(self.joints,q)))
  if set(tf)!=set(self.cameras):raise ValueError('encoder_fk_camera_set_mismatch')
  item=dict(physics_step=physics_step,canonical_sim_time_num=physics_step,canonical_sim_time_den=self.hz,reported_sim_time_s=float(sim_time_s),encoder_read_start_ns=read_start_ns,encoder_read_end_ns=read_end_ns,joint_names=list(self.joints),encoder_q=q.tolist(),camera_fk={k:rigid(v) for k,v in tf.items()})
  self.history[physics_step]=item;self.last_step=physics_step;self.last_wall=read_end_ns
  while len(self.history)>self.history_size:self.history.popitem(last=False)
  return physics_step
 def _render_step(self,raw):
  if not isinstance(raw,dict):raise ValueError('missing_native_render_metadata')
  n=raw.get('referenceTimeNumerator');d=raw.get('referenceTimeDenominator')
  if not integer(n) or not integer(d) or d==0:raise ValueError('invalid_native_render_time')
  step=Fraction(n,d)*self.hz;rounded=round(step)
  if abs(Fraction(n,d)-Fraction(rounded,self.hz))>Fraction(1,1_000_000_000):raise ValueError('renderer_time_not_physics_epoch')
  return rounded
 def issue(self,*,native_camera_metadata,pixels,current_physics_step,readback_return_ns,now_ns):
  if not all(integer(v) for v in [current_physics_step,readback_return_ns,now_ns]) or not readback_return_ns<=now_ns:raise ValueError('invalid_readback_clock')
  if set(native_camera_metadata)!=set(self.cameras) or set(pixels)!=set(self.cameras):raise ValueError('camera_set_mismatch')
  steps={self._render_step(v) for v in native_camera_metadata.values()}
  if len(steps)!=1:raise ValueError('mixed_camera_render_epochs')
  step=steps.pop()
  if current_physics_step!=self.last_step or step>current_physics_step:raise ValueError('current_physics_epoch_not_recorded')
  if step<=self.last_issue:raise ValueError('repeated_or_reset_render_epoch')
  if step not in self.history:raise ValueError('historical_encoder_epoch_missing')
  history=self.history[step]
  # Conservative lower bound includes time since historical state acquisition,
  # not merely the later host get_data() call or current encoder read.
  lower=history['encoder_read_start_ns']
  if not history['encoder_read_end_ns']<=readback_return_ns<=now_ns or now_ns-lower>500_000_000:raise ValueError('historical_capture_stale_or_clock_invalid')
  if (current_physics_step-step)/self.hz>.5:raise ValueError('historical_sensor_clock_stale')
  body=dict(schema='native-render-encoder-association-v1',issuer_id=self.issuer_id,run_id=self.run_id,clock_epoch=self.epoch,physics_hz=self.hz,current_physics_step=current_physics_step,capture_physics_step=step,model_sha256=self.model,source_sha256=self.source,encoder_history=json.loads(canonical(history)),native_camera_metadata=json.loads(canonical(native_camera_metadata)),pixel_sha256={k:digest_image(v) for k,v in pixels.items()},capture_monotonic_lower_bound_ns=lower,readback_return_ns=readback_return_ns,issued_ns=now_ns,scope='sensor_capture_association_only',motion_authorized=False)
  sealed={'body':body,'signature':hmac.new(self.secret,canonical(body),hashlib.sha256).hexdigest()};self.last_issue=step
  return sealed
 def verify(self,sealed,*,pixels,now_ns,expected_current_run=None):
  result=dict(matched=False,image_capture_association_verified=False,depth_capture_association_verified=False,rgbd_capture_association_verified=False,motion_authorized=False)
  try:
   b=sealed['body'];signature=sealed['signature']
   if not isinstance(signature,str) or not hmac.compare_digest(signature,hmac.new(self.secret,canonical(b),hashlib.sha256).hexdigest()):raise ValueError('untrusted_or_modified_issuer_certificate')
   if b['issuer_id']!=self.issuer_id or b['run_id']!=self.run_id or b['clock_epoch']!=self.epoch or expected_current_run not in (None,self.run_id):raise ValueError('issuer_session_mismatch')
   if b['capture_physics_step']<=self.last_verified:raise ValueError('duplicate_capture_at_consumer')
   if not integer(now_ns) or not b['issued_ns']<=now_ns or now_ns-b['capture_monotonic_lower_bound_ns']>500_000_000:raise ValueError('capture_stale_at_consumer')
   if set(pixels)!=set(self.cameras) or any(digest_image(pixels[k])!=v for k,v in b['pixel_sha256'].items()):raise ValueError('pixels_changed_or_camera_missing')
   self.last_verified=b['capture_physics_step']
   return dict(result,matched=True,image_capture_association_verified=True,capture_physics_step=b['capture_physics_step'],capture_monotonic_lower_bound_ns=b['capture_monotonic_lower_bound_ns'],encoder_history=b['encoder_history'],run_id=self.run_id,clock_epoch=self.epoch,reason='native_render_epoch_joined_to_recorded_encoder')
  except (KeyError,ValueError,TypeError,OverflowError) as e:return dict(result,reason=str(e))
