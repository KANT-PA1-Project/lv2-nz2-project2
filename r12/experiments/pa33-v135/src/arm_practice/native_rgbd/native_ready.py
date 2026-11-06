"""Bounded exact-native-epoch readiness, never a capture certificate or new clock."""
from fractions import Fraction
MAX_RENDERS=4
MAX_READY_NS=200_000_000

def integer(x):return type(x)is int and x>=0
class NativeReadyWindow:
 def __init__(self,*,physics_step,physics_hz,cameras,depths,encoder_read_start_ns,max_renders=MAX_RENDERS,max_ready_ns=MAX_READY_NS):
  if not integer(physics_step)or not integer(encoder_read_start_ns)or type(physics_hz)is not int or physics_hz<=0:raise ValueError('invalid_native_ready_epoch')
  if type(max_renders)is not int or not 1<=max_renders<=MAX_RENDERS or type(max_ready_ns)is not int or not 1<=max_ready_ns<=MAX_READY_NS:raise ValueError('invalid_native_ready_budget')
  if len(cameras)!=8 or len(set(cameras))!=8 or len(depths)!=4 or len(set(depths))!=4:raise ValueError('native_ready_requires8RGB4depth')
  self.step=physics_step;self.hz=physics_hz;self.cameras=set(cameras);self.depths=set(depths);self.start_ns=encoder_read_start_ns;self.last_ns=encoder_read_start_ns;self.max_renders=max_renders;self.max_ready_ns=max_ready_ns;self.attempts=0;self.assessed_attempts=0;self.ready=False
 def _clock(self,now_ns,physics_step):
  if not integer(now_ns)or now_ns<self.last_ns:raise ValueError('native_ready_clock_reset')
  if physics_step!=self.step or type(physics_step)is not int:raise ValueError('physics_advanced_during_native_wait')
  self.last_ns=now_ns
  if now_ns-self.start_ns>self.max_ready_ns:raise ValueError('native_ready_wall_timeout')
 def begin_render(self,*,now_ns,physics_step):
  self._clock(now_ns,physics_step)
  if self.ready:raise ValueError('native_ready_already_consumed')
  if self.attempts!=self.assessed_attempts:raise ValueError('native_ready_previous_readback_unassessed')
  if self.attempts>=self.max_renders:raise ValueError('native_ready_render_budget_exhausted')
  self.attempts+=1
 def _step(self,raw):
  if not isinstance(raw,dict):raise ValueError('native_ready_metadata_missing')
  n=raw.get('referenceTimeNumerator');d=raw.get('referenceTimeDenominator')
  if not integer(n)or not integer(d)or d==0:raise ValueError('native_ready_invalid_reference_time')
  value=Fraction(n,d);step=round(value*self.hz)
  if abs(value-Fraction(step,self.hz))>Fraction(1,1_000_000_000):raise ValueError('native_ready_unaligned_reference_time')
  return step
 def assess(self,*,before,after,now_ns,physics_step):
  result={'ready':False,'motion_authorized':False,'certificate':None,'attempts':self.attempts,'target_physics_step':self.step,'encoder_read_start_ns':self.start_ns}
  try:
   self._clock(now_ns,physics_step)
   if self.ready or not 1<=self.attempts<=self.max_renders or self.assessed_attempts==self.attempts:raise ValueError('native_ready_attempt_state_invalid')
   self.assessed_attempts=self.attempts
   if set(before)!= {'rgb','depth'} or set(before['rgb'])!=self.cameras or set(before['depth'])!=self.depths:raise ValueError('native_ready_sensor_set_mismatch')
   if after!=before:raise ValueError('native_ready_changed_during_readback')
   epochs=[self._step(v)for group in ['rgb','depth']for v in before[group].values()]
   result['native_physics_steps']=epochs
   if any(s>self.step for s in epochs):raise ValueError('native_ready_future_epoch')
   if len(set(epochs))!=1:raise ValueError('native_ready_partial_epoch_update')
   if epochs[0]!=self.step:raise ValueError('native_ready_stale_epoch')
   self.ready=True;result.update(ready=True,reason='all8RGB4depth_match_current_recorded_physics_epoch',ready_age_ms=(now_ns-self.start_ns)/1e6)
  except (ValueError,TypeError,KeyError)as e:result['reason']=str(e)
  return result
