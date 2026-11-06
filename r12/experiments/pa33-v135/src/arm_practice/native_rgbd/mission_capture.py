"""Inactive mission producer adapter for native raw RGBD acquisition.
The producer owns annotators, issuer key, encoder read and render callback. This
never moves physics, fabricates IDs, authorizes motion or creates IPC receipts.
Current local feedback remains a separate fresh read for TrustedCaptureChannel.
"""
import time
import numpy as np
from .owned_rgb import owned_rgb
from .native_ready import NativeReadyWindow,MAX_RENDERS

def plain(x):
 if isinstance(x,dict):return {str(k):plain(v)for k,v in x.items()}
 if isinstance(x,(tuple,list)):return [plain(v)for v in x]
 if isinstance(x,np.ndarray):return x.tolist()
 if isinstance(x,np.generic):return x.item()
 return x

class NativeMissionCapture:
 def __init__(self,*,issuer,rgb,depth,rgb_reference,depth_reference,read_epoch,read_joint_positions,arm_indices,joint_names,render,clock=time.monotonic_ns):
  if set(rgb)!=set(issuer.cameras)or set(rgb_reference)!=set(rgb)or set(depth)!=set(issuer.depth_map)or set(depth_reference)!=set(depth):raise ValueError('native_annotator_set_mismatch')
  if list(joint_names)!=list(issuer.joints)or list(joint_names)!=[f'arm_joint{i}'for i in range(1,6)]:raise ValueError('native_joint_mapping_mismatch')
  if len(arm_indices)!=5 or len(set(arm_indices))!=5 or any(type(i)is not int or i<0 for i in arm_indices):raise ValueError('native_arm_index_mapping_invalid')
  self.issuer=issuer;self.rgb=dict(rgb);self.depth=dict(depth);self.rgb_ref=dict(rgb_reference);self.depth_ref=dict(depth_reference);self.epoch=read_epoch;self.positions=read_joint_positions;self.indices=list(arm_indices);self.render=render;self.clock=clock;self.last_acquire_step=-1;self.stage_timings=[]
 def timed_annotation(self,name,ann,device=None):
  start=self.clock();value=ann.get_data() if device is None else ann.get_data(device=device);end=self.clock()
  self.stage_timings.append(dict(stage=name,start_ns=start,end_ns=end))
  return value
 def timed_render(self):
  # Python proxy observes the exact original app.update once, without extra update.
  owner=getattr(self.render,'__self__',None);app=getattr(owner,'_app',None)
  if app is None:return self.render()
  timings=self.stage_timings;clock=self.clock
  class AppTimingProxy:
   def __getattr__(self,name):return getattr(app,name)
   def update(self,*args,**kwargs):
    start=clock()
    try:return app.update(*args,**kwargs)
    finally:timings.append(dict(stage='render_app_update',start_ns=start,end_ns=clock()))
  owner._app=AppTimingProxy()
  try:return self.render()
  finally:owner._app=app
 def read_current_feedback(self):
  step,t=self.epoch();start=self.clock();q=np.asarray(self.positions()).copy();end=self.clock()
  if type(start)is not int or type(end)is not int or start<0 or end<start:
   raise ValueError('invalid_current_feedback_clock')
  after=self.epoch()
  if after!=(step,t):raise ValueError('physics_advanced_during_encoder_read')
  if q.ndim!=1 or max(self.indices)>=len(q)or not np.isfinite(q).all():raise ValueError('invalid_native_joint_feedback')
  measured=np.rint(q[self.indices]*4096/(2*np.pi))*2*np.pi/4096
  return {'q':measured.tolist(),'read_start_ns':start,'read_end_ns':end,'physics_step':step}
 def acquire(self):
  """Bounded native epoch. Read owned pixels only after exact native reference epoch readiness."""
  f=self.read_current_feedback();step,t=self.epoch()
  if step!=f['physics_step']:raise ValueError('physics_advanced_before_history_record')
  if step<=self.last_acquire_step:raise ValueError('repeated_or_reset_physics_step')
  self.last_acquire_step=step
  if self.issuer.last_step==step:
   history=self.issuer.history[step]
   if not np.array_equal(np.asarray(history['encoder_q']),np.asarray(f['q'])):raise ValueError('same_step_encoder_changed_before_capture')
   f=dict(f,read_start_ns=history['encoder_read_start_ns'],read_end_ns=history['encoder_read_end_ns'])
  else:
   self.issuer.record_encoder_epoch(physics_step=step,sim_time_s=t,read_start_ns=f['read_start_ns'],read_end_ns=f['read_end_ns'],measured_encoder=f['q'])
  window=NativeReadyWindow(physics_step=step,physics_hz=self.issuer.hz,cameras=list(self.rgb),depths=list(self.depth),encoder_read_start_ns=f['read_start_ns'])
  reasons=[];trace=[]
  for _ in range(MAX_RENDERS):
   attempt=dict(target_step=step,begin_ns=self.clock(),full_readback=False)
   trace.append(attempt)
   try:
    window.begin_render(now_ns=self.clock(),physics_step=self.epoch()[0]);attempt['attempt']=window.attempts;epoch0=self.epoch();q0=np.asarray(self.positions()).copy();attempt['physics_before_render']=list(epoch0);attempt['render_start_ns']=self.clock();self.timed_render();attempt['render_end_ns']=self.clock()
    before={'rgb':{k:plain(self.timed_annotation("RGB_reference:"+k,a))for k,a in self.rgb_ref.items()},'depth':{k:plain(self.timed_annotation("depth_reference:"+k,a))for k,a in self.depth_ref.items()}}
    attempt['metadata_before']=before;attempt['metadata_read_ns']=self.clock()
    try:
     epochs=[window._step(v)for group in ['rgb','depth']for v in before[group].values()]
     metadata_current=all(epoch==step for epoch in epochs)
     attempt['native_physics_steps']=epochs
    except (ValueError,TypeError,KeyError):metadata_current=False
    pixels=depths=None
    # Stale/future/mixed reference metadata cannot certify owned current pixels.
    if metadata_current:
     attempt['full_readback']=True;attempt['readback_start_ns']=self.clock()
     pixels={k:owned_rgb(self.timed_annotation('RGB:'+k,a,'cpu'))for k,a in self.rgb.items()};attempt['rgb_read_end_ns']=self.clock();depths={k:np.asarray(self.timed_annotation('depth:'+k,a,'cpu')).copy()for k,a in self.depth.items()}
     attempt['readback_end_ns']=self.clock()
    else:attempt['readback_skipped_reason']='metadata_not_target_epoch'
    after={'rgb':{k:plain(self.timed_annotation("RGB_reference:"+k,a))for k,a in self.rgb_ref.items()},'depth':{k:plain(self.timed_annotation("depth_reference:"+k,a))for k,a in self.depth_ref.items()}};readback_end=self.clock();attempt['metadata_after']=after;attempt['physics_after_readback']=list(self.epoch())
    try:attempt['native_physics_steps_after']=[window._step(v)for group in ['rgb','depth']for v in after[group].values()]
    except (ValueError,TypeError,KeyError):attempt['native_physics_steps_after']=None
    if self.epoch()!=epoch0 or not np.array_equal(q0,self.positions()):raise ValueError('physics_or_joint_changed_during_native_wait')
    verdict=window.assess(before=before,after=after,now_ns=readback_end,physics_step=self.epoch()[0]);reasons.append(verdict['reason']);attempt.update(reason=verdict['reason'],assessed_ns=readback_end,age_ms=(readback_end-f['read_start_ns'])/1e6)
    if not verdict['ready']:continue
    if pixels is None or depths is None:raise ValueError('native_ready_without_owned_readback')
    for a in list(pixels.values())+list(depths.values()):
     if not a.flags.owndata or not a.flags.c_contiguous:raise ValueError('native_capture_requires_owned_contiguous_bytes')
     a.flags.writeable=False
    issue_start=self.clock()
    sealed=self.issuer.issue(native_camera_metadata=before['rgb'],pixels=pixels,native_depth_metadata=before['depth'],depths=depths,native_metadata_after=after,current_physics_step=step,readback_return_ns=readback_end,now_ns=self.clock())
    self.stage_timings.append(dict(stage='issuer_hash_HMAC',start_ns=issue_start,end_ns=self.clock()))
    return {'captured':True,'certificate':sealed,'pixels':pixels,'depths':depths,'attempts':window.attempts,'ready_age_ms':verdict['ready_age_ms'],'motion_authorized':False,'IPC_receipts_created':False,'ready_trace':trace}
   except ValueError as e:
    reasons.append(str(e));attempt.update(reason=str(e),failure_ns=self.clock());break
  return {'captured':False,'certificate':None,'pixels':None,'depths':None,'attempts':window.attempts,'reasons':reasons,'motion_authorized':False,'IPC_receipts_created':False,'ready_trace':trace}
