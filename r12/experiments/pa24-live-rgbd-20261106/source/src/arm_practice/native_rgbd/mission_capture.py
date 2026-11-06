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
  self.issuer=issuer;self.rgb=dict(rgb);self.depth=dict(depth);self.rgb_ref=dict(rgb_reference);self.depth_ref=dict(depth_reference);self.epoch=read_epoch;self.positions=read_joint_positions;self.indices=list(arm_indices);self.render=render;self.clock=clock;self.recorded_step=None;self.captured_step=None
 def read_current_feedback(self):
  step,t=self.epoch();start=self.clock();q=np.asarray(self.positions()).copy();end=self.clock()
  if type(start)is not int or type(end)is not int or start<0 or end<start:
   raise ValueError('invalid_current_feedback_clock')
  after=self.epoch()
  if after!=(step,t):raise ValueError('physics_advanced_during_encoder_read')
  if q.ndim!=1 or max(self.indices)>=len(q)or not np.isfinite(q).all():raise ValueError('invalid_native_joint_feedback')
  measured=np.rint(q[self.indices]*4096/(2*np.pi))*2*np.pi/4096
  return {'q':measured.tolist(),'read_start_ns':start,'read_end_ns':end,'physics_step':step}
 def record_physics_step(self, *, native_qd, same_step_fd):
  """Producer hook after each physics step; velocities must share that epoch.

  Use unquantized native positions for FD upstream; quantized encoder FK is
  retained independently. Sparse samples cannot satisfy the continuous dwell.
  """
  f=self.read_current_feedback();step,t=self.epoch()
  if step!=f['physics_step']:raise ValueError('physics_advanced_before_history_record')
  self.issuer.record_encoder_epoch(physics_step=step,sim_time_s=t,
   read_start_ns=f['read_start_ns'],read_end_ns=f['read_end_ns'],
   measured_encoder=f['q'],native_qd=native_qd,same_step_fd=same_step_fd)
  self.recorded_step=step
  return dict(self.issuer.history[step]['capture_settling'])
 def acquire(self):
  """One bounded current epoch, owned unmasked bytes. Failure returns no buffer."""
  f=self.read_current_feedback();step,t=self.epoch()
  if step!=f['physics_step']:raise ValueError('physics_advanced_before_history_record')
  if self.captured_step==step:raise ValueError('repeated_or_reset_physics_step')
  if self.recorded_step==step:
   historical=self.issuer.history[step]
   if historical['encoder_q']!=f['q']:raise ValueError('encoder_changed_after_physics_record')
   f={'read_start_ns':historical['encoder_read_start_ns']}
  else:
   self.issuer.record_encoder_epoch(physics_step=step,sim_time_s=t,read_start_ns=f['read_start_ns'],read_end_ns=f['read_end_ns'],measured_encoder=f['q'])
  self.captured_step=step
  window=NativeReadyWindow(physics_step=step,physics_hz=self.issuer.hz,cameras=list(self.rgb),depths=list(self.depth),encoder_read_start_ns=f['read_start_ns'])
  reasons=[];diagnostics=[]
  for _ in range(MAX_RENDERS):
   try:
    window.begin_render(now_ns=self.clock(),physics_step=self.epoch()[0]);epoch0=self.epoch();q0=np.asarray(self.positions()).copy();self.render()
    before={'rgb':{k:plain(a.get_data())for k,a in self.rgb_ref.items()},'depth':{k:plain(a.get_data())for k,a in self.depth_ref.items()}}
    metadata_current=False
    try:
     epochs=[window._step(v)for group in ('rgb','depth')for v in before[group].values()]
     metadata_current=len(epochs)==len(self.rgb)+len(self.depth) and all(e==step for e in epochs)
    except (ValueError,TypeError,KeyError):pass
    pixels=depths=None
    if metadata_current:
     pixels={k:owned_rgb(a.get_data(device='cpu'))for k,a in self.rgb.items()};depths={k:np.asarray(a.get_data(device='cpu')).copy()for k,a in self.depth.items()}
    after={'rgb':{k:plain(a.get_data())for k,a in self.rgb_ref.items()},'depth':{k:plain(a.get_data())for k,a in self.depth_ref.items()}};readback_end=self.clock()
    if self.epoch()!=epoch0 or not np.array_equal(q0,self.positions()):raise ValueError('physics_or_joint_changed_during_native_wait')
    verdict=window.assess(before=before,after=after,now_ns=readback_end,physics_step=self.epoch()[0]);reasons.append(verdict['reason'])
    diagnostics.append({'attempt':window.attempts,'target_physics_step':step,'elapsed_ms':(readback_end-window.start_ns)/1e6,'before':before,'after':after,'verdict':plain(verdict)})
    if not verdict['ready']:continue
    for a in list(pixels.values())+list(depths.values()):
     if not a.flags.owndata or not a.flags.c_contiguous:raise ValueError('native_capture_requires_owned_contiguous_bytes')
     a.flags.writeable=False
    sealed=self.issuer.issue(native_camera_metadata=before['rgb'],pixels=pixels,native_depth_metadata=before['depth'],depths=depths,native_metadata_after=after,current_physics_step=step,readback_return_ns=readback_end,now_ns=self.clock())
    return {'captured':True,'certificate':sealed,'pixels':pixels,'depths':depths,'attempts':window.attempts,'ready_age_ms':verdict['ready_age_ms'],'diagnostics':diagnostics,'motion_authorized':False,'IPC_receipts_created':False}
   except ValueError as e:
    diagnostics.append({'attempt':window.attempts,'target_physics_step':step,'error':str(e)})
    reasons.append(str(e));break
  return {'captured':False,'certificate':None,'pixels':None,'depths':None,'attempts':window.attempts,'reasons':reasons,'diagnostics':diagnostics,'motion_authorized':False,'IPC_receipts_created':False}
