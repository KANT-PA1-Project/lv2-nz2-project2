"""Live diagnostic receipt join. No pose uncertainty or semantic authority invented."""
import os,time,copy,hashlib,json
from fractions import Fraction
import numpy as np
from .separated_sensor_inputs import SeparatedSensorInputs
from .trusted_capture_ipc import make_request,send_packet
from .camera_geometry import geometry as pixel_geometry
from .combine import shadow
from .semantic_candidate import IdentityCandidate

class LiveShadowInputs:
 def __init__(self,sock,pid,run,schema):self.sock=sock;self.pid=pid;self.run=run;self.schema=schema;self.inputs=None;self.session=None;self.semantic=IdentityCandidate(0)
 def begin(self,msg,arrays):
  s=msg['receipt_session'];b=msg['certificate']['body'];session={k:v for k,v in s.items()if k!='nonce'}
  if self.inputs is None:
   self.session=session;self.inputs=SeparatedSensorInputs(self.sock,producer_pid=self.pid,uid=os.getuid(),run_id=self.run,clock_epoch=s['clock_epoch'],source_sha256=s['source_sha256'],model_sha256=s['model_sha256'],publisher_gid=s['publisher_gid'],feedback_stream_id=s['feedback_stream_id'],max_pending_captures=1,max_pending_feedback=1,camera_schema=self.schema)
  if session!=self.session:raise ValueError('shadow_session_changed')
  pixels={k:arrays[k]for k in b['pixel_sha256']};depths={k:arrays[k]for k in b['depth_sha256']}
  request=make_request(msg['certificate'],pixels,depths,nonce=s['nonce'],gid=s['publisher_gid'],camera_schema=self.schema);send_packet(self.sock,request)
  _,ch,fh=self.inputs.receive_response(request,sealed=msg['certificate'],pixels=pixels,depths=depths)
  if ch is None or fh is None:raise ValueError('shadow_capture_receipt_denied')
  return dict(msg=msg,arrays=arrays,request=request,pixels=pixels,depths=depths,ch=ch,fh=fh,received_ns=time.monotonic_ns())
 def finish(self,context,tags,depth,K,decoded,streak):
  try:return self._finish(context,tags,depth,K,decoded,streak)
  except Exception:
   self.semantic.reset()
   raise
 def _finish(self,context,tags,depth,K,decoded,streak):
  start=time.monotonic_ns();result=shadow(tags,depth,K,shape='cuboid',decoded=decoded);ctx=context;msg=ctx['msg'];b=msg['certificate']['body'];raw=b['native_camera_metadata']['arm_camera_left_optical'];stamp=round(Fraction(raw['referenceTimeNumerator'],raw['referenceTimeDenominator'])*1000000000)
  # Actual post-inference local read from producer; never relabel historical q/time.
  self.inputs.discard_pair(None,ctx['fh']);request=dict(ctx['request'],kind='refresh_local_feedback');refresh_send_start=time.monotonic_ns();send_packet(self.sock,request);refresh_send_end=time.monotonic_ns()
  _,_,fh=self.inputs.receive_response(request,sealed=msg['certificate'],pixels=ctx['pixels'],depths=ctx['depths'])
  refresh_response_end=time.monotonic_ns()
  result['refresh_client_timing']=dict(request_send_start_ns=refresh_send_start,request_send_end_ns=refresh_send_end,response_validated_ns=refresh_response_end)
  ctx['fh']=fh  # Caller discards the newest receipt if the final join rejects.
  frame=dict(ctx['arrays'],**self.schema.archive_fields(),run_id=np.array(self.run),frame_index=np.array(b['capture_physics_step']),capture_monotonic_ns=np.array(b['capture_monotonic_lower_bound_ns']),stamp_ns=np.array(stamp),clock_domain=np.array('native_render_reference_time_v2'))
  packet=self.inputs.local_packet(fh);packet.update(capture_run_id=self.run,stamp_ns=stamp,frame_capture_monotonic_ns=b['capture_monotonic_lower_bound_ns'],publisher_gid=self.session['publisher_gid'],packet_received_monotonic_ns=ctx['received_ns'],camera_transforms=b['encoder_history']['camera_fk'])
  pair=self.inputs.join(frame,packet,capture_receipt=ctx['ch'],feedback_receipt=fh,now_ns=time.monotonic_ns());done=time.monotonic_ns()
  result.update(perception_envelope=dict(run_id=self.run,stamp_ns=stamp,capture_monotonic_ns=b['capture_monotonic_lower_bound_ns'],camera='arm_camera',camera_frame='arm_camera_left_optical',pixel_geometry=pixel_geometry(self.schema)),identity_evidence=dict(id=result['id'],run_id=self.run,stamp_ns=stamp,wrist_consistent_frames=streak,evidence_role='diagnostic_distinct_ID_streak_not_semantic_authority'),live_input_audit=dict(capture_receipt_consumed=ctx['ch']not in self.inputs.captures,feedback_receipt_consumed=fh not in self.inputs.feedback,historical_physics_step=pair['historical_encoder']['physics_step'],current_physics_step=pair['current_local_feedback']['physics_step'],current_read_start_ns=pair['current_local_feedback']['read_start_ns'],current_age_ms=(done-pair['current_local_feedback']['read_start_ns'])/1e6,capture_age_ms=(done-b['capture_monotonic_lower_bound_ns'])/1e6,RGBD_hashes_rechecked=True,read_timestamps_preserved=True,publisher_gid_role='authenticated_diagnostic_IPC_session_not_ROS_gid'),shadow_and_refresh_ms=(done-start)/1e6,adapter_ready=False,missing=['validated pose confidence','validated translation and rotation error bounds','unique semantic box association','authoritative confirmed identity and symmetry-alignment policy'])
  # The sole real join above has consumed process-local receipts and rehashed
  # the capture. Freeze the original detector output; no caller "joined" flag.
  original={k:copy.deepcopy(decoded[k]) for k in ('run_id','frame','capture_ns','detections','tags')}
  frozen=json.dumps(original,sort_keys=True,separators=(',',':'),allow_nan=False).encode()
  diagnostic=dict(original,live_shadow=result,shadow_fresh_at_completion=(time.monotonic_ns()-b['capture_monotonic_lower_bound_ns'])<=500_000_000)
  semantic=self.semantic.update(diagnostic)
  semantic.update(original_result_sha256=hashlib.sha256(frozen).hexdigest(),dual_plane_result_sha256=hashlib.sha256(json.dumps(result['pose_geometry'],sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest(),binding='computed_depth_geometry_then_authenticated_one_use_join_then_semantic_diagnostic',same_ID_other_object_uniqueness_proven=False)
  final_ns=time.monotonic_ns()
  if final_ns-b['capture_monotonic_lower_bound_ns']>500_000_000 or final_ns-pair['current_local_feedback']['read_start_ns']>100_000_000:
   self.semantic.reset();raise ValueError('semantic_completion_stale')
  semantic['completed_monotonic_ns']=final_ns
  result['semantic_candidate']=semantic
  return result
