"""Prepare authenticated sensor publication. No world creation/control or fixtures."""
import time
from .publication_adapter import NativePublicationAdapter

class RuntimePublicationPrepare:
 def __init__(self,*,issuer,initial_step,refresh_packet,ros,priority,clock=time.monotonic_ns):
  if not callable(refresh_packet):raise ValueError('actual_postcapture_refresh_required')
  self.issuer=issuer;self.refresh_packet=refresh_packet;self.ros=ros;self.priority=priority;self.clock=clock;self.pending=None;self.sequence=0
  self.adapter=NativePublicationAdapter(run_id=issuer.run_id,initial_step=initial_step,verify_capture=self._verify)
 def _verify(self,certificate):
  if self.pending is None or certificate is not self.pending['certificate']:raise ValueError('capture_not_owned')
  sample=self.pending
  self.verification=self.issuer.verify(certificate,pixels=sample['pixels'],depths=sample['depths'],now_ns=self.clock(),expected_current_run=self.issuer.run_id)
  if self.verification.get('rgbd_capture_association_verified')is not True:raise ValueError('capture_authentication_failed')
  return certificate['body']
 def __call__(self,sample):
  if self.pending is not None:raise ValueError('prepare_pending')
  self.pending=sample
  # Refresh callback must perform actual control samples+IMUs after capture,
  # preserve history and return NativePacketTimeline packet. Adapter below
  # validates distinct epochs, sensor sample identity and original wall ages.
  packet=self.refresh_packet(sample)
  plan=self.adapter.prepare(packet=packet,certificate=sample['certificate'],now_ns=self.clock())
  self.sequence+=1
  self.priority.send(packet['current_feedback'])
  self.ros.publish_current(packet['current_feedback'],self.issuer.source,self.sequence)
  result=(plan,self.verification);self.pending=None
  return result
