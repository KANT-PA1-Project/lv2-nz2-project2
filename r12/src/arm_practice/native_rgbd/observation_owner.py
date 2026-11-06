"""Inactive same-thread owner prototype. Inject trusted adapters, never fixture setup.
Does not construct authority, move physics, or implement motion controls.
"""
class ObservationOwner:
 def __init__(self,*,capture,prepare_publication,decoder,ros):
  self.capture=capture;self.prepare=prepare_publication;self.decoder=decoder;self.ros=ros;self.state='idle';self.pending=None
 def observe(self,frame):
  if self.state!='idle' or self.pending is not None:raise ValueError('owner_not_idle')
  self.state='capturing'
  try:
   sample=self.capture.acquire()
   if not sample['captured']:
    self.state='idle';return dict(captured=False,reasons=sample.get('reasons',[]),motion_authorized=False)
   self.pending=sample;self.state='authenticating'
   # Adapter must verify certificate and obtain real current feedback/required
   # IMUs before creating plan. No cached replacement is permitted by contract.
   plan,verification=self.prepare(sample)
   self.decoder.send(sample['certificate'],sample['pixels'],sample['depths'],verification,frame)
   self.decoder.verify_before_publication();self.state='publishing'
   ros_result=self.ros.publish(sample['certificate'],sample['pixels'],sample['depths'],plan,verification,decoder=self.decoder)
   if ros_result.get('all_received')is not True:raise ValueError('required_ROS_incomplete')
   self.state='collecting';decoded=self.decoder.collect(frame)
   result=dict(captured=True,ROS=ros_result,decoded=decoded,motion_authorized=False)
   self.pending=None;self.state='idle';return result
  except Exception:
   # Retain owned bytes/evidence and prohibit next capture. Explicit external
   # teardown/recovery required; a failed transaction cannot silently restart.
   self.state='failed';raise
