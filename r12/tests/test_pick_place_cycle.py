import unittest,dataclasses
import numpy as np
from arm_practice.pick_place_cycle import PickPlaceCycle,Observation,Clearance
class Harness:
 def __init__(self,mode='positive'):
  self.t=0.;self.frame=0;self.mode=mode;self.q=np.zeros(5);self.obj=np.array([.3,0,.5]);self.tcp=self.obj.copy();self.aperture=.075;self.traces=[]
  self.c=PickPlaceCycle(3,dict(approach=[0]*5,lift=[.01,0,0,0,0],place=[0]*5,retreat=[0,.01,0,0,0],refold=[0]*5),[-1]*5,[1]*5,self.clearance,placement_base_m=[.3,0,.5])
 def clearance(self,phase,q,proposal,width,obs):
  return Clearance(self.mode!='unknown',obs.capture_ns,phase,self.mode!='unknown',.03,3 if phase in self.c.CONTACT_PHASES else None,('left_pad','right_pad') if phase in self.c.CONTACT_PHASES else ())
 def obs(self):return Observation('sensor_run',self.frame,round(self.t*1e9),round(self.t*1e9),3,'arm_camera_left_optical','wrist_rgbd_encoder_fk',tuple(self.obj),tuple(self.tcp),True,True,.001,0.)
 def step(self,obs=None):
  phase=self.c.phase
  if phase in ('lift','hold'):
   self.tcp[2]=.54
   if self.mode!='no_lift':self.obj[2]=.54
  elif phase=='place':self.obj[2]=self.tcp[2]=.5
  elif phase in ('retreat','verify_release','refold'):
   self.tcp[0]=.35
   if self.mode=='stuck':self.obj[0]=.35
  if self.c.reference is not None:self.q=self.c.reference.copy()
  self.aperture=max(self.c.width,.030) if phase in self.c.CONTACT_PHASES and self.mode!='empty' else self.c.width
  result=self.c.step(self.t,self.q,[0]*5,self.aperture,0.,feedback_capture_ns=round(self.t*1e9),base_stopped=True,observation=obs or self.obs());self.traces.append(result);self.t+=.05;self.frame+=1;return result
 def run(self):
  for _ in range(800):
   r=self.step()
   if r['phase'] in ('failed','complete'):return r
  raise AssertionError('not terminal')
class TestCycle(unittest.TestCase):
 def test_sensor_and_contact_ids_require_integer_identity(self):
  # Python equality otherwise accepts 3.0 as 3 and True as target 1.
  for target,invalid in [(3,3.0),(1,True)]:
   with self.subTest(target=target,invalid=invalid):
    h=Harness();h.c.target_id=target
    obs=dataclasses.replace(h.obs(),target_id=invalid)
    self.assertFalse(h.c._observation(obs,0))
    valid=dataclasses.replace(obs,target_id=target)
    self.assertTrue(h.c._observation(valid,0))
    h.c.phase='close'
    h.c.clearance=lambda *args:Clearance(True,0,'close',True,.03,invalid,('left_pad','right_pad'))
    self.assertFalse(h.c._clear(h.q,h.q,.03,valid,0))
    h.c.clearance=lambda *args:Clearance(True,0,'close',True,.03,target,('left_pad','right_pad'))
    self.assertTrue(h.c._clear(h.q,h.q,.03,valid,0))
 def test_stage_evidence_only_follows_completed_gates(self):
  h=Harness();initial=h.c.command()['stage_evidence']
  self.assertFalse(any(v for v in initial.values()))
  while h.c.phase!='close':h.step()
  e=h.c.command()['stage_evidence']
  self.assertTrue(e['wrist_identity']);self.assertTrue(e['collision_checked_approach'])
  self.assertFalse(e['grasp']);self.assertFalse(e['lift_and_hold'])
  while h.c.phase!='release':h.step()
  e=h.c.command()['stage_evidence'];self.assertTrue(e['placement_pose'])
  self.assertTrue(e['lift_and_hold']);self.assertFalse(e['placement_and_release'])
  result=h.run();e=result['stage_evidence']
  self.assertTrue(e['placement_and_release']);self.assertTrue(e['arm_return'])
  # A stationary cycle provides no evidence of navigation/base transport.
  self.assertIsNone(e['base_transport']);self.assertNotIn('mission_success',result)
 def test_stage_evidence_snapshot_cannot_mutate_cycle(self):
  h=Harness();e=h.c.command()['stage_evidence'];e['grasp']=True
  self.assertFalse(h.c.command()['stage_evidence']['grasp'])
 def test_caller_plan_mutation_cannot_retarget_or_widen_limits(self):
  h=Harness();lo=np.full(5,-1.);hi=np.ones(5)
  wp={k:v.copy() for k,v in h.c.waypoints.items()};placement=np.array([.3,0,.5])
  h.c=PickPlaceCycle(3,wp,lo,hi,h.clearance,placement_base_m=placement)
  # Reusing caller buffers after validation previously silently changed the plan.
  for q in wp.values():q[:]=100.
  lo[:]=-100.;hi[:]=100.;placement[:]=100.
  r=h.run()
  self.assertTrue(r['cycle_success'])
  self.assertTrue(all(np.max(np.abs(x['arm_target']))<=.01 for x in h.traces if x['arm_target'] is not None))
  h=Harness();hi=np.ones(5)
  h.c=PickPlaceCycle(3,h.c.waypoints,np.full(5,-1.),hi,h.clearance,placement_base_m=[.3,0,.5])
  hi[:]=100.;h.q[:]=2.
  self.assertEqual(h.step()['reason'],'invalid_or_moving_feedback')
 def test_synthetic_sensor_trace_complete(self):
  h=Harness();r=h.run();self.assertEqual(r['phase'],'complete');self.assertTrue(r['grasp_verified']);self.assertTrue(r['release_verified']);self.assertTrue(r['cycle_success']);self.assertNotIn('mission_success',r)
 def test_whole_trace_never_claims_full_mission(self):
  h=Harness();h.run();self.assertTrue(all('mission_success' not in r for r in h.traces));self.assertTrue(h.traces[-1]['cycle_success']);self.assertFalse(any(r['cycle_success'] for r in h.traces[:-1]))
 def test_lost_object_after_verified_lift_does_not_complete(self):
  h=Harness()
  while h.c.phase!='place':h.step()
  self.assertTrue(h.c.retained)
  # Prior lift is historical evidence; a later loss must terminate this cycle.
  o=dataclasses.replace(h.obs(),object_base_m=(.4,0,.54))
  r=h.step(o)
  self.assertEqual(r['reason'],'object_slip');self.assertFalse(r['cycle_success'])
  self.assertTrue(r['grasp_verified']);self.assertFalse(r['release_verified'])
  self.assertEqual(r['gripper_hold_mode'],'retain_existing_clamp')
  self.assertTrue(r['cancel_previous_arm_target'])
  self.assertEqual(h.step()['phase'],'failed')
 def test_refold_failure_does_not_promote_release_to_cycle_success(self):
  h=Harness()
  while h.c.phase!='refold':h.step()
  self.assertTrue(h.c.released)
  h.mode='unknown';r=h.step()
  self.assertEqual(r['reason'],'live_clearance_unconfirmed')
  self.assertTrue(r['grasp_verified']);self.assertTrue(r['release_verified'])
  self.assertFalse(r['cycle_success']);self.assertIsNone(r['arm_target'])
  self.assertTrue(r['cancel_previous_arm_target'])
  self.assertEqual(h.step()['phase'],'failed')
 def test_close_only_never_grasp(self):
  h=Harness()
  while h.c.phase!='lift':h.step()
  self.assertFalse(h.c.retained);self.assertFalse(h.c.command()['cycle_success'])
 def test_empty_close_rejected(self):
  r=Harness('empty').run();self.assertEqual(r['reason'],'empty_or_missing_grasp_contact');self.assertFalse(r['grasp_verified'])
 def test_no_object_lift_rejected(self):self.assertEqual(Harness('no_lift').run()['reason'],'object_slip')
 def test_stuck_release_rejected(self):self.assertEqual(Harness('stuck').run()['reason'],'release_separation_not_observed')
 def test_unknown_volume_blocks_motion(self):
  h=Harness('unknown');r=h.run();self.assertEqual(r['reason'],'live_clearance_unconfirmed');self.assertIsNone(r['arm_target']);self.assertTrue(r['cancel_previous_arm_target'])
 def test_stale_wrist_blocks_start(self):
  h=Harness();h.t=1.;r=h.step(dataclasses.replace(h.obs(),capture_ns=0,encoder_capture_ns=0));self.assertEqual(r['phase'],'observe')
 def test_top_camera_cannot_confirm(self):
  h=Harness()
  for _ in range(20):h.step(dataclasses.replace(h.obs(),camera='r07_camera1_left_optical'))
  self.assertEqual(h.c.phase,'observe')
 def test_truth_source_cannot_confirm(self):
  h=Harness()
  for _ in range(20):h.step(dataclasses.replace(h.obs(),source='physx_ground_truth'))
  self.assertEqual(h.c.phase,'observe')
 def test_duplicate_frame_not_three(self):
  h=Harness();o=h.obs()
  for _ in range(11):h.step(o)
  self.assertEqual(h.c.phase,'observe');self.assertEqual(len(h.c.identity_frames),1)
 def test_same_capture_cannot_change_sensor_evidence(self):
  for field,value in [('object_base_m',(.31,0,.5)),('tcp_base_m',(.31,0,.5)),
                      ('encoder_capture_ns',1),('uncertainty_m',.002),('alignment_error_rad',.01)]:
   with self.subTest(field=field):
    h=Harness();o=h.obs();h.step(o)
    r=h.step(dataclasses.replace(o,**{field:value}))
    self.assertEqual(r['reason'],'frame_observation_changed')
    self.assertFalse(r['cycle_success']);self.assertFalse(r['grasp_verified'])
    self.assertTrue(r['cancel_previous_arm_target'])
 def test_caller_mutation_cannot_change_saved_capture(self):
  h=Harness();obj=np.array([.3,0,.5]);o=dataclasses.replace(h.obs(),object_base_m=obj)
  h.step(o);obj[0]+=.001
  self.assertEqual(h.step(o)['reason'],'frame_observation_changed')
 def test_changed_timestamp_same_frame_rejected(self):
  h=Harness();h.step();self.assertEqual(h.step(dataclasses.replace(h.obs(),frame=0))['reason'],'frame_timestamp_changed')
 def test_stale_after_approach_holds_not_opens(self):
  h=Harness()
  while h.c.phase!='lift':h.step()
  width=h.c.width;r=h.step(dataclasses.replace(h.obs(),capture_ns=0,encoder_capture_ns=0));self.assertTrue(r['hold']);self.assertEqual(r['gripper_width_m'],width)
 def test_feedback_stale_rejected(self):
  h=Harness();r=h.c.step(1.,[0]*5,[0]*5,.075,0.,feedback_capture_ns=0,base_stopped=True,observation=h.obs());self.assertEqual(r['reason'],'invalid_or_moving_feedback')
 def test_settling_threshold_not_relaxed(self):
  h=Harness()
  for _ in range(20):
   h.c.step(h.t,[0]*5,[.02,0,0,0,0],.075,0,feedback_capture_ns=round(h.t*1e9),base_stopped=True,observation=h.obs());h.t+=.05;h.frame+=1
  self.assertEqual(h.c.phase,'observe')
 def test_target_must_be_at_tcp(self):
  h=Harness();h.tcp[0]=.35;self.assertEqual(h.run()['reason'],'target_not_at_grasp_tcp')
 def test_hold_aperture_loss_cannot_confirm_grasp(self):
  h=Harness()
  while h.c.phase!='hold':h.step()
  r=h.c.step(h.t,h.q,[0]*5,.075,0,feedback_capture_ns=round(h.t*1e9),base_stopped=True,observation=h.obs())
  self.assertEqual(r['reason'],'grasp_aperture_lost');self.assertFalse(r['grasp_verified'])
 def test_verification_bursts_reset_when_unsettled(self):
  for phase,field in [('hold','hold_samples'),('verify_release','release_samples')]:
   for moving_arm in [True,False]:
    h=Harness()
    while h.c.phase!=phase:h.step()
    h.step();self.assertEqual(len(getattr(h.c,field)),1)
    r=h.c.step(h.t,h.q,[.02,0,0,0,0] if moving_arm else [0]*5,h.aperture,0 if moving_arm else .002,feedback_capture_ns=round(h.t*1e9),base_stopped=True,observation=h.obs())
    self.assertEqual(r['phase'],phase);self.assertEqual(getattr(h.c,field),[])
 def test_moving_captures_do_not_count_toward_settled_identity(self):
  h=Harness()
  for _ in range(3):
   h.c.step(h.t,[0]*5,[.02,0,0,0,0],.075,0,feedback_capture_ns=round(h.t*1e9),base_stopped=True,observation=h.obs());h.t+=.05;h.frame+=1
  self.assertEqual(h.c.identity_frames,set())
  h.step();self.assertEqual(len(h.c.identity_frames),1)
 def test_settling_interruption_discards_previous_identity_burst(self):
  for qd,aperture in [([.02,0,0,0,0],.075),([0]*5,.073)]:
   h=Harness();h.step();h.step()
   self.assertEqual(len(h.c.identity_frames),2)
   h.c.step(h.t,[0]*5,qd,aperture,0,feedback_capture_ns=round(h.t*1e9),base_stopped=True,observation=h.obs())
   self.assertEqual(h.c.identity_frames,set());self.assertIsNone(h.c.stable)
 def test_slip_after_retention_rejected(self):
  h=Harness()
  while h.c.phase!='place':h.step()
  o=dataclasses.replace(h.obs(),object_base_m=(.4,0,.54));self.assertEqual(h.step(o)['reason'],'object_slip')
 def test_wrong_id_or_excess_uncertainty_blocks(self):
  for kwargs in [dict(target_id=4),dict(uncertainty_m=.006),dict(alignment_error_rad=.11),dict(encoder_capture_ns=-2_000_000)]:
   with self.subTest(kwargs=kwargs):
    h=Harness();self.assertEqual(h.step(dataclasses.replace(h.obs(),**kwargs))['phase'],'observe')
 def test_nonfinite_clock_fails_closed(self):
  h=Harness();r=h.c.step(float('nan'),[0]*5,[0]*5,.075,0,feedback_capture_ns=0,base_stopped=True,observation=h.obs());self.assertEqual(r['reason'],'invalid_clock')
class TestClockSplit(unittest.TestCase):
 def test_slow_rtf_dwell_uses_sim_clock(self):
  h=Harness()
  for i in range(8):
   wall=100_000_000_000+i*150_000_000;o=dataclasses.replace(h.obs(),frame=i,capture_ns=wall,encoder_capture_ns=wall)
   h.c.step(i*.05,[0]*5,[0]*5,.075,0.,feedback_capture_ns=wall,base_stopped=True,observation=o,now_monotonic_ns=wall+10_000_000)
  self.assertEqual(h.c.phase,'observe') # wall >1s, simulation .35s: not settled.
 def test_wall_stale_rejected_despite_recent_sim_time(self):
  h=Harness();wall=100_000_000_000;o=dataclasses.replace(h.obs(),capture_ns=wall,encoder_capture_ns=wall)
  r=h.c.step(0.,[0]*5,[0]*5,.075,0.,feedback_capture_ns=wall+600_000_000,base_stopped=True,observation=o,now_monotonic_ns=wall+600_000_000)
  self.assertEqual(r['phase'],'observe');self.assertEqual(r['reason'],'await_fresh_wrist')
 def test_explicit_invalid_monotonic_rejects(self):
  h=Harness();r=h.c.step(0.,[0]*5,[0]*5,.075,0.,feedback_capture_ns=0,base_stopped=True,observation=h.obs(),now_monotonic_ns=float('nan'));self.assertEqual(r['reason'],'invalid_monotonic_clock')
if __name__=='__main__':unittest.main(verbosity=2)

class TestCommandBoundaryFreshness(unittest.TestCase):
 def exercise(self,obs_age,feedback_age,elapsed,expected):
  from unittest.mock import patch
  h=Harness()
  while h.c.phase!='lift':h.step()
  before=h.c.reference.copy();width=h.c.width;wall=10**15
  o=dataclasses.replace(h.obs(),capture_ns=wall-obs_age,encoder_capture_ns=wall-obs_age)
  # Keep a valid new frame in the runtime clock epoch for this isolated boundary.
  h.c.last_capture_ns=o.capture_ns-1
  with patch('arm_practice.pick_place_cycle.time.monotonic_ns',return_value=wall+elapsed):
   r=h.c.step(h.t,h.q,[0]*5,h.aperture,0.,feedback_capture_ns=wall-feedback_age,base_stopped=True,observation=o,now_monotonic_ns=wall)
  self.assertEqual(r['reason'],expected)
  if expected:
   self.assertEqual(r['phase'],'failed');self.assertIsNone(r['arm_target'])
   self.assertTrue(r['cancel_previous_arm_target']);self.assertEqual(r['gripper_width_m'],width)
   np.testing.assert_array_equal(h.c.reference,before)
 def test_observation_expires_during_clearance(self):self.exercise(490_000_000,0,20_000_000,'observation_expired_during_clearance')
 def test_feedback_expires_during_clearance(self):self.exercise(0,90_000_000,20_000_000,'feedback_expired_during_clearance')
 def test_exact_deadline_is_allowed(self):self.exercise(490_000_000,90_000_000,10_000_000,None)

 def test_release_requires_full_two_seconds(self):
  h=Harness()
  while h.c.phase!='verify_release':h.step()
  start=h.t
  while h.t-start<1.95:
   h.step();self.assertFalse(h.c.released)
  while h.c.phase=='verify_release':h.step()
  self.assertTrue(h.c.released)
  self.assertGreaterEqual(h.c.release_samples[-1][0]-h.c.release_samples[0][0],2.)
 def test_release_rejects_twelve_mm_destination_error(self):
  h=Harness()
  while h.c.phase!='verify_release':h.step()
  h.obj[1]+=.012
  h.c.release_anchor=h.obj.copy()  # Stable release, but outside mission destination tolerance.
  r=h.step()
  self.assertEqual(r['reason'],'release_separation_not_observed')
  self.assertFalse(r['release_verified'])

 def test_release_stale_frame_cannot_finish_two_second_window(self):
  h=Harness()
  while h.c.phase!='verify_release':h.step()
  frozen=h.obs()
  for _ in range(12):
   r=h.step(frozen)
   if r['phase']=='failed':break
  self.assertEqual(r['reason'],'wrist_observation_missing_or_stale')
  self.assertFalse(r['release_verified'])
 def test_release_motion_restarts_full_two_second_window(self):
  h=Harness()
  while h.c.phase!='verify_release':h.step()
  for _ in range(30):h.step()
  self.assertFalse(h.c.released)
  h.c.step(h.t,h.q,[.02,0,0,0,0],h.aperture,0.,feedback_capture_ns=round(h.t*1e9),base_stopped=True,observation=h.obs())
  h.t+=.05;h.frame+=1
  self.assertEqual(h.c.release_samples,[])
  for _ in range(30):h.step()
  self.assertFalse(h.c.released)
  while h.c.phase=='verify_release':h.step()
  self.assertGreaterEqual(h.c.release_samples[-1][0]-h.c.release_samples[0][0],2.)
