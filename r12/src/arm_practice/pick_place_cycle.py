"""Simulation-only sensor-gated pick/lift/place coordinator candidate.

No planner, detector, collision checker, actuator, simulator pose access, or
hardware adapter is hidden here. This is one stationary object cycle;
base transport and multi-object mission aggregation are not implemented. Caller-supplied sensor-derived joint waypoints
and live swept-volume certification remain mandatory. Closing is never grasp
success. A failed gate freezes the last command; the adapter must hold safely.
"""
from dataclasses import dataclass
import math
import time
import numpy as np

@dataclass(frozen=True)
class Observation:
    run_id: str
    frame: int
    capture_ns: int
    encoder_capture_ns: int
    target_id: int
    camera: str
    source: str
    object_base_m: tuple
    tcp_base_m: tuple
    depth_valid: bool
    identity_verified: bool
    uncertainty_m: float
    alignment_error_rad: float

@dataclass(frozen=True)
class Clearance:
    allowed: bool
    capture_ns: int
    phase: str
    unknown_volume_clear: bool
    cad_margin_m: float
    contact_target_id: int | None = None
    contact_bodies: tuple = ()

class PickPlaceCycle:
    PHASES=('observe','approach','close','lift','hold','place','release','retreat','verify_release','refold','complete')
    CONTACT_PHASES=('close','lift','hold','place','release')
    def __init__(self, target_id, waypoints, lower, upper, clearance, *, placement_base_m,
                 simulation_only=True, close_width=.027, object_width=.030, timeout=90., arm_reference_rate=.005):
        if simulation_only is not True:raise ValueError('simulation_only')
        if type(target_id) is not int or not 0<=target_id<12:raise ValueError('target_id')
        # Own the validated plan: later edits to caller arrays must not change
        # joint bounds, a collision proposal, or the release destination.
        self.lo,self.hi=np.array(lower,dtype=float,copy=True),np.array(upper,dtype=float,copy=True)
        self.waypoints={k:np.array(v,dtype=float,copy=True) for k,v in waypoints.items()}
        if set(self.waypoints)!={'approach','lift','place','retreat','refold'}:raise ValueError('waypoint_keys')
        if self.lo.shape!=(5,) or self.hi.shape!=(5,) or not np.isfinite(np.r_[self.lo,self.hi]).all() or np.any(self.lo>self.hi):raise ValueError('joint_limits')
        for q in self.waypoints.values():
            if q.shape!=(5,) or not np.isfinite(q).all() or np.any(q<self.lo) or np.any(q>self.hi):raise ValueError('invalid_waypoint')
        self.placement=np.array(placement_base_m,dtype=float,copy=True)
        if self.placement.shape!=(3,) or not np.isfinite(self.placement).all():raise ValueError('placement_sensor_pose')
        if not callable(clearance) or not .020<=close_width<object_width<=.075 or not 0<timeout<=180:raise ValueError('configuration')
        if isinstance(arm_reference_rate,bool) or not math.isfinite(arm_reference_rate) or not 0<arm_reference_rate<=.005:raise ValueError('arm_reference_rate')
        self.arm_reference_rate=float(arm_reference_rate)
        self.target_id=target_id;self.clearance=clearance;self.close_width=close_width;self.object_width=object_width;self.timeout=timeout
        self.failed_phase=None;self.phase='observe';self.reason='await_fresh_wrist';self.last=self.started=self.stable=None;self.reference=None;self.width=.075
        self.run_id=None;self.last_frame=-1;self.last_capture_ns=-1;self.last_observation_signature=None;self.identity_frames=set();self.initial_object=None;self.relative=None;self.hold_samples=[];self.release_samples=[];self.retained=False;self.released=False;self.audit=[]
        self.identity_verified=False;self.approach_verified=False;self.placement_pose_verified=False
    def reference_travel_lower_bound(self, initial_q):
        """Necessary travel time only; excludes settling, sensing and gripper dwell."""
        previous=np.asarray(initial_q,dtype=float)
        if previous.shape!=(5,) or not np.isfinite(previous).all():
            raise ValueError('initial_q')
        duration=0.
        for phase in ('approach','lift','place','retreat','refold'):
            endpoint=self.waypoints[phase]
            duration+=float(np.max(np.abs(endpoint-previous)))/self.arm_reference_rate
            previous=endpoint
        return duration
    def _fault(self,reason):self.failed_phase=self.phase;self.phase='failed';self.reason=reason;return self.command()
    def command(self):
        # Stage evidence is historical for this cycle, not current object safety
        # or full-mission success. None denotes an unimplemented transport stage.
        failed=self.phase=='failed'
        retain_clamp=failed and self.failed_phase in ('lift','hold','place') and self.relative is not None
        return dict(phase=self.phase,reason=self.reason,failed_phase=self.failed_phase,
                    arm_target=None if failed or self.reference is None else self.reference.tolist(),
                    gripper_width_m=(self.width if retain_clamp or not failed else None),
                    gripper_hold_mode='retain_existing_clamp' if retain_clamp else 'hold_measured_aperture' if failed else 'track',
                    cancel_previous_arm_target=failed,requires_fresh_local_hold=failed,
                    base_command=[0.,0.,0.],hold=failed,grasp_verified=self.retained,release_verified=self.released,cycle_success=self.phase=='complete',
                    stage_evidence=dict(wrist_identity=self.identity_verified,
                                        collision_checked_approach=self.approach_verified,
                                        grasp=self.retained,lift_and_hold=self.retained,
                                        base_transport=None,placement_pose=self.placement_pose_verified,
                                        placement_and_release=self.released,
                                        arm_return=self.phase=='complete'))
    def _next(self,phase,now):self.phase=phase;self.stable=None;self.reason=None;self.audit.append(dict(phase=phase,time=now))
    def _observation(self,obs,now_ns):
        if not isinstance(obs,Observation):return False
        if (type(obs.frame) is not int or obs.frame<0 or type(obs.capture_ns) is not int or type(obs.encoder_capture_ns) is not int or
            obs.camera!='arm_camera_left_optical' or obs.source!='wrist_rgbd_encoder_fk' or type(obs.target_id) is not int or obs.target_id!=self.target_id or
            obs.depth_valid is not True or obs.identity_verified is not True or not isinstance(obs.run_id,str) or not obs.run_id or
            not 0<=now_ns-obs.capture_ns<=500_000_000 or abs(obs.capture_ns-obs.encoder_capture_ns)>1_000_000 or
            not math.isfinite(obs.uncertainty_m) or not 0<=obs.uncertainty_m<=.005 or
            not math.isfinite(obs.alignment_error_rad) or abs(obs.alignment_error_rad)>.10):return False
        p,t=np.asarray(obs.object_base_m,float),np.asarray(obs.tcp_base_m,float)
        return p.shape==(3,) and t.shape==(3,) and bool(np.isfinite(np.r_[p,t]).all())
    def _clear(self,q,proposal,width,obs,now_ns):
        try:c=self.clearance(self.phase,q.copy(),proposal.copy(),width,obs)
        except Exception:return False
        if not isinstance(c,Clearance) or c.allowed is not True or c.unknown_volume_clear is not True or c.phase!=self.phase or c.capture_ns!=obs.capture_ns or not 0<=now_ns-c.capture_ns<=500_000_000:return False
        if not math.isfinite(c.cad_margin_m) or c.cad_margin_m<.03:return False
        if c.contact_target_id is None:return self.phase not in self.CONTACT_PHASES and c.contact_bodies==()
        return self.phase in self.CONTACT_PHASES and type(c.contact_target_id) is int and c.contact_target_id==self.target_id and c.contact_bodies==('left_pad','right_pad')
    def step(self,now,q,qd,aperture,aperture_speed,*,feedback_capture_ns,base_stopped,observation,now_monotonic_ns=None):
        if self.phase in ('failed','complete'):return self.command()
        if not math.isfinite(now):return self._fault('invalid_clock')
        q,qd=np.asarray(q,float),np.asarray(qd,float)
        # now drives physical/simulation dwell and slew; capture age uses wall monotonic.
        # None is reserved for deterministic same-clock unit traces. Runtime adapters
        # must provide the real monotonic clock explicitly.
        now_ns=round(now*1e9) if now_monotonic_ns is None else now_monotonic_ns
        if type(now_ns) is not int or now_ns<0:return self._fault('invalid_monotonic_clock')
        if (not math.isfinite(now) or q.shape!=(5,) or qd.shape!=(5,) or not np.isfinite(np.r_[q,qd,aperture,aperture_speed]).all() or
            np.any(q<self.lo) or np.any(q>self.hi) or not .019<=aperture<=.076 or type(feedback_capture_ns) is not int or
            not 0<=now_ns-feedback_capture_ns<=100_000_000 or base_stopped is not True):return self._fault('invalid_or_moving_feedback')
        if self.started is None:
            self.started=now;self.reference=q.copy()
            if self.reference_travel_lower_bound(q)>self.timeout:
                return self._fault('plan_travel_exceeds_cycle_timeout')
        if now-self.started>self.timeout:return self._fault('cycle_timeout')
        dt=0. if self.last is None else now-self.last;self.last=now
        if dt<0 or dt>.100000001:return self._fault('clock_gap')
        if np.max(abs(q-self.reference))>.025:return self._fault('arm_tracking_error')
        if not self._observation(observation,now_ns):
            if self.phase=='observe':self.identity_frames.clear();self.stable=None;self.reason='await_fresh_wrist';return self.command()
            return self._fault('wrist_observation_missing_or_stale')
        obs=observation
        if self.run_id is None:self.run_id=obs.run_id
        if obs.run_id!=self.run_id:return self._fault('capture_run_changed')
        if obs.frame<self.last_frame:return self._fault('frame_regression')
        new_frame=obs.frame>self.last_frame
        if new_frame and obs.capture_ns<=self.last_capture_ns:return self._fault('capture_timestamp_not_increasing')
        if not new_frame and obs.capture_ns!=self.last_capture_ns:return self._fault('frame_timestamp_changed')
        # A frame denotes one immutable capture. Snapshot numeric values rather
        # than retaining caller-owned arrays that can be mutated after acceptance.
        signature=(obs.encoder_capture_ns, tuple(np.asarray(obs.object_base_m,float)),
                   tuple(np.asarray(obs.tcp_base_m,float)), float(obs.uncertainty_m),
                   float(obs.alignment_error_rad))
        if not new_frame and signature!=self.last_observation_signature:
            return self._fault('frame_observation_changed')
        self.last_observation_signature=signature
        self.last_frame=max(self.last_frame,obs.frame);self.last_capture_ns=obs.capture_ns
        position=np.asarray(obs.object_base_m);relative=position-np.asarray(obs.tcp_base_m)
        # Fresh measured aperture must support retention throughout motion,
        # even before a camera frame shows relative object slip. This is a
        # loss veto, never positive evidence that closing grasped the object.
        if self.relative is not None and self.phase in ('lift','hold','place'):
            if aperture<self.object_width-.002 or aperture>self.object_width+.003:
                return self._fault('grasp_aperture_lost')
        if self.phase=='observe':
            # A moving-arm capture cannot contribute to the settled view burst.
            # Restart the evidence sequence whenever settling is interrupted.
            if np.max(abs(qd))>=.02 or abs(aperture-.075)>.001:
                self.identity_frames.clear();self.stable=None;return self.command()
            if new_frame:self.identity_frames.add(obs.frame)
            if self.stable is None:self.stable=now
            if len(self.identity_frames)>=3 and now-self.stable>=.5:
                self.identity_verified=True
                self.initial_object=position.copy();self._next('approach',now)
            return self.command()
        if self.phase in ('approach','lift','place','retreat','refold'):
            endpoint=self.waypoints[self.phase];proposal=self.reference+np.clip(endpoint-self.reference,-self.arm_reference_rate*dt,self.arm_reference_rate*dt)
        else:proposal=self.reference.copy()
        desired=.075 if self.phase in ('release','retreat','verify_release','refold') else self.close_width if self.phase in self.CONTACT_PHASES else .075
        width=self.width+float(np.clip(desired-self.width,-.08*dt,.08*dt))
        if not self._clear(q,proposal,width,obs,now_ns):return self._fault('live_clearance_unconfirmed')
        # Clearance can be expensive: recheck at the command boundary before
        # committing a trajectory or gripper change. Deterministic same-clock
        # traces remain available only when no runtime wall clock was supplied.
        if now_monotonic_ns is not None:
            commit_ns=max(now_ns,time.monotonic_ns())
            if not 0<=commit_ns-obs.capture_ns<=500_000_000:
                return self._fault('observation_expired_during_clearance')
            if not 0<=commit_ns-feedback_capture_ns<=100_000_000:
                return self._fault('feedback_expired_during_clearance')
        self.reference=proposal;self.width=width
        if self.relative is not None and self.phase in ('lift','hold','place') and np.linalg.norm(relative-self.relative)>.010:return self._fault('object_slip')
        if self.phase in ('approach','lift','place','retreat','refold'):
            endpoint=self.waypoints[self.phase];settled=np.max(abs(q-endpoint))<.025 and np.max(abs(qd))<.02 and np.max(abs(self.reference-endpoint))<1e-9
            if not settled:self.stable=None;return self.command()
            if self.stable is None:self.stable=now
            if now-self.stable<.5:return self.command()
            if self.phase=='approach':
                if np.linalg.norm(relative)>.015:return self._fault('target_not_at_grasp_tcp')
                self.approach_verified=True
                self._next('close',now)
            elif self.phase=='lift':self.hold_samples=[];self._next('hold',now)
            elif self.phase=='place':
                if np.linalg.norm(position-self.placement)>.010:return self._fault('placement_pose_not_reached')
                self.placement_pose_verified=True
                self.release_anchor=position.copy();self._next('release',now)
            elif self.phase=='retreat':self.release_samples=[];self._next('verify_release',now)
            else:
                if not self.released:return self._fault('release_not_verified')
                self._next('complete',now)
        elif self.phase=='close':
            if abs(self.width-self.close_width)>1e-9:return self.command()
            if aperture<self.object_width-.002:return self._fault('empty_or_missing_grasp_contact')
            if aperture>self.object_width+.003 or abs(aperture_speed)>=.002:self.stable=None;return self.command()
            if self.stable is None:self.stable=now
            if now-self.stable>=.5:
                self.relative=relative.copy();self.initial_object=position.copy();self._next('lift',now)
        elif self.phase=='hold':
            if position[2]-self.initial_object[2]<.025 or np.linalg.norm(relative-self.relative)>.010:return self._fault('lift_retention_not_observed')
            if np.max(abs(qd))>=.02 or abs(aperture_speed)>=.002:
                self.hold_samples.clear();return self.command()
            if new_frame:self.hold_samples.append((now,obs.frame))
            if len(self.hold_samples)>=3 and self.hold_samples[-1][0]-self.hold_samples[0][0]>=.5:
                self.retained=True;self._next('place',now)
        elif self.phase=='release':
            if abs(self.width-.075)>1e-9 or abs(aperture-.075)>.001 or abs(aperture_speed)>=.002:self.stable=None;return self.command()
            if self.stable is None:self.stable=now
            if now-self.stable>=.5:self._next('retreat',now)
        elif self.phase=='verify_release':
            if np.linalg.norm(position-self.release_anchor)>.010 or np.linalg.norm(position-self.placement)>.010 or np.linalg.norm(relative-self.relative)<.030:return self._fault('release_separation_not_observed')
            if np.max(abs(qd))>=.02 or abs(aperture-.075)>.001 or abs(aperture_speed)>=.002:
                self.release_samples.clear();return self.command()
            if new_frame:self.release_samples.append((now,obs.frame))
            if len(self.release_samples)>=3 and self.release_samples[-1][0]-self.release_samples[0][0]>=2.:
                self.released=True;self._next('refold',now)
        return self.command()
