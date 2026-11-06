"""Stationary implicit80/8 reference reparameterization, simulation candidate.
Uses applied-reference and quantized encoder history only. No native force,
gravity, object truth, camera timing, or queued target enters this calculation.
"""
import numpy as np

def stationary_support_hold(*, encoder_window, applied_reference_window, sim_times,
                            read_capture_monotonic_ns, now_monotonic_ns,
                            kp, kd, previous_explicit_effort, solver_type, total_limit_nm=2.):
 if solver_type!='PGS':raise ValueError('unqualified_solver_mode')
 q=np.asarray(encoder_window,float);ref=np.asarray(applied_reference_window,float);t=np.asarray(sim_times,float)
 kp=np.asarray(kp,float);kd=np.asarray(kd,float);previous=np.asarray(previous_explicit_effort,float)
 if q.ndim!=2 or q.shape[1]!=5 or len(q)<11 or ref.shape!=q.shape or t.shape!=(len(q),) or not np.isfinite(q).all() or not np.isfinite(ref).all() or not np.isfinite(t).all():raise ValueError('invalid_encoder_reference_history')
 if kp.shape!=(5,) or kd.shape!=(5,) or not np.array_equal(kp,np.full(5,80.)) or not np.array_equal(kd,np.full(5,8.)):raise ValueError('unqualified_drive_gains')
 if previous.shape!=(5,) or not np.array_equal(previous,np.zeros(5)):raise ValueError('previous_explicit_effort_unsupported')
 if type(total_limit_nm) not in (float,int) or total_limit_nm!=2.:raise ValueError('qualified_total_limit_is_2Nm')
 if type(read_capture_monotonic_ns) is not int or type(now_monotonic_ns) is not int or not 0<=now_monotonic_ns-read_capture_monotonic_ns<=100000000:raise ValueError('stale_local_encoder')
 dt=np.diff(t)
 if not np.all(dt>0) or t[-1]-t[0]<.5 or np.max(dt)>.050001:raise ValueError('insufficient_continuous_settle_window')
 if not np.array_equal(ref,np.repeat(ref[-1][None,:],len(q),axis=0)):raise ValueError('applied_reference_not_static')
 if np.max(abs(np.diff(np.unwrap(q,axis=0),axis=0)/dt[:,None]))>=.02:raise ValueError('encoder_not_settled')
 if np.max(abs(ref-q))>.025:raise ValueError('reference_error_unbounded')
 held=q[-1].copy();bias=kp*(ref[-1]-held)
 # At most half the total budget is feedforward: valid if SDK maxForce
 # bounds only implicit drive or also sums user effort. Never exceed2Nm.
 if np.max(abs(bias))>total_limit_nm/2:raise ValueError('support_exceeds_qualified_half_budget')
 # Model the exact float32 effort submitted to the actuator, then round
 # the remaining implicit budget down so conversion cannot exceed 2 Nm.
 bias=bias.astype(np.float32).astype(np.float64)
 budget=total_limit_nm-np.abs(bias)
 cap32=budget.astype(np.float32)
 rounded_up=cap32.astype(np.float64)>budget
 cap32[rounded_up]=np.nextafter(cap32[rounded_up],np.float32(-np.inf))
 drive_cap=cap32.astype(np.float64)
 if np.any(np.abs(bias)+drive_cap>total_limit_nm):raise ValueError('effort_budget_rounding_failure')
 return dict(hold_measured_target=held.tolist(),feedforward_nm=bias.tolist(),implicit_max_effort_nm=drive_cap.tolist(),total_bound_nm=(np.abs(bias)+drive_cap).tolist(),source='static_applied_reference_and_quantized_encoder',qualified_solver='PGS',runtime_authorized=False,requires_zero_queued_motion=True,hardware_authorized=False,moving_stop_qualified=False)
