"""Check same-home IK path preconditions; does not authorize execution."""
import numpy as np

def check_start_state(home, positions, velocities, *, now, state_stamp,
                      base_twist, lower, upper, timeout_s=.25,
                      home_tolerance_rad=.01, stationary_rad_s=.02):
    arrays=[np.asarray(x,dtype=float) for x in (home,positions,velocities,lower,upper)]
    h,q,v,lo,hi=arrays;b=np.asarray(base_twist,dtype=float)
    def reject(reason):return {'eligible_for_path_validation':False,'reason':reason,'execution_authorized':False}
    if any(a.shape!=(5,) or not np.isfinite(a).all() for a in arrays) or b.shape!=(3,) or not np.isfinite(b).all():return reject('invalid_feedback')
    if not np.isfinite([now,state_stamp]).all() or not 0<=now-state_stamp<=timeout_s:return reject('stale_or_future_feedback')
    if np.any(q<lo) or np.any(q>hi):return reject('joint_limit_violation')
    if np.any(np.abs(v)>=stationary_rad_s):return reject('arm_moving')
    if np.linalg.norm(b[:2])>=.01 or abs(b[2])>=.02:return reject('base_moving')
    error=float(np.max(np.abs(q-h)))
    if error>home_tolerance_rad:
        return {**reject('requires_separate_return_to_home_plan'),'max_home_error_rad':error}
    return {'eligible_for_path_validation':True,'reason':'home_stationary_fresh','max_home_error_rad':error,'execution_authorized':False}
