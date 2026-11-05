"""Brake a rolling module before a large, drive-gated steering transition."""
import numpy as np


def brake_before_large_turn(target, current, wheel_target, wheel_measured):
    target=np.asarray(target,float).copy()
    current=np.asarray(current,float)
    wheel_target=np.asarray(wheel_target,float).copy()
    wheel_measured=np.asarray(wheel_measured,float)
    # The allocator's cosine gate requests near-zero drive during reversal.
    # Keep that module's heading until encoder speed is low; wheel slew still
    # enforces the existing acceleration bound. Other modules remain untouched.
    braking=(np.abs(target-current)>.3)&(np.abs(wheel_measured)>2.)&(np.abs(wheel_target)<.2*np.abs(wheel_measured))
    target[braking]=current[braking]
    wheel_target[braking]=0.
    return target,wheel_target,braking


def project_current_heading(command, current):
    """Signed rolling velocity along encoder heading; no desired-angle sign jump."""
    from .swerve import OFFSETS,BASE_DIRECTIONS,RADIUS
    command=np.asarray(command,float);current=np.asarray(current,float)
    vectors=command[:2]+command[2]*np.column_stack([-OFFSETS[:,1],OFFSETS[:,0]])
    peak=np.max(np.linalg.norm(vectors,axis=1));vectors*=min(1.,.899795667/max(peak,1e-9))
    heading=BASE_DIRECTIONS+current
    return np.clip(np.sum(vectors*np.column_stack([np.cos(heading),np.sin(heading)]),axis=1)/RADIUS,-29.321531434,29.321531434)


def coordinate_turn_with_drive(command,current,target,wheel_measured,dt=.05,return_status=False):
    """Advance only through a contiguous feasible steering interval.

    If current rolling demand is unreachable, hold steering and slew wheel
    demand toward it. That fallback is explicitly not rolling-compatible yet.
    This experimental helper is not enabled in the simulator by default.
    """
    command=np.asarray(command,float);current=np.asarray(current,float)
    target=np.asarray(target,float);measured=np.asarray(wheel_measured,float)
    if command.shape!=(3,) or any(x.shape!=(4,) for x in (current,target,measured)):
        raise ValueError('Expected body twist and four module states')
    if not np.isfinite(np.r_[command,current,target,measured,dt]).all() or dt<=0:
        raise ValueError('Invalid coordination state/time')
    if np.max(abs(measured))>29.321531434:
        raise ValueError('Measured overspeed requires separate recovery')
    target=np.clip(target,-np.pi/2,np.pi/2)
    target=current+np.clip(target-current,-7.330382858*dt,7.330382858*dt)
    chosen=current.copy();budget=24.45*dt
    initial=project_current_heading(command,current)
    reachable=np.abs(initial-measured)<=budget
    active=reachable.copy()
    # Do not jump across an infeasible intermediate angle to a later island.
    for fraction in np.linspace(0.,1.,33)[1:]:
        candidate=current+fraction*(target-current)
        rolling=project_current_heading(command,candidate)
        active &= np.abs(rolling-measured)<=budget
        chosen=np.where(active,candidate,chosen)
    rolling=project_current_heading(command,chosen)
    wheels=measured+np.clip(rolling-measured,-budget,budget)
    status={'rolling_compatible':reachable.tolist(),
            'held_for_drive':(~reachable).tolist(),
            'projection_residual_rad_s':(wheels-rolling).tolist()}
    return (chosen,wheels,status) if return_status else (chosen,wheels)
