"""Offline mirror of moving, zero-residual V97/V109 reference generation.

Uses sensor steering/wheel velocity and previous references. Excludes stop-yaw
brake, emergency overrides and PI/plant dynamics; callers must handle these.
"""
import numpy as np
from .latest_policy import allocate
from .steering_transition import brake_before_large_turn


def moving_reference(command,steering,encoder_wheels,previous_steer,previous_wheels,dt=.05):
    arrays=[np.asarray(x,float) for x in (command,steering,encoder_wheels,previous_steer,previous_wheels)]
    if arrays[0].shape!=(3,) or any(x.shape!=(4,) for x in arrays[1:]):raise ValueError('Invalid state shape')
    if not all(np.isfinite(x).all() for x in arrays) or not np.isfinite(dt) or not 0<dt<=.1:raise ValueError('Invalid state/time')
    if np.linalg.norm(arrays[0])<1e-6:raise ValueError('Stationary stop controller is outside this model')
    command,steering,encoder_wheels,previous_steer,previous_wheels=arrays
    target,wheels=allocate(command,steering,previous_steer,np.zeros(4),dt=dt)
    target,wheels,braking=brake_before_large_turn(target,steering,wheels,encoder_wheels)
    wheels=previous_wheels+np.clip(wheels-previous_wheels,-24.45*dt,24.45*dt)
    return target,wheels,braking
