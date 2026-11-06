"""Inactive encoder-feedback LQR drive candidate; requires validated plant gains.

The 4x transmission, steering 1:1, continuous efforts and back-EMF envelope
match the existing simulation actuator contract. This module cannot authorize
motion or bypass collision, tilt, stale-sensor or arm-stow interlocks.
"""
import numpy as np
from .discrete_lqr import DiscreteLQR


def _four(x):
    a=np.asarray(x,dtype=float)
    if a.shape!=(4,) or not np.isfinite(a).all():raise ValueError('four finite module values required')
    return a


def dc_envelope(velocity, *, steering):
    velocity=_four(velocity)
    no_load, stall, continuous=(7.330382858,2.7,.54) if steering else (29.321531434,.54,.108)
    # A separate plant-side clamp must still use physical shaft velocity.
    upper=np.minimum(continuous,stall*(1-velocity/no_load))
    lower=np.maximum(-continuous,stall*(-1-velocity/no_load))
    if np.any(lower>upper):raise ValueError('velocity outside valid motor envelope')
    return lower,upper


class EncoderDriveLQR:
    def __init__(self, wheel, steering):
        # Each module needs its own validated model; never copy one fitted axis
        # across four distinct contact/load conditions implicitly.
        if not isinstance(wheel, (tuple,list)) or not isinstance(steering, (tuple,list)) or len(wheel)!=4 or len(steering)!=4:
            raise ValueError('four explicit wheel and four steering models required')
        for w,s in zip(wheel,steering):
            if not isinstance(w,DiscreteLQR) or not isinstance(s,DiscreteLQR):raise ValueError('model-derived LQR required')
            if w.gain.shape!=(1,1) or s.gain.shape!=(1,2):raise ValueError('wheel speed and steering angle/rate models required')
            if abs(w.sample_period_s-1/120)>1e-12 or abs(s.sample_period_s-1/120)>1e-12:raise ValueError('120Hz encoder model required')
        self.wheel=tuple(wheel);self.steering=tuple(steering)

    def efforts(self, *, wheel_target, wheel_velocity, steer_target, steer_position,
                steer_velocity, steer_target_velocity, wheel_equilibrium_effort, steer_equilibrium_effort,
                observed_ns, now_ns, safety_permitted):
        if type(safety_permitted) is not bool or not safety_permitted:raise ValueError('independent safety permit required')
        # One feedback update per 120Hz control interval, no stale sample reuse.
        if type(observed_ns) is not int or type(now_ns) is not int or not 0<=observed_ns<=now_ns or now_ns-observed_ns>8_333_334:raise ValueError('invalid or stale encoder sample')
        wt,wv,st,sp,sv,stv,we,se=map(_four,(wheel_target,wheel_velocity,steer_target,steer_position,steer_velocity,steer_target_velocity,wheel_equilibrium_effort,steer_equilibrium_effort))
        if np.any(abs(wt)>29.321531434) or np.any(abs(st)>np.pi/2) or np.any(abs(sp)>np.pi/2) or np.any(abs(stv)>7.330382858):raise ValueError('reference or steering joint limit')
        wl,wu=dc_envelope(wv,steering=False);sl,su=dc_envelope(sv,steering=True)
        w=np.empty(4);s=np.empty(4)
        for i in range(4):
            w[i]=self.wheel[i].command([wv[i]-wt[i]],equilibrium_input=[we[i]],lower=[wl[i]],upper=[wu[i]])[0]
            s[i]=self.steering[i].command([sp[i]-st[i],sv[i]-stv[i]],equilibrium_input=[se[i]],lower=[sl[i]],upper=[su[i]])[0]
        return {'wheel_effort_Nm':w,'steer_effort_Nm':s,'motion_authorized':False,'scope':'candidate effort calculation only'}
