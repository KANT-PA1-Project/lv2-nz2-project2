"""Two rigidly mounted IMUs -> specific force at a declared body origin.
Vectors are SI/REP-145, rotations map sensor coordinates into body coordinates.
No gravity removal, no position integration, no implicit zero-offset fallback.
"""
from dataclasses import dataclass
import numpy as np


def skew(v):
    x,y,z=v
    return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])


def rotation(q):
    q=np.asarray(q,dtype=float)
    if q.shape!=(4,) or not np.isfinite(q).all() or np.linalg.norm(q)<1e-8:
        raise ValueError('Invalid XYZW mounting quaternion')
    x,y,z,w=q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


@dataclass
class Sample:
    time: float
    gyro: np.ndarray
    accel: np.ndarray
    gyro_cov: np.ndarray
    accel_cov: np.ndarray


def interpolate(samples, stamp, max_gap=.05):
    """Only interpolate bracketed data; never extrapolate a stale IMU."""
    for a in samples:
        if abs(a.time-stamp)<1e-9:return a
    for a,b in zip(samples,list(samples)[1:]):
        if a.time<stamp<b.time:
            if b.time-a.time>max_gap:raise ValueError('IMU interpolation gap')
            u=(stamp-a.time)/(b.time-a.time)
            # Conservative covariance: adjacent filtered IMU samples can correlate.
            return Sample(stamp,*[(1-u)*getattr(a,k)+u*getattr(b,k) for k in ('gyro','accel','gyro_cov','accel_cov')])
    raise ValueError('No synchronized IMU bracket')


def origin_force(force, omega, alpha, lever):
    return force-np.cross(alpha,lever)-np.cross(omega,np.cross(omega,lever))


class DualImuFusion:
    def __init__(self, positions, quaternions, gyro_bias=None, accel_bias=None,
                 alpha_tau=.03, max_gap=.1, gyro_difference=.35, accel_difference=2.):
        if alpha_tau<0 or min(max_gap,gyro_difference,accel_difference)<=0:raise ValueError('Invalid filter/gate settings')
        self.r=np.asarray(positions,dtype=float)
        if self.r.shape!=(2,3) or not np.isfinite(self.r).all():raise ValueError('Two finite lever arms required')
        self.rot=np.array([rotation(q) for q in quaternions])
        self.gb=np.zeros((2,3)) if gyro_bias is None else np.asarray(gyro_bias,dtype=float)
        self.ab=np.zeros((2,3)) if accel_bias is None else np.asarray(accel_bias,dtype=float)
        if self.gb.shape!=(2,3) or self.ab.shape!=(2,3) or not np.isfinite([self.gb,self.ab]).all():raise ValueError('Invalid bias calibration')
        self.tau=alpha_tau;self.max_gap=max_gap;self.gate_w=gyro_difference;self.gate_f=accel_difference
        self.previous=None;self.alpha=np.zeros(3);self.alpha_cov=np.zeros((3,3))

    def update(self, samples):
        if len(samples)!=2 or abs(samples[0].time-samples[1].time)>1e-8:raise ValueError('Synchronized pair required')
        t=samples[0].time
        if not np.isfinite(t):raise ValueError('Nonfinite stamp')
        for s in samples:
            if not all(np.isfinite(v).all() for v in [s.gyro,s.accel,s.gyro_cov,s.accel_cov]):raise ValueError('Nonfinite measurement')
        w=np.array([self.rot[i]@(s.gyro-self.gb[i]) for i,s in enumerate(samples)])
        f=np.array([self.rot[i]@(s.accel-self.ab[i]) for i,s in enumerate(samples)])
        cw=np.array([self.rot[i]@s.gyro_cov@self.rot[i].T for i,s in enumerate(samples)])
        cf=np.array([self.rot[i]@s.accel_cov@self.rot[i].T for i,s in enumerate(samples)])
        if np.linalg.norm(w[0]-w[1])>self.gate_w:
            self.previous=None;raise ValueError('Gyro disagreement: cannot identify faulty sensor from two alone')
        omega=w.mean(0);omega_cov=cw.sum(0)/4
        if self.previous is None:
            self.previous=(t,omega.copy(),omega_cov.copy());return None
        pt,pw,pc=self.previous;dt=t-pt
        if dt<=0:raise ValueError('Nonmonotonic synchronized stamp')
        if dt>self.max_gap:
            self.previous=(t,omega.copy(),omega_cov.copy());self.alpha[:]=0;self.alpha_cov[:]=0;return None
        mix=dt/(self.tau+dt);raw=(omega-pw)/dt
        # Temporal correlation unknown: conservative derivative covariance bound.
        derivative_cov=2*(omega_cov+pc)/(dt*dt)
        self.alpha=(1-mix)*self.alpha+mix*raw
        self.alpha_cov=(1-mix)*self.alpha_cov+mix*derivative_cov
        self.previous=(t,omega.copy(),omega_cov.copy())
        corrected=np.array([origin_force(f[i],omega,self.alpha,self.r[i]) for i in range(2)])
        residual=corrected[0]-corrected[1]
        if np.linalg.norm(residual)>self.gate_f:raise ValueError('Corrected acceleration disagreement')
        rbar=self.r.mean(0);ja=skew(rbar)
        # d[-omega x (omega x r)] / d omega
        jw=skew(np.cross(omega,rbar))+skew(omega)@skew(rbar)
        covariance=cf.sum(0)/4+ja@self.alpha_cov@ja.T+jw@omega_cov@jw.T
        # Shared correction and systematic disagreement do not vanish by averaging.
        covariance+=np.outer(residual,residual)/4
        individual_cov=[]
        for i,r in enumerate(self.r):
            jai=skew(r);jwi=skew(np.cross(omega,r))+skew(omega)@skew(r)
            individual_cov.append(cf[i]+jai@self.alpha_cov@jai.T+jwi@omega_cov@jwi.T)
        return dict(time=t,gyro=omega,accel=corrected.mean(0),gyro_cov=omega_cov,
                    accel_cov=covariance,alpha=self.alpha.copy(),corrected=corrected,corrected_gyro=w,
                    rotated_gyro_cov=cw,corrected_accel_cov=np.array(individual_cov),
                    gyro_difference=float(np.linalg.norm(w[0]-w[1])),
                    accel_difference=float(np.linalg.norm(residual)))
