"""Sensor-only provisional holonomic acceleration envelope.

Limits require payload/configuration-specific physical validation. Emergency
sensor/obstacle stops override comfort slew limits. No simulator truth is input.
"""
import numpy as np

class MotionEnvelope:
    def __init__(self, speed=.12, yaw_speed=.2, acceleration=.20,
                 yaw_acceleration=.4, jerk=.8, yaw_jerk=1.6):
        values=np.array([speed,yaw_speed,acceleration,yaw_acceleration,jerk,yaw_jerk])
        if not np.isfinite(values).all() or (values<=0).any():
            raise ValueError('Limits must be positive and finite')
        self.speed,self.yaw_speed,self.acceleration,self.yaw_acceleration,self.jerk,self.yaw_jerk=values
        self.velocity=np.zeros(3);self.accel=np.zeros(3);self.tilt=np.zeros(2)
        self.initialized=False;self.latched=False

    def stop(self, reason):
        self.velocity[:]=0;self.accel[:]=0
        return self.velocity.copy(), {'reason':reason,'tilt_deg':np.degrees(self.tilt).tolist(),'latched':self.latched}

    def step(self, target, gyro, specific_force, dt, valid=True, visual_tilt=None):
        target=np.asarray(target,float);gyro=np.asarray(gyro,float);force=np.asarray(specific_force,float)
        if (not valid or any(x.shape!=(3,) for x in (target,gyro,force))
            or not all(np.isfinite(x).all() for x in (target,gyro,force))
            or not np.isfinite(dt) or not 0<dt<=.1):
            return self.stop('invalid_sensor_or_time')
        # Complementary roll/pitch observer. Gravity updates are gated during
        # commanded acceleration; raw acceleration is not treated as tilt.
        gravity=np.array([np.arctan2(force[1],force[2]),np.arctan2(-force[0],np.hypot(force[1],force[2]))])
        if not self.initialized:
            if abs(np.linalg.norm(force)-9.81)>.5:return self.stop('await_gravity')
            self.tilt=gravity;self.initialized=True
        roll,pitch=self.tilt;p,q,r=gyro
        rates=np.array([p+np.sin(roll)*np.tan(pitch)*q+np.cos(roll)*np.tan(pitch)*r,np.cos(roll)*q-np.sin(roll)*r])
        self.tilt+=rates*dt
        if abs(np.linalg.norm(force)-9.81)<.3 and np.linalg.norm(self.accel[:2])<.04:
            self.tilt+=(1-np.exp(-dt/1.))*((gravity-self.tilt+np.pi)%(2*np.pi)-np.pi)
        # Independent, current-stamp stereo attitude arrests gyro integration drift.
        # Caller must reject stale/lost tracking and planar-constrained odometry.
        if visual_tilt is not None:
            visual=np.asarray(visual_tilt,float)
            if visual.shape!=(2,) or not np.isfinite(visual).all():
                return self.stop('invalid_visual_attitude')
            if np.max(np.abs(visual))>=np.deg2rad(8):
                self.latched=True
            innovation=(visual-self.tilt+np.pi)%(2*np.pi)-np.pi
            if not self.latched and np.max(np.abs(innovation))>np.deg2rad(4):
                return self.stop('attitude_disagreement')
            self.tilt+=(1-np.exp(-dt/.2))*innovation
        tilt=float(np.max(np.abs(self.tilt)))
        if tilt>=np.deg2rad(8):self.latched=True
        if self.latched:return self.stop('tilt_stop')
        factor=float(np.clip((np.deg2rad(8)-tilt)/np.deg2rad(4),0,1))
        target=target.copy();target[:2]*=min(1,self.speed*factor/max(np.linalg.norm(target[:2]),1e-12));target[2]=np.clip(target[2],-self.yaw_speed*factor,self.yaw_speed*factor)
        desired=(target-self.velocity)/dt
        # Reserve acceleration for rotating body coordinates (omega cross v).
        budget=max(.01,self.acceleration-np.linalg.norm(self.velocity[:2])*abs(self.velocity[2]))
        desired[:2]*=min(1,budget/max(np.linalg.norm(desired[:2]),1e-12));desired[2]=np.clip(desired[2],-self.yaw_acceleration,self.yaw_acceleration)
        delta=desired-self.accel;delta[:2]*=min(1,self.jerk*dt/max(np.linalg.norm(delta[:2]),1e-12));delta[2]=np.clip(delta[2],-self.yaw_jerk*dt,self.yaw_jerk*dt)
        self.accel+=delta
        candidate=self.velocity+self.accel*dt
        # Avoid overshoot at zero/goal. Landing on target may shorten the final
        # acceleration step; jerk bounds apply away from this terminal clamp.
        for i in range(3):
            if (target[i]-self.velocity[i])*(target[i]-candidate[i])<=0:
                candidate[i]=target[i];self.accel[i]=0
        candidate[:2]*=min(1,self.speed*factor/max(np.linalg.norm(candidate[:2]),1e-12))
        candidate[2]=np.clip(candidate[2],-self.yaw_speed*factor,self.yaw_speed*factor)
        self.accel=(candidate-self.velocity)/dt
        self.velocity=candidate
        return candidate.copy(),{'reason':'limited','tilt_deg':np.degrees(self.tilt).tolist(),'latched':False,'speed_factor':factor}

class WheelSlew:
    """Limit wheel setpoint jumps after PPO scaling; interlocks override ramp."""
    def __init__(self, acceleration_rad_s2=.30/.03068719889482):
        self.acceleration=acceleration_rad_s2;self.previous=np.zeros(4)
    def step(self,target,dt=.05,interlock=False):
        target=np.asarray(target,float)
        if interlock or target.shape!=(4,) or not np.isfinite(target).all() or not 0<dt<=.1:
            self.previous[:]=0
        else:
            target=np.clip(target,-10,10)
            self.previous+=np.clip(target-self.previous,-self.acceleration*dt,self.acceleration*dt)
        return self.previous.copy()
