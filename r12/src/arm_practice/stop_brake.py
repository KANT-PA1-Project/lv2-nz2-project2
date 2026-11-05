"""Sensor-only stop yaw damping candidate; runtime integration is opt-in."""
import numpy as np


def stop_yaw_targets(steer, gyro_z, previous, dt, *, valid=True):
    """Return bounded wheel targets while the caller commands a normal stop.

    Do not use for emergency stops. Caller must enforce fresh sensors and
    command ownership. Uses no simulation ground truth.
    """
    steer = np.asarray(steer, dtype=float)
    previous = np.asarray(previous, dtype=float)
    if (not valid or steer.shape != (4,) or previous.shape != (4,)
            or not np.isfinite(steer).all() or not np.isfinite(previous).all()
            or not np.isfinite(gyro_z) or not np.isfinite(dt) or not 0 < dt <= .1):
        return np.zeros(4)
    yaw = np.clip(-.8 * gyro_z, -.15, .15) if abs(gyro_z) > .005 else 0.
    heading = steer + np.array([-np.pi/2, -np.pi/2, 0., np.pi])
    x = np.array([.1, .1, -.1, -.1])
    y = np.array([.1, -.1, .1, -.1])
    desired = yaw * (-y * np.cos(heading) + x * np.sin(heading)) / .03068719889482
    return np.clip(previous + np.clip(desired - previous, -24.45*dt, 24.45*dt),
                   -29.321531434, 29.321531434)
