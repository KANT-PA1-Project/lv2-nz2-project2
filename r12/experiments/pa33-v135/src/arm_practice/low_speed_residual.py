"""Opt-in runtime diagnostic limiting PPO corrections near a normal stop.

This changes neither the saved policy nor motor torque limits. At wheel-vector
speeds <= 0.04 m/s a correction cannot reverse a nominal wheel target. Between
0.04 and 0.08 m/s it smoothly returns to the existing residual behavior.
"""
import numpy as np


def limit_low_speed_residual(nominal, residual, peak_speed_m_s, fraction):
    nominal = np.asarray(nominal, dtype=float)
    residual = np.asarray(residual, dtype=float)
    if (nominal.shape != (4,) or residual.shape != (4,)
            or not np.isfinite(nominal).all() or not np.isfinite(residual).all()
            or not np.isfinite(peak_speed_m_s) or peak_speed_m_s < 0
            or not np.isfinite(fraction) or not 0 <= fraction < 1):
        raise ValueError('Invalid low-speed residual inputs')
    if peak_speed_m_s >= .08:
        return residual.copy()
    bounded = np.clip(residual, -fraction * np.abs(nominal), fraction * np.abs(nominal))
    if peak_speed_m_s <= .04:
        return bounded
    x = (peak_speed_m_s - .04) / .04
    blend = x * x * (3 - 2 * x)
    return bounded + blend * (residual - bounded)
