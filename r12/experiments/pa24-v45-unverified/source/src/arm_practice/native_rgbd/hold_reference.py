"""Preserve an existing stationary arm command across camera initialization.

Under gravity, measured equilibrium differs from a PD position target. Replacing
that target with the measurement removes the position-error support torque.
This helper is not a fault-stop policy or evidence that the arm has settled.
"""
import numpy as np


def preserve_arm_reference(measured, commanded, arm_indices):
    measured = np.asarray(measured, dtype=float)
    commanded = np.asarray(commanded, dtype=float)
    if (measured.ndim != 1 or measured.shape != commanded.shape
            or not np.isfinite(measured).all()
            or not np.isfinite(commanded).all()):
        raise ValueError('invalid hold reference')
    indices = list(arm_indices)
    if (len(indices) != 5
            or any(type(i) is not int or not 0 <= i < len(measured) for i in indices)
            or len(set(indices)) != 5):
        raise ValueError('invalid arm mapping')
    result = measured.copy()
    result[indices] = commanded[indices]
    return result
