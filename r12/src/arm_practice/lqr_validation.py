"""Offline identification checks; never authorizes actuator control."""
import numpy as np


def affine_rollout_errors(A, B, offset, states, efforts, step_indices, *, stride):
    """Compare measured holdout data with one-step and uncorrected rollouts.

    states contains N+1 samples; efforts[k] was held over transition k.
    Reject gaps instead of treating samples across resets as adjacent. Units,
    actuator alignment and an independent holdout split remain caller duties.
    No score here is a mission or closed-loop performance acceptance.
    """
    a, b, c, x, u = [np.asarray(v, dtype=float) for v in
                     (A, B, offset, states, efforts)]
    if a.ndim != 2 or a.shape[0] == 0 or a.shape[0] != a.shape[1]:
        raise ValueError('invalid A')
    n = a.shape[0]
    if (b.ndim != 2 or b.shape[0] != n or b.shape[1] == 0
            or c.shape != (n,) or x.ndim != 2 or x.shape[1] != n
            or len(x) < 2 or u.shape != (len(x)-1, b.shape[1])
            or not all(np.isfinite(v).all() for v in (a, b, c, x, u))):
        raise ValueError('invalid affine holdout data')
    indices = list(step_indices)
    if (type(stride) is not int or stride <= 0 or len(indices) != len(x)
            or any(type(k) is not int or k < 0 for k in indices)
            or any(right-left != stride for left, right in zip(indices, indices[1:]))):
        raise ValueError('noncontiguous physics samples')
    predicted = np.empty_like(x)
    predicted[0] = x[0]
    with np.errstate(over='raise', invalid='raise'):
        try:
            one_step = x[:-1] @ a.T + u @ b.T + c
            for k in range(len(u)):
                predicted[k+1] = a @ predicted[k] + b @ u[k] + c
            one_rmse = np.sqrt(np.mean((one_step-x[1:])**2, axis=0))
            rollout_rmse = np.sqrt(np.mean((predicted[1:]-x[1:])**2, axis=0))
            terminal_error = predicted[-1]-x[-1]
        except FloatingPointError as error:
            raise ValueError('nonfinite model rollout') from error
    return {'transitions': len(u), 'one_step_rmse': one_rmse.tolist(),
            'rollout_rmse': rollout_rmse.tolist(),
            'terminal_error': terminal_error.tolist(),
            'motion_authorized': False}
