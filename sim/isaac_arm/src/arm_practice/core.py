"""Small numerical examples for learning the action/reward contract."""
import numpy as np


def joint_targets(action, home, scale, lower, upper):
    """Normalized action -> bounded absolute target, in configured joint order."""
    a, q, s, lo, hi = [np.asarray(x, dtype=float) for x in (action, home, scale, lower, upper)]
    if a.ndim != 1 or not (a.shape == q.shape == s.shape == lo.shape == hi.shape):
        raise ValueError("All inputs must be matching one-dimensional joint vectors")
    if not all(np.isfinite(x).all() for x in (a, q, s, lo, hi)):
        raise ValueError("NaN/Inf joint inputs are invalid")
    if np.any(lo >= hi) or np.any(s <= 0) or np.any((q < lo) | (q > hi)):
        raise ValueError("Invalid limits, home pose or action scale")
    return np.clip(q + s * np.clip(a, -1, 1), lo, hi)


def reach_score(tcp, target, action, previous_action, tolerance=0.03):
    """CPU teaching example, not the complete Isaac Lab reward implementation."""
    p, g, a, prev = [np.asarray(x, dtype=float) for x in (tcp, target, action, previous_action)]
    if p.shape != (3,) or g.shape != (3,) or a.ndim != 1 or a.shape != prev.shape:
        raise ValueError("Expected xyz and matching action vectors")
    if not all(np.isfinite(x).all() for x in (p, g, a, prev)) or not np.isfinite(tolerance) or tolerance <= 0:
        raise ValueError("Inputs must be finite; tolerance must be positive")
    distance = float(np.linalg.norm(p - g))
    success = distance <= tolerance
    reward = -distance - 0.01 * float(np.sum((a - prev) ** 2)) + 2 * success
    return {"distance_m": distance, "success": bool(success), "reward": float(reward)}
