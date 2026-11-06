"""Simulation candidate: model-derived discrete LQR, no actuator admission.

A/B must describe the supplied sample period and measured state coordinates.
No R12 plant parameters are guessed here. Saturated control requires separate
nonlinear simulation validation; the linear stability result does not cover it.
"""
from dataclasses import dataclass
import numpy as np
from scipy.linalg import solve_discrete_are


def _matrix(value, shape=None):
    array = np.array(value, dtype=float, copy=True)
    if array.ndim != 2 or (shape is not None and array.shape != shape) or not np.isfinite(array).all():
        raise ValueError('invalid model matrix')
    return array


@dataclass(frozen=True)
class DiscreteLQR:
    gain: np.ndarray
    riccati: np.ndarray
    poles: np.ndarray
    sample_period_s: float
    model_source: str

    @classmethod
    def design(cls, A, B, Q, R, *, sample_period_s, model_source):
        if not np.isfinite(sample_period_s) or sample_period_s <= 0 or not isinstance(model_source, str) or not model_source.strip():
            raise ValueError('explicit sample period and model provenance required')
        a = _matrix(A)
        if a.shape[0] == 0 or a.shape[0] != a.shape[1]:
            raise ValueError('A must be nonempty and square')
        b = _matrix(B)
        n, m = a.shape[0], b.shape[1]
        if b.shape[0] != n or m == 0:
            raise ValueError('invalid input dimension')
        q, r = _matrix(Q, (n, n)), _matrix(R, (m, m))
        for matrix in (q, r):
            if not np.allclose(matrix, matrix.T, rtol=0, atol=1e-12):
                raise ValueError('cost must be symmetric')
        if np.linalg.eigvalsh(q).min() < -1e-12 or np.linalg.eigvalsh(r).min() <= 0:
            raise ValueError('Q must be positive semidefinite and R positive definite')
        try:
            p = solve_discrete_are(a, b, q, r)
            k = np.linalg.solve(r + b.T @ p @ b, b.T @ p @ a)
        except (ValueError, np.linalg.LinAlgError) as error:
            raise ValueError('no valid stabilizing Riccati solution') from error
        poles = np.linalg.eigvals(a - b @ k)
        residual = a.T @ p @ a - p - a.T @ p @ b @ k + q
        if not np.isfinite(k).all() or np.max(np.abs(poles)) >= 1 or np.linalg.norm(residual) > 1e-7 * max(1., np.linalg.norm(p)):
            raise ValueError('unstable or inaccurate LQR solution')
        for array in (k, p, poles):
            array.setflags(write=False)
        return cls(k, p, poles, float(sample_period_s), model_source)

    def command(self, error, *, equilibrium_input, lower, upper):
        """u=u_eq-K error, then externally supplied actuator envelope.

Caller provides sensor-derived error, valid equilibrium feedforward, current
voltage/velocity envelope and every independent freshness/collision interlock.
        """
        n, m = self.gain.shape[1], self.gain.shape[0]
        e, u, lo, hi = [np.asarray(x, dtype=float) for x in (error, equilibrium_input, lower, upper)]
        if e.shape != (n,) or any(x.shape != (m,) for x in (u, lo, hi)) or not all(np.isfinite(x).all() for x in (e, u, lo, hi)) or np.any(lo > hi):
            raise ValueError('invalid feedback or actuator envelope')
        raw = u - self.gain @ e
        return np.clip(raw, lo, hi)


def design_joint_lqr(mass, damping, Q, R, *, sample_period_s, model_source):
    """Local joint model x=[q-q_ref,dq-dq_ref], u=effort-effort_eq.

Caller supplies validated effective mass for its base/contact constraints.
A submatrix of floating-base mass is NOT automatically an effective mass.
Gravity/coriolis equilibrium and model validity are caller responsibilities.
    """
    from scipy.linalg import expm
    mass = _matrix(mass)
    if mass.shape[0] == 0 or mass.shape[0] != mass.shape[1]:
        raise ValueError('invalid effective mass dimensions')
    n = mass.shape[0]
    damping = _matrix(damping, (n, n))
    if not np.allclose(mass, mass.T, atol=1e-12, rtol=0) or np.linalg.eigvalsh(mass).min() <= 0:
        raise ValueError('effective mass must be symmetric positive definite')
    if not np.allclose(damping, damping.T, atol=1e-12, rtol=0) or np.linalg.eigvalsh(damping).min() < -1e-12:
        raise ValueError('damping must be symmetric positive semidefinite')
    if not np.isfinite(sample_period_s) or sample_period_s <= 0:
        raise ValueError('invalid sample period')
    continuous = np.block([[np.zeros((n,n)), np.eye(n)], [np.zeros((n,n)), -np.linalg.solve(mass,damping)]])
    inputs = np.vstack([np.zeros((n,n)),np.linalg.solve(mass,np.eye(n))])
    augmented = np.zeros((3*n,3*n))
    augmented[:2*n,:2*n] = continuous
    augmented[:2*n,2*n:] = inputs
    discrete = expm(augmented*sample_period_s)
    return DiscreteLQR.design(discrete[:2*n,:2*n], discrete[:2*n,2*n:], Q, R,
                              sample_period_s=sample_period_s, model_source=model_source)
