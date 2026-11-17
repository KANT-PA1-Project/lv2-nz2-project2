"""Nearest planar velocity in an intersection of lidar safety half-planes."""
import numpy as np


def project_velocity(target, normals, limits):
    """Solve min ||v-target||² subject to normals @ v <= limits.

    Incremental two-dimensional active-set projection. When a new plane
    excludes the previous optimum, the new optimum is on its boundary. The
    earlier planes reduce that line to an interval, whose nearest point is
    found analytically. Nonnegative limits guarantee a feasible zero velocity.
    No safety limit is expanded; degeneracy fails to zero.
    """
    target=np.asarray(target,float);normals=np.asarray(normals,float);limits=np.asarray(limits,float)
    if (target.shape!=(2,) or normals.ndim!=2 or normals.shape[1]!=2
            or limits.shape!=(len(normals),) or not np.isfinite(target).all()
            or not np.isfinite(normals).all() or not np.isfinite(limits).all()
            or (limits<0).any()):
        raise ValueError('Finite planar target and feasible half-planes required')
    lengths=np.linalg.norm(normals,axis=1)
    if (lengths<1e-12).any():raise ValueError('Zero half-plane normal')
    normals=normals/lengths[:,None];limits=limits/lengths
    result=target.copy()
    for i,(n,b) in enumerate(zip(normals,limits)):
        if n@result<=b+1e-10:continue
        anchor=n*b;direction=np.array([-n[1],n[0]])
        coefficients=normals[:i]@direction
        bounds=limits[:i]-normals[:i]@anchor
        parallel=np.abs(coefficients)<1e-12
        if np.any(bounds[parallel]<-1e-9):return np.zeros(2)
        positive=coefficients>1e-12;negative=coefficients< -1e-12
        high=float(np.min(bounds[positive]/coefficients[positive])) if positive.any() else np.inf
        low=float(np.max(bounds[negative]/coefficients[negative])) if negative.any() else -np.inf
        if low>high+1e-9:return np.zeros(2)
        coordinate=np.clip(direction@(target-anchor),low,high)
        result=anchor+coordinate*direction
    if np.any(normals@result>limits+1e-8):return np.zeros(2)
    return result
