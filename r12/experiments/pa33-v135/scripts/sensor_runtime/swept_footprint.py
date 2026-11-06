"""Bounded conservative constant-twist geometry veto; no new-space exemption.

Footprint already contains CAD padding. Known-map, freshness and actuator
contracts remain with callers. This module requires only numpy and stdlib.
"""
from functools import lru_cache
import numpy as np
from conservative_grid_sweep import convex_hull, convex_cell_cover, motion_clear


@lru_cache(maxsize=16)
def _samples(vertices, resolution):
    """Legacy planning samples, not a full-cell safety cover.

    Retained for coarse candidate construction and query estimates. Final veto
    and footprint evidence use convex_cell_cover, never this sample list alone.
    """
    ordered = convex_hull(vertices)
    if not np.isfinite(resolution) or not 1e-6 <= resolution <= 1000.:
        raise ValueError('invalid_sample_resolution')
    step = resolution*.5
    shape = np.ceil((ordered.max(0)-ordered.min(0))/step).astype(np.int64)+1
    count = int(shape[0])*int(shape[1])
    if count*len(ordered)>5_000_000:
        raise ValueError('bounded_planning_sample_budget')
    gx,gy = np.meshgrid(np.arange(ordered[:,0].min(),ordered[:,0].max()+step,step),
                        np.arange(ordered[:,1].min(),ordered[:,1].max()+step,step))
    points = np.column_stack((gx.ravel(),gy.ravel()))
    edges = np.roll(ordered,-1,axis=0)-ordered
    normals = np.column_stack((edges[:,1],-edges[:,0]))
    normals /= np.linalg.norm(normals,axis=1)[:,None]
    offset = -(ordered*normals).sum(1)
    points = points[(points@normals.T+offset<=1e-9).all(axis=1)]
    boundary = [np.linspace(a,b,max(2,int(np.ceil(np.linalg.norm(b-a)/step))+1))
                for a,b in zip(ordered,np.roll(ordered,-1,axis=0))]
    return np.vstack([points,*boundary])
