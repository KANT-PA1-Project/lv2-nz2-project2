"""Conservative free-volume queries under a piecewise-constant depth model.

This is geometry only: callers must separately establish timestamp, calibration,
registration, sensor uncertainty, motion bounds and scene validity.
"""
import itertools
import numpy as np
from scipy.ndimage import minimum_filter


def obb_volume_cells(transform, size, voxel_size, clearance=.03):
    """Over-cover an OBB's Euclidean clearance volume with whole grid cells.

    The half-diagonal allowance includes every cell intersecting the clearance
    volume. This is a single-pose cover, not a bound between joint samples.
    """
    transform, size = np.asarray(transform, float), np.asarray(size, float)
    if (transform.shape != (4,4) or size.shape != (3,)
            or not np.isfinite(transform).all() or not np.isfinite(size).all()
            or np.any(size <= 0) or not np.isfinite(voxel_size) or voxel_size <= 0
            or not np.isfinite(clearance) or clearance < 0):
        raise ValueError('Invalid OBB geometry')
    rotation, translation = transform[:3,:3], transform[:3,3]
    if not np.allclose(rotation.T@rotation, np.eye(3), atol=1e-7):
        raise ValueError('OBB transform must be rigid')
    allowance = clearance + np.sqrt(3)*voxel_size/2
    extent = np.abs(rotation)@(size/2)+allowance
    low = np.floor((translation-extent)/voxel_size).astype(int)
    high = np.ceil((translation+extent)/voxel_size).astype(int)
    cells = np.stack(np.meshgrid(*[np.arange(a,b+1) for a,b in zip(low,high)],
                                indexing='ij'), axis=-1).reshape(-1,3)
    local = ((cells+.5)*voxel_size-translation)@rotation
    distance = np.linalg.norm(np.maximum(np.abs(local)-size/2, 0), axis=1)
    return cells[distance <= allowance+1e-12]


def depth_covers_voxels(centers, half_size, camera_to_reference, depth,
                        *, intrinsics=(400., 400., 320., 240.), depth_margin=.03,
                        exact_rectangle=False):
    """Require valid clearance over the entire projected box rectangle.

Enclosing square minimum filters can reject extra space but cannot clear an
unobserved pixel. Corner-only checks are insufficient for the rectangle interior.
"""
    centers = np.asarray(centers, float)
    transform = np.asarray(camera_to_reference, float)
    depth = np.asarray(depth, float)
    if (centers.ndim != 2 or centers.shape[1] != 3 or not np.isfinite(centers).all()
            or not np.isfinite(half_size) or half_size <= 0
            or not np.isfinite(depth_margin) or depth_margin < 0
            or transform.shape != (4,4) or not np.isfinite(transform).all()
            or depth.ndim != 2):
        raise ValueError('Invalid volume geometry')
    if not np.allclose(transform[3], [0,0,0,1]) or not np.allclose(
            transform[:3,:3].T@transform[:3,:3], np.eye(3), atol=1e-7):
        raise ValueError('Camera transform must be rigid')
    fx,fy,cx,cy = intrinsics
    if not np.isfinite(intrinsics).all() or fx <= 0 or fy <= 0:
        raise ValueError('Invalid intrinsics')
    signs = np.asarray(list(itertools.product([-1.,1.], repeat=3)))
    corners = centers[:,None,:]+half_size*signs[None,:,:]
    camera = (corners-transform[:3,3])@transform[:3,:3]
    zmin = camera[:,:,2].min(axis=1)
    zmax = camera[:,:,2].max(axis=1)
    valid = zmin > .03
    ids = np.flatnonzero(valid)
    free = np.zeros(len(centers), bool)
    if not len(ids):
        return free
    xy = camera[ids,:,:2]/camera[ids,:,2,None]*[fx,fy]+[cx,cy]
    low = np.floor(xy.min(axis=1)).astype(int)
    high = np.ceil(xy.max(axis=1)).astype(int)
    height,width = depth.shape
    inside = (low[:,0]>=0)&(high[:,0]<width)&(low[:,1]>=0)&(high[:,1]<height)
    ids,low,high = ids[inside],low[inside],high[inside]
    safe_depth = np.where(np.isfinite(depth)&(depth>.03)&(depth<10), depth, 0.)
    if exact_rectangle:
        # Keep every pixel in the projected rectangle, including its interior.
        # Unlike the faster square approximation, exclude pixels outside it.
        for index,lo,hi in zip(ids,low,high):
            minimum = safe_depth[lo[1]:hi[1]+1,lo[0]:hi[0]+1].min()
            free[index] = zmax[index] < minimum-depth_margin
        return free
    center = (low+high)//2
    radius = np.maximum(high-center, center-low).max(axis=1)
    # Round up, never down, to bound both image axes with one filter.
    powers = np.where(radius==0, 0, 2**np.ceil(np.log2(np.maximum(radius,1))).astype(int))
    for power in np.unique(powers):
        selection = powers == power
        minimum = minimum_filter(safe_depth, size=int(2*power+1), mode='constant', cval=0.)
        z = minimum[center[selection,1], center[selection,0]]
        free[ids[selection]] = zmax[ids[selection]] < z-depth_margin
    return free
