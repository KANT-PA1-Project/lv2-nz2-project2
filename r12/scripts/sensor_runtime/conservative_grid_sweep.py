"""Inactive candidate: conservative closed-cell cover of a convex footprint.

This does not exempt present occupied volume or erase unknown. The footprint
must already contain the CAD safety padding. No live freshness authorization.
"""
import numpy as np
from functools import lru_cache


def convex_hull(vertices):
    """CCW monotone-chain hull using only stdlib/numpy; reject degenerate input."""
    vertices = np.asarray(vertices, float)
    if (vertices.ndim != 2 or vertices.shape[1] != 2
            or not 3 <= len(vertices) <= 4096
            or not np.isfinite(vertices).all()
            or np.max(np.abs(vertices)) > 1_000_000.):
        raise ValueError('invalid_or_unbounded_footprint')
    points = sorted(set(map(tuple, vertices)))
    if len(points) < 3:
        raise ValueError('degenerate_footprint')
    def cross(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    lower, upper = [], []
    for point in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)
    for point in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)
    result = np.asarray(lower[:-1]+upper[:-1])
    if len(result) < 3:
        raise ValueError('degenerate_footprint')
    return result


# Exact-value preparation caches only; never cache cells, poses or map decisions.
# Each key owns immutable float64 bytes. 16 entries per cache bound worst-case
# resident polygon/axis payload to less than 5 MiB at the 4096-vertex limit.
@lru_cache(maxsize=16)
def _body_preparation(raw, count):
    p = convex_hull(np.frombuffer(raw, dtype=np.float64).reshape(count, 2))
    p.setflags(write=False)
    return p, float(np.linalg.norm(p, axis=1).max())


def _prepare_body(footprint):
    p = np.asarray(footprint, dtype=np.float64)
    if (p.ndim != 2 or p.shape[1] != 2 or not 3 <= len(p) <= 4096
            or not np.isfinite(p).all() or np.max(np.abs(p)) > 1_000_000.):
        raise ValueError('invalid_or_unbounded_footprint')
    return _body_preparation(p.tobytes(order='C'), len(p))


@lru_cache(maxsize=16)
def _axis_preparation(raw, count, yaw, interval_angle):
    p = np.frombuffer(raw, dtype=np.float64).reshape(count, 2)
    c, s = np.cos(yaw), np.sin(yaw)
    ca, sa = np.cos(interval_angle), np.sin(interval_angle)
    rotation = np.array([[c, -s], [s, c]])
    heading = np.array([[ca, -sa], [sa, ca]])
    # Only the axes are prepared in a translation-free coordinate system.
    # Every call still projects its current world vertices onto these axes.
    # SAT on any finite axes is conservative, even with roundoff in normals.
    p = (p @ heading.T) @ rotation.T
    edges = np.roll(p, -1, axis=0) - p
    normals = np.column_stack((-edges[:, 1], edges[:, 0]))
    lengths = np.linalg.norm(normals, axis=1)
    if not np.isfinite(lengths).all() or np.any(lengths <= 0):
        raise ValueError('invalid_prepared_axes')
    normals /= lengths[:, None]
    axes = np.vstack((np.eye(2), normals))
    axes.setflags(write=False)
    return axes


def preparation_cache_info():
    return dict(body=_body_preparation.cache_info()._asdict(),
                axes=_axis_preparation.cache_info()._asdict())


def clear_preparation_cache():
    _body_preparation.cache_clear()
    _axis_preparation.cache_clear()


def convex_cell_cover(vertices, resolution, expansion=0., *, max_cells=2_000_000,
                      max_axis_cell_tests=5_000_000):
    """Cells intersecting convex polygon plus an axis-aligned expansion square.

    Separating-axis tests use cell axes and polygon edge normals. The square
    expansion bounds a Euclidean displacement ball; positive-area and touching
    intersections are both retained. Numerical tolerance only adds cells.
    """
    p = convex_hull(vertices)
    edges = np.roll(p, -1, axis=0) - p
    normals = np.column_stack((-edges[:, 1], edges[:, 0]))
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    axes = np.vstack((np.eye(2), normals))
    return _cell_cover_prepared(p, axes, resolution, expansion, max_cells=max_cells,
                                max_axis_cell_tests=max_axis_cell_tests)


def _cell_spans_prepared(p, axes, resolution, expansion, *, max_cells,
                         max_axis_cell_tests=5_000_000):
    if not np.isfinite(p).all() or np.max(np.abs(p)) > 1_000_000.:
        raise ValueError('invalid_or_unbounded_footprint')
    # A valid body polygon may collapse after translation in finite precision.
    # Preserve the old world-hull failure rather than accepting a point/line.
    relative = p[1:] - p[0]
    if not np.any(relative[:-1, 0] * relative[1:, 1]
                  - relative[:-1, 1] * relative[1:, 0]):
        raise ValueError('degenerate_world_footprint')
    if (not np.isfinite(resolution) or not 1e-6 <= resolution <= 1000.
            or not np.isfinite(expansion) or not 0 <= expansion <= 1_000_000.
            or not isinstance(max_cells, int) or not 0 < max_cells <= 2_000_000
            or not isinstance(max_axis_cell_tests, int)
            or not 0 < max_axis_cell_tests <= 5_000_000):
        raise ValueError('invalid_footprint_cover')
    tol = max(1e-12, 64*np.finfo(float).eps*max(1.,np.max(np.abs(p)),expansion,resolution))
    low_float = np.floor((p.min(0) - expansion - tol) / resolution)
    high_float = np.floor((p.max(0) + expansion + tol) / resolution)
    # Bound finite huge values before integer conversion or grid allocation.
    if max(np.max(np.abs(low_float)),np.max(np.abs(high_float))) > 1_000_000_000:
        raise ValueError('bounded_grid_index')
    low, high = low_float.astype(np.int64), high_float.astype(np.int64)
    shape = high - low + 1
    total = int(shape[0])*int(shape[1])
    if (np.any(shape <= 0) or total > max_cells
            or total*(len(p)+2) > max_axis_cell_tests):
        raise ValueError('bounded_footprint_cover_budget')
    # Equivalent SAT inequalities, solved per y-row for a closed x interval.
    # Convexity makes each row contiguous; no per-cell dense projection array.
    projection = p @ axes.T
    support = (resolution / 2 + expansion) * np.abs(axes).sum(1)
    lo = projection.min(0) - support - tol
    hi = projection.max(0) + support + tol
    # Keep temporary row-by-axis matrices within the original 2MiB bound.
    # A very tall thin polygon must not allocate all row projections at once.
    ax, ay = axes[:,0], axes[:,1]
    zero, nonzero = ax == 0., ax != 0.
    chunk = max(1,min(2048,262144//len(axes)))
    total_kept = 0
    for begin in range(int(low[1]),int(high[1])+1,chunk):
        ys = np.arange(begin,min(begin+chunk,int(high[1])+1),dtype=np.int64)
        yc = (ys+.5)*resolution
        eligible = ((yc[:,None]*ay[zero] >= lo[zero])
                    & (yc[:,None]*ay[zero] <= hi[zero])).all(1)
        x1 = (lo[nonzero]-yc[:,None]*ay[nonzero])/ax[nonzero]
        x2 = (hi[nonzero]-yc[:,None]*ay[nonzero])/ax[nonzero]
        lower = np.minimum(x1,x2).max(1)
        upper = np.maximum(x1,x2).min(1)
        # Extra tolerance only broadens coverage; keep the bounded grid.
        ix0 = np.maximum(low[0], np.ceil(lower/resolution-.5-tol/resolution))
        ix1 = np.minimum(high[0], np.floor(upper/resolution-.5+tol/resolution))
        eligible &= ix0 <= ix1
        starts, stops = ix0[eligible].astype(np.int64), ix1[eligible].astype(np.int64)
        counts = stops-starts+1
        n = int(counts.sum());total_kept += n
        if total_kept > max_cells:raise ValueError('bounded_scanline_cover_budget')
        if n == 0:continue
        yield ys[eligible], starts, stops


def _cell_cover_prepared(p, axes, resolution, expansion, *, max_cells,
                         max_axis_cell_tests=5_000_000):
    """Materialize exactly the same closed scanline spans for diagnostics."""
    parts = []
    for ys, starts, stops in _cell_spans_prepared(p, axes, resolution, expansion,
            max_cells=max_cells, max_axis_cell_tests=max_axis_cell_tests):
        counts = stops-starts+1
        n = int(counts.sum())
        row_start = np.r_[0,np.cumsum(counts[:-1])]
        xs = np.repeat(starts,counts)+np.arange(n)-np.repeat(row_start,counts)
        parts.append(np.column_stack((xs,np.repeat(ys,counts))))
    return np.vstack(parts) if parts else np.empty((0,2),np.int64)





def _trajectory_prepared(xy, yaw, command, footprint, resolution, horizon=.6,
                       *, max_cells=2_000_000, max_queries=5_000_000):
    """Cover every constant-body-twist interval, not just sampled poses.

    Any material point moves <= (|v|+|w|*radius)*dt during one interval.
    Expanding the interval-start polygon by that distance along both axes
    therefore over-covers its entire intermediate sweep. No old-cover subtraction.
    """
    xy, command = (np.asarray(x, float) for x in (xy, command))
    vertices, radius = _prepare_body(footprint)
    if (xy.shape != (2,) or command.shape != (3,)
            or not np.isfinite(np.r_[xy, command, vertices.ravel(), yaw, resolution, horizon]).all()
            or not 1e-6 <= resolution <= 1000. or not 0 <= horizon <= 1000.
            or np.max(np.abs(xy)) > 1_000_000. or abs(yaw) > 1_000_000.
            or np.max(np.abs(command)) > 1_000_000.
            or not isinstance(max_queries,int) or not 0 < max_queries <= 5_000_000):
        raise ValueError('invalid_sweep_input')
    raw_vertices = vertices.tobytes(order='C')
    speed = np.linalg.norm(command[:2]) + abs(command[2]) * radius
    count = max(1, int(np.ceil(horizon * speed / (resolution * .5))))
    if count > 10000:
        raise ValueError('bounded_sweep_interval_budget')
    dt = horizon / count
    c, s = np.cos(yaw), np.sin(yaw)
    rotation = np.array([[c, -s], [s, c]])
    vx, vy, w = command
    for i in range(count):
        t = i * dt
        a = w * t
        ca, sa = np.cos(a), np.sin(a)
        if abs(w) < 1e-9:
            local = command[:2] * t
        else:
            local = np.array([(sa * vx - (1 - ca) * vy) / w,
                              ((1 - ca) * vx + sa * vy) / w])
        heading = np.array([[ca, -sa], [sa, ca]])
        world = (vertices @ heading.T + local) @ rotation.T + xy
        axes = _axis_preparation(raw_vertices, len(vertices), float(yaw), float(a))
        yield world, axes, speed * dt


def _trajectory_covers(xy, yaw, command, footprint, resolution, horizon=.6,
                       *, max_cells=2_000_000, max_queries=5_000_000):
    queries = 0
    for world, axes, expansion in _trajectory_prepared(xy,yaw,command,footprint,
            resolution,horizon,max_cells=max_cells,max_queries=max_queries):
        cells = _cell_cover_prepared(world, axes, resolution, expansion, max_cells=max_cells)
        queries += len(cells)
        if queries > max_queries:
            raise ValueError('bounded_sweep_cell_budget')
        yield cells


def trajectory_cover(xy, yaw, command, footprint, resolution, horizon=.6,
                     *, max_cells=2_000_000, max_queries=5_000_000):
    """Diagnostic union; production veto streams one bounded interval at a time."""
    return np.unique(np.vstack(list(_trajectory_covers(xy,yaw,command,footprint,
        resolution,horizon,max_cells=max_cells,max_queries=max_queries))),axis=0)


def motion_clear(xy, yaw, command, footprint, occupied, resolution, horizon=.6):
    """Fail-closed CPU geometry veto. Unknown must remain occupied at caller."""
    try:
        occupied = np.asarray(occupied)
        if occupied.ndim != 2 or min(occupied.shape) < 1 or occupied.size > 2_000_000:
            return False
        if occupied.dtype != np.bool_:
            # Existing static maze maps use uint8 0/1; live slices use bool.
            # Never cast floats, unknown -1 or nonbinary values to free space.
            if (not np.issubdtype(occupied.dtype,np.integer)
                    or not ((occupied==0)|(occupied==1)).all()):
                return False
        queries = 0
        for world, axes, expansion in _trajectory_prepared(xy,yaw,command,footprint,
                resolution,horizon):
            for ys, starts, stops in _cell_spans_prepared(world,axes,resolution,
                    expansion,max_cells=2_000_000):
                # The exact old cells are contiguous [start, stop] in each row.
                # Query that set directly; never skip unknown or out-of-grid.
                queries += int((stops-starts+1).sum())
                if queries > 5_000_000:
                    raise ValueError('bounded_sweep_cell_budget')
                if (ys[0] < 0 or ys[-1] >= occupied.shape[0]
                        or starts.min() < 0 or stops.max() >= occupied.shape[1]):
                    return False
                left, right = int(starts.min()), int(stops.max())
                xs = np.arange(left, right+1, dtype=np.int64)
                covered = (xs[None,:] >= starts[:,None]) & (xs[None,:] <= stops[:,None])
                if np.any(occupied[ys, left:right+1], where=covered):
                    return False
        return True
    except (ValueError, TypeError, IndexError, OverflowError):
        return False
