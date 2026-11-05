"""Smooth a planned center path without changing occupancy or CAD clearance."""
import numpy as np
from scipy.spatial import ConvexHull


def smooth_path(path, occupied, clearance, resolution, footprint, yaw=np.pi):
    points = np.asarray(path, float)
    if len(points) < 3 or not np.isfinite(points).all():
        return points.tolist()
    # Reduce grid stair steps while retaining original endpoints.
    keep = [0]
    for i in range(1, len(points)-1):
        if np.linalg.norm(points[i]-points[keep[-1]]) >= .06:
            keep.append(i)
    keep.append(len(points)-1)
    p = points[keep].copy()
    vertices = np.asarray(footprint, float)
    hull = ConvexHull(vertices)
    gx, gy = np.meshgrid(np.arange(vertices[:,0].min(), vertices[:,0].max()+resolution*.5, resolution*.5),
                         np.arange(vertices[:,1].min(), vertices[:,1].max()+resolution*.5, resolution*.5))
    samples = np.column_stack([gx.ravel(), gy.ravel()])
    samples = samples[(samples@hull.equations[:,:2].T+hull.equations[:,2] <= 1e-9).all(axis=1)]
    c,s = np.cos(yaw),np.sin(yaw)
    offsets = np.vstack([samples,vertices]) @ np.array([[c,s],[-s,c]])
    def clear(a,b):
        centers=np.linspace(a,b,max(2,int(np.ceil(np.linalg.norm(b-a)/(resolution*.5)))+1))
        cells=np.floor((centers[:,None,:]+offsets[None,:,:])/resolution).astype(int).reshape(-1,2)
        inside=(cells[:,0]>=0)&(cells[:,0]<occupied.shape[1])&(cells[:,1]>=0)&(cells[:,1]<occupied.shape[0])
        return inside.all() and not occupied[cells[:,1],cells[:,0]].any()
    # Never simplify through obstacles. Existing follower will safely reject
    # any original path segment which is itself incompatible with the footprint.
    if not all(clear(a,b) for a,b in zip(p[:-1],p[1:])):
        return points.tolist()
    gy,gx=np.gradient(clearance,resolution)
    for _ in range(15):
        for i in range(1,len(p)-1):
            col,row=np.floor(p[i]/resolution).astype(int)
            grad=np.array([gx[row,col],gy[row,col]])
            norm=np.linalg.norm(grad)
            # A soft preference for free space; not an inflated obstacle or
            # a new minimum distance. Narrow passages remain admissible.
            away=grad/max(norm,1.)*min(.012,max(0.,.45-clearance[row,col])*.04)
            move=.25*(p[i-1]+p[i+1]-2*p[i])+away
            move*=min(1.,.025/max(np.linalg.norm(move),1e-12))
            candidate=p[i]+move
            if clear(p[i-1],candidate) and clear(candidate,p[i+1]):p[i]=candidate
    return p.tolist()
