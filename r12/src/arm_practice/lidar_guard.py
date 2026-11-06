"""Body-frame lidar velocity constraints; permit motion away from an obstacle."""
import numpy as np
from .planar_projection import project_velocity

def constrain_velocity(command, scans, extrinsics, radius=.30, horizon=.6, footprint=None):
    velocity=np.asarray(command,dtype=float).copy()
    if not np.isfinite(velocity).all() or set(scans)!=set(extrinsics):
        return np.zeros(3), 'invalid_sensor'
    points=[]
    for name,ranges in scans.items():
        r=np.asarray(ranges)
        if r.shape!=(90,) or not np.isfinite(r).all() or (r<0).any():
            return np.zeros(3), 'invalid_sensor'
        angles=np.linspace(-np.pi/2,np.pi/2,len(r))
        local=np.column_stack([r*np.cos(angles),r*np.sin(angles),np.zeros(len(r))])
        t=np.asarray(extrinsics[name])
        points.extend((local@t[:3,:3].T+t[:3,3])[r<5.95,:2])
    points=np.asarray(points).reshape(-1,2)
    if not len(points):return velocity,None
    distance=np.linalg.norm(points,axis=1)
    if (distance<.01).any():return np.zeros(3),'invalid_sensor'
    normal=points/distance[:,None]
    original=velocity.copy()
    if footprint is None:
        support=radius
    else:
        vertices=np.asarray(footprint,float)
        if vertices.ndim!=2 or vertices.shape[1]!=2 or len(vertices)<3 or not np.isfinite(vertices).all():return np.zeros(3),'invalid_sensor'
        # Support planes conservatively contain the asymmetric transport hull.
        support=np.max(normal@vertices.T,axis=1)
        reach=float(np.linalg.norm(vertices,axis=1).max())
        yaw_limit=float(np.min(np.maximum(0,distance-support))/(horizon*reach))
        velocity[2]=float(np.clip(velocity[2],-yaw_limit,yaw_limit))
    limits=np.maximum(0,(distance-support)/horizon)
    if footprint is not None:limits=np.maximum(0,limits-abs(velocity[2])*reach)
    # Solve the same convex no-approach constraints exactly in two dimensions.
    # A fixed iteration limit previously zeroed otherwise feasible translation
    # near oblique walls while leaving yaw active.
    velocity[:2]=project_velocity(velocity[:2],normal,limits)
    return velocity, 'limited' if np.linalg.norm(velocity-original)>1e-8 else None
