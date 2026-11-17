"""Geometric heading proposal for known path tangents, not a collision certificate."""
import numpy as np
from .swerve import BASE_DIRECTIONS


def steering_margin(velocities,heading):
    v=np.asarray(velocities,float)
    direction=np.arctan2(v[:,1],v[:,0])-heading
    raw=direction[:,None]-BASE_DIRECTIONS
    angle=(raw+np.pi/2)%np.pi-np.pi/2
    return float(np.min(np.pi/2-np.abs(angle)))


def choose_heading(velocities,current=0.):
    v=np.asarray(velocities,float)
    if v.ndim!=2 or v.shape[1]!=2 or not np.isfinite(v).all():raise ValueError('Finite Nx2 tangents required')
    v=v[np.linalg.norm(v,axis=1)>1e-6]
    if not len(v):return float(current),None
    candidates=current+np.linspace(-np.pi/2,np.pi/2,361)
    margins=np.array([steering_margin(v,h) for h in candidates]);best=margins.max()
    ids=np.flatnonzero(margins>=best-1e-10);i=ids[np.argmin(abs(candidates[ids]-current))]
    return float(candidates[i]),float(margins[i])


def world_to_body(vector,heading):
    c,s=np.cos(heading),np.sin(heading)
    return np.array([[c,s],[-s,c]])@np.asarray(vector,float)


def twist_steering_margin(velocities,heading,yaw_rates):
    """Worst module branch margin including rotational velocity at its offset."""
    from .swerve import OFFSETS
    v=np.asarray(velocities,float);rates=np.asarray(yaw_rates,float)
    if v.ndim!=2 or v.shape[1]!=2 or rates.ndim!=1 or not len(rates):
        raise ValueError('Need Nx2 velocities and nonempty yaw-rate scenarios')
    if not np.isfinite(v).all() or not np.isfinite(rates).all() or not np.isfinite(heading):
        raise ValueError('Nonfinite heading scenario')
    c,s=np.cos(heading),np.sin(heading)
    body=v@np.array([[c,-s],[s,c]])
    modules=body[:,None,None,:]+rates[None,:,None,None]*np.column_stack([-OFFSETS[:,1],OFFSETS[:,0]])[None,None,:,:]
    raw=np.arctan2(modules[...,1],modules[...,0])-BASE_DIRECTIONS
    angle=(raw+np.pi/2)%np.pi-np.pi/2
    margin=np.pi/2-np.abs(angle)
    # A stationary module has no preferred rolling direction.
    margin=np.where(np.linalg.norm(modules,axis=-1)>1e-6,margin,np.pi/2)
    return float(margin.min())


def choose_twist_heading(velocities,current=0.,yaw_rates=(-.2,0.,.2)):
    """Geometric proposal only: no collision/transition safety certification.

    Velocities must be m/s, not unscaled path-point differences. Scores include
    bounded yaw-rate scenarios; the actual transition still requires validation.
    """
    v=np.asarray(velocities,float)
    if v.ndim!=2 or v.shape[1]!=2 or not np.isfinite(v).all() or not np.isfinite(current):
        raise ValueError('Finite Nx2 velocities and current heading required')
    if not len(v) or np.max(np.linalg.norm(v,axis=1))<1e-6:return float(current),None
    candidates=current+np.linspace(-np.pi/2,np.pi/2,361)
    margins=_twist_margins_batch(v,candidates,yaw_rates)
    ids=np.flatnonzero(margins>=margins.max()-1e-10)
    i=ids[np.argmin(abs(candidates[ids]-current))]
    return float(candidates[i]),float(margins[i])


def heading_transition_margin(velocities,start,end,yaw_rates=(-.2,0.,.2)):
    """Sample geometric branch margin along a heading change, at <=0.5deg."""
    headings=np.linspace(start,end,max(2,int(np.ceil(abs(end-start)/np.deg2rad(.5)))+1))
    return min(twist_steering_margin(velocities,h,yaw_rates) for h in headings)


def choose_transition_heading(velocities,current=0.,yaw_rates=(-.2,0.,.2),min_margin=.06):
    """Restrict a geometric proposal to the current connected margin region.

    When initially below min_margin, require no further margin loss. Holding
    current heading is explicit fallback; this may stall and is not a mission
    planner. Occupancy and actual actuator dynamics still need validation.
    """
    v=np.asarray(velocities,float)
    if v.ndim!=2 or v.shape[1]!=2 or not np.isfinite(v).all() or not np.isfinite(current):
        raise ValueError('Finite Nx2 velocities and heading required')
    if not np.isfinite(min_margin) or not 0<=min_margin<=np.pi/2:raise ValueError('Invalid margin')
    if not len(v) or np.max(np.linalg.norm(v,axis=1))<1e-6:return float(current),None
    candidates=current+np.linspace(-np.pi/2,np.pi/2,361)
    margins=_twist_margins_batch(v,candidates,yaw_rates)
    middle=180;floor=min(min_margin,margins[middle]);valid=np.zeros(361,bool);valid[middle]=True
    for step in [-1,1]:
        for i in range(middle+step,361 if step==1 else -1,step):
            if margins[i]<floor-1e-10:break
            valid[i]=True
    best=margins[valid].max();ids=np.flatnonzero(valid&(margins>=best-1e-10))
    i=ids[np.argmin(abs(candidates[ids]-current))]
    # Recheck the selected interval using the exact validator sampling grid.
    if heading_transition_margin(v,current,candidates[i],yaw_rates)<floor-1e-10:
        return float(current),float(margins[middle])
    return float(candidates[i]),float(margins[i])


def _twist_margins_batch(velocities,headings,yaw_rates):
    from .swerve import OFFSETS
    v=np.unique(np.asarray(velocities,float),axis=0)
    rates=np.asarray(yaw_rates,float)
    if rates.ndim!=1 or not len(rates) or not np.isfinite(rates).all():raise ValueError('Invalid yaw scenarios')
    h=np.asarray(headings);c=np.cos(h);sn=np.sin(h)
    body=np.stack([c[:,None]*v[None,:,0]+sn[:,None]*v[None,:,1],-sn[:,None]*v[None,:,0]+c[:,None]*v[None,:,1]],axis=-1)
    modules=body[:,:,None,None,:]+rates[None,None,:,None,None]*np.column_stack([-OFFSETS[:,1],OFFSETS[:,0]])[None,None,None,:,:]
    raw=np.arctan2(modules[...,1],modules[...,0])-BASE_DIRECTIONS
    angle=(raw+np.pi/2)%np.pi-np.pi/2
    margin=np.where(np.linalg.norm(modules,axis=-1)>1e-6,np.pi/2-abs(angle),np.pi/2)
    return margin.min(axis=(1,2,3))
