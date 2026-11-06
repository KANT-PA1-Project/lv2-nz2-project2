"""Offline fixed-chassis-heading holonomic path candidate.

Build configuration-space cells with the exact existing padded CAD samples.
A fixed heading is deliberately independent of translation direction. The
caller must still apply live sensing, actuator limits, tracking-error reserves,
and swept-footprint veto; this candidate is not a runtime safety certificate.
"""
from heapq import heappop,heappush
import math
import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.spatial import ConvexHull
from swept_footprint import _samples,motion_clear


def configuration_obstacles(occupied, resolution, footprint, yaw, extra_center_margin=0.):
    if resolution<=0 or extra_center_margin<0 or not np.isfinite([resolution,yaw,extra_center_margin]).all():
        raise ValueError('Invalid resolution/heading/reserve')
    occupied=np.asarray(occupied,bool);samples=_samples(tuple(map(tuple,np.asarray(footprint,float))),float(resolution))
    c,s=np.cos(yaw),np.sin(yaw);points=samples@np.array([[c,s],[-s,c]])
    offsets=np.unique(np.floor(.5+points/resolution).astype(int),axis=0)
    # Outside the fixed known map is occupied, matching motion_clear.
    h,w=occupied.shape;out=np.zeros_like(occupied)
    for dx,dy in offsets:
        shifted=np.ones_like(occupied);x0,x1=max(0,-dx),min(w,w-dx);y0,y1=max(0,-dy),min(h,h-dy)
        if x0<x1 and y0<y1:shifted[y0:y1,x0:x1]=occupied[y0+dy:y1+dy,x0+dx:x1+dx]
        out|=shifted
    if extra_center_margin:
        out|=distance_transform_edt(~out)*resolution<=extra_center_margin
    return out


def _segment_clear(a,b,yaw,footprint,occupied,resolution):
    a,b=np.asarray(a,float),np.asarray(b,float);delta=b-a;c,s=np.cos(yaw),np.sin(yaw)
    body=np.array([[c,s],[-s,c]])@delta
    return motion_clear(a,yaw,[*body,0.],footprint,occupied,resolution,horizon=1.)


def plan_fixed_heading(start,goal,yaw,footprint,occupied,resolution,extra_center_margin=0.,allowed_centers=None):
    start,goal=np.asarray(start,float),np.asarray(goal,float)
    if extra_center_margin:
        # Minkowski sum with a square position-error bound. Every segment and
        # endpoint is checked using the expanded polygon, including shortcuts.
        corners=np.array([[-1,-1],[-1,1],[1,-1],[1,1]])*extra_center_margin
        expanded=(np.asarray(footprint)[:,None,:]+corners[None,:,:]).reshape(-1,2)
        footprint=expanded[ConvexHull(expanded).vertices]
    if not _segment_clear(start,start,yaw,footprint,occupied,resolution):return {'ok':False,'reason':'start_footprint_invalid'}
    if not _segment_clear(goal,goal,yaw,footprint,occupied,resolution):return {'ok':False,'reason':'goal_footprint_invalid'}
    blocked=configuration_obstacles(occupied,resolution,footprint,yaw,0.)
    if allowed_centers is not None:
        allowed_centers=np.asarray(allowed_centers,bool)
        if allowed_centers.shape!=blocked.shape:raise ValueError("Allowed-center map shape mismatch")
        blocked|=~allowed_centers
    # Do not snap through obstacles. Explicitly validate endpoint attachments.
    def endpoint(q):
        xy=np.floor(q/resolution).astype(int);choices=[]
        for dx in range(-2,3):
            for dy in range(-2,3):
                v=xy+[dx,dy];p=(v+.5)*resolution
                if (v>=0).all() and v[0]<blocked.shape[1] and v[1]<blocked.shape[0] and not blocked[v[1],v[0]] and _segment_clear(q,p,yaw,footprint,occupied,resolution):
                    choices.append((float(np.linalg.norm(q-p)),tuple(v)))
        return min(choices)[1] if choices else None
    s,t=endpoint(start),endpoint(goal)
    if s is None or t is None:return {'ok':False,'reason':'no_clear_endpoint_attachment'}
    clear=distance_transform_edt(~blocked)*resolution
    opened=[(math.dist(s,t),0.,s)];cost={s:0.};previous={};closed=set();expansions=0
    while opened:
        _,g,at=heappop(opened)
        if at in closed:continue
        closed.add(at);expansions+=1
        if at==t:break
        x,y=at
        for dx,dy in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
            q=(x+dx,y+dy)
            if not (0<=q[0]<blocked.shape[1] and 0<=q[1]<blocked.shape[0]) or blocked[q[1],q[0]]:continue
            if dx and dy and (blocked[y,q[0]] or blocked[q[1],x]):continue
            # Prefer actual free-space reserve while permitting narrow passages.
            ng=g+math.hypot(dx,dy)*(1.+.05/max(.01,float(clear[q[1],q[0]])))
            if ng<cost.get(q,float('inf')):
                cost[q]=ng;previous[q]=at;heappush(opened,(ng+math.dist(q,t),ng,q))
    if t not in closed:return {'ok':False,'reason':'heading_has_no_path','expanded_cells':expansions}
    cells=[t]
    while cells[-1]!=s:cells.append(previous[cells[-1]])
    points=np.vstack([start,(np.array(cells[::-1])+.5)*resolution,goal]);keep=[points[0]];i=0
    # Shortcut only after a complete swept check at the selected chassis yaw.
    while i<len(points)-1:
        j=len(points)-1
        while j>i+1 and not _segment_clear(points[i],points[j],yaw,footprint,occupied,resolution):j-=1
        if not _segment_clear(points[i],points[j],yaw,footprint,occupied,resolution):return {'ok':False,'reason':'edge_continuous_validation_failed'}
        keep.append(points[j]);i=j
    length=float(np.linalg.norm(np.diff(np.asarray(keep),axis=0),axis=1).sum())
    return {'ok':True,'yaw_rad':float(yaw),'path':np.asarray(keep).tolist(),'length_m':length,'expanded_cells':expansions,'extra_center_margin_m':extra_center_margin,'continuous_swept_segments_checked':len(keep)-1}
