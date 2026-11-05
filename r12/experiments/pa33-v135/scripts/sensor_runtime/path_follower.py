"""Select a visible forward path target using map and live lidar constraints."""
import numpy as np
import os
from arm_practice.lidar_guard import constrain_velocity
from swept_footprint import motion_clear

def follow(path,xy,yaw,goal,clearance,resolution,speed,scans,extrinsics,footprint=None,occupied=None,braking_stop_distance_m=.01):
    if not np.isfinite(braking_stop_distance_m) or not 0<=braking_stop_distance_m<=.01:
        raise ValueError('Invalid braking endpoint distance')
    path=np.asarray(path,float);xy=np.asarray(xy,float);goal=np.asarray(goal,float)
    near=int(np.argmin(np.linalg.norm(path-xy,axis=1)))
    far=min(near+max(15,int((speed*1.5+.25)/resolution)),len(path)-1)
    distance=np.linalg.norm(goal-xy)
    cap=min(speed,.8*distance,np.sqrt(2*.15*max(0,distance-braking_stop_distance_m)))
    # Anticipate lateral acceleration through bends, without inflating walls.
    arc=0.
    for j in range(max(1,near),len(path)-1):
        arc+=np.linalg.norm(path[j]-path[j-1])
        if arc>max(.6,speed*2):break
        u=path[j]-path[j-1];v=path[j+1]-path[j]
        area=abs(u[0]*v[1]-u[1]*v[0])
        if area>1e-9:
            radius=np.linalg.norm(u)*np.linalg.norm(v)*np.linalg.norm(u+v)/(2*area)
            bend_speed=np.sqrt(.15*radius)
            cap=min(cap,np.sqrt(bend_speed*bend_speed+2*.15*max(0,arc-.15)))
    c,s=np.cos(yaw),np.sin(yaw);world_to_body=np.array([[c,s],[-s,c]])
    heading_goal=float(os.getenv('R12_DIAGNOSTIC_HEADING_DEG','180'))*np.pi/180
    if not np.isfinite(heading_goal):raise ValueError('Invalid diagnostic heading')
    if os.getenv('R12_LOOKAHEAD_HEADING','0')=='1':
        from arm_practice.heading_feasibility import choose_heading
        end=near+1;lookahead_arc=0.
        while end<len(path)-1 and lookahead_arc<.6:
            lookahead_arc+=float(np.linalg.norm(path[end]-path[end-1]));end+=1
        tangents=np.diff(path[near:end+1],axis=0)
        if len(tangents):
            if os.getenv('R12_TRANSITION_HEADING','0')=='1':
                from arm_practice.heading_feasibility import choose_transition_heading
                lengths=np.linalg.norm(tangents,axis=1)
                valid=lengths>1e-6
                velocities=tangents[valid]/lengths[valid,None]*cap
                heading_goal,_=choose_transition_heading(velocities,current=yaw)
            else:heading_goal,_=choose_heading(tangents,current=yaw)
    turn=float(np.clip(-.6*((yaw-heading_goal+np.pi)%(2*np.pi)-np.pi),-.2,.2))
    offsets=None
    if footprint is not None:
        from scipy.spatial import ConvexHull
        vertices=np.asarray(footprint);hull=ConvexHull(vertices)
        gx,gy=np.meshgrid(np.arange(vertices[:,0].min(),vertices[:,0].max()+resolution*.5,resolution*.5),np.arange(vertices[:,1].min(),vertices[:,1].max()+resolution*.5,resolution*.5))
        samples=np.column_stack([gx.ravel(),gy.ravel()]);samples=samples[(samples@hull.equations[:,:2].T+hull.equations[:,2]<=1e-9).all(axis=1)]
        offsets=np.vstack([samples,vertices])@world_to_body
    last_reason='no_visible_forward_target'
    best=None;best_score=-1.
    # The nearest point may already be the endpoint before docking is complete.
    lower=near-1 if near==len(path)-1 else near
    for index in range(far,lower,-1):
        delta=path[index]-xy
        # At most half a grid cell between checks; thin walls cannot be skipped.
        cells=np.floor(np.linspace(xy,path[index],max(2,int(np.ceil(np.linalg.norm(delta)/(resolution*.5)))+1))/resolution).astype(int)
        inside=(cells[:,0]>=0)&(cells[:,0]<clearance.shape[1])&(cells[:,1]>=0)&(cells[:,1]<clearance.shape[0])
        if not inside.all():continue
        if offsets is None:
            if np.min(clearance[cells[:,1],cells[:,0]])<.30:continue
        else:
            if occupied is None:raise ValueError('Occupancy required for polygon footprint')
            centers=np.linspace(xy,path[index],max(2,int(np.ceil(np.linalg.norm(delta)/(resolution*.5)))+1))
            swept=np.floor((centers[:,None,:]+offsets[None,:,:])/resolution).astype(int).reshape(-1,2)
            inside=(swept[:,0]>=0)&(swept[:,0]<occupied.shape[1])&(swept[:,1]>=0)&(swept[:,1]<occupied.shape[0])
            if not inside.all() or occupied[swept[:,1],swept[:,0]].any():continue
        velocity=delta*min(cap/max(np.linalg.norm(delta),1e-9),1.1)
        desired=np.r_[world_to_body@velocity,turn]
        constrained,reason=constrain_velocity(desired,scans,extrinsics,footprint=footprint)
        if reason=='invalid_sensor':return constrained,reason,{'path_index':index}
        # A candidate that cannot replace the already validated winner needs
        # no additional SE2 sweep. Every returned motion is still validated.
        candidate_score=np.linalg.norm(constrained[:2])*(1+.02*(index-near)/max(1,far-near))
        if best is not None and candidate_score<=best_score:continue
        if footprint is not None and not motion_clear(xy,yaw,constrained,footprint,occupied,resolution):
            last_reason='rotation_sweep_blocked';continue
        # A blocked distant target must not hide a reachable nearer path point.
        if np.linalg.norm(constrained[:2])>1e-4 and np.dot(constrained[:2],desired[:2])>0:
            # Prefer an achievable direction over a distant corner-cut target
            # that lidar reduces to a crawl. Distance only breaks near ties.
            score=np.linalg.norm(constrained[:2])*(1+.02*(index-near)/max(1,far-near))
            if score>best_score:
                best_score=score;best=(constrained,reason,{'path_index':index,'target_xy':path[index].tolist(),'near_index':near,'target_score':float(score),'heading_goal_rad':float(heading_goal),'yaw_rad':float(yaw),'speed_cap_m_s':float(cap),'candidate_desired_twist':desired.tolist(),'candidate_guarded_twist':constrained.tolist()})
        last_reason='lidar_blocked_target'
    if best is not None:return best
    fallback=np.array([0.,0.,turn])
    if footprint is not None and abs(turn)>1e-9 and not motion_clear(xy,yaw,fallback,footprint,occupied,resolution):
        fallback[:]=0;last_reason='rotation_sweep_blocked'
    return fallback,last_reason,{'near_index':near,'far_index':far,'heading_goal_rad':float(heading_goal),'yaw_rad':float(yaw),'speed_cap_m_s':float(cap)}
