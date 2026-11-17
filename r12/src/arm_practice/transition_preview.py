"""Experimental reference-only preview; not a physics model or runtime controller.

Input velocities are live planned WORLD-frame vectors, not future truth. Each
candidate yaw rate produces a finite-rate steering/wheel reference trajectory.
Scores compare ideal wheel rolling vectors with requested rigid-body vectors.
Contact forces, slip, sensing delay and collision checks remain external.
"""
import numpy as np
from .swerve import OFFSETS,BASE_DIRECTIONS,RADIUS


def preview(world_velocities,yaw,steering,wheel_reference,yaw_rate=0.,dt=.05,reference_model="ideal_projection"):
    if reference_model not in ("ideal_projection","runtime_allocator"):raise ValueError("Unknown reference model")
    velocities=np.asarray(world_velocities,float)
    initial=np.r_[yaw,steering,wheel_reference,yaw_rate,dt]
    if velocities.ndim!=2 or velocities.shape[1]!=2 or not len(velocities):raise ValueError('Need nonempty Nx2 path velocities')
    if np.asarray(steering).shape!=(4,) or np.asarray(wheel_reference).shape!=(4,):raise ValueError('Need four module states')
    if not np.isfinite(velocities).all() or not np.isfinite(initial).all() or not 0<dt<=.1:raise ValueError('Invalid preview input')
    if np.max(np.linalg.norm(velocities,axis=1))>.899795667+1e-9:raise ValueError('Path speed exceeds ceiling')
    if np.max(abs(np.asarray(steering)))>np.pi/2+1e-9 or np.max(abs(np.asarray(wheel_reference)))>29.321531434+1e-9 or abs(yaw_rate)>.2+1e-9:raise ValueError('Initial reference outside bounds')
    candidates=[]
    for goal_rate in [-.2,-.1,0.,.1,.2]:
        angle=np.asarray(steering,float).copy();wheels=np.asarray(wheel_reference,float).copy();heading=float(yaw);rate=float(yaw_rate);trace=[]
        for world in velocities:
            rate+=float(np.clip(goal_rate-rate,-.4*dt,.4*dt))
            c,s=np.cos(heading),np.sin(heading);body=np.array([[c,s],[-s,c]])@world
            vectors=body+rate*np.column_stack([-OFFSETS[:,1],OFFSETS[:,0]])
            if np.max(np.linalg.norm(vectors,axis=1))>.899795667+1e-9:
                trace=[];break  # Reject instead of silently scaling a planned path.
            raw=(np.arctan2(vectors[:,1],vectors[:,0])-BASE_DIRECTIONS+np.pi)%(2*np.pi)-np.pi
            target=np.where(raw>np.pi/2,raw-np.pi,np.where(raw< -np.pi/2,raw+np.pi,raw))
            hold=(abs(abs(raw)-np.pi/2)<.06)&(abs(angle)>.7)
            target=np.where(hold,np.sign(angle)*np.pi/2,target)
            stationary=np.linalg.norm(vectors,axis=1)<1e-9;target=np.where(stationary,angle,target)
            duration=float(np.max(abs(target-angle))/7.330382858)
            old_angle=angle.copy();old_wheels=wheels.copy()
            angle+=np.clip(target-angle,-7.330382858*dt,7.330382858*dt)
            axes=np.column_stack([np.cos(BASE_DIRECTIONS+angle),np.sin(BASE_DIRECTIONS+angle)])
            demand=np.sum(vectors*axes,axis=1)/RADIUS
            wheels+=np.clip(demand-wheels,-24.45*dt,24.45*dt)
            if reference_model=="runtime_allocator" and np.linalg.norm(np.r_[body,rate])>=1e-6:
                from .transition_reference import moving_reference
                # Ideal tracking assumption only; this is NOT a simulated encoder.
                angle,wheels,_=moving_reference(np.r_[body,rate],old_angle,old_wheels,old_angle,old_wheels,dt)
                axes=np.column_stack([np.cos(BASE_DIRECTIONS+angle),np.sin(BASE_DIRECTIONS+angle)])
            mismatch=float(np.sqrt(np.mean((RADIUS*wheels[:,None]*axes-vectors)**2)))
            trace.append({'requested_body_twist':np.r_[body,rate].tolist(),'yaw_rad':heading,'yaw_rate_rad_s':rate,'steer_reference_rad':angle.tolist(),'wheel_reference_rad_s':wheels.tolist(),'remaining_steer_time_before_step_s':duration,'rolling_vector_rmse_m_s':mismatch})
            heading+=rate*dt
        if trace:
            candidates.append({'reference_model':reference_model,'goal_yaw_rate_rad_s':goal_rate,'score':float(np.mean([x['rolling_vector_rmse_m_s']**2 for x in trace])),'trace':trace})
    return sorted(candidates,key=lambda x:(x['score'],abs(x['goal_yaw_rate_rad_s'])))
