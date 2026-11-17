"""Offline sampled occupancy/progress screen for reference-preview candidates.

Checks both requested-twist and ideal rolling-reference paths. Neither is a
physical prediction. Live lidar veto and actual control safety remain mandatory.
"""
import numpy as np
from arm_practice.swerve import wheel_odometry
from swept_footprint import motion_clear


def advance(xy,yaw,twist,dt):
    vx,vy,w=twist;a=w*dt;c,s=np.cos(yaw),np.sin(yaw)
    local=np.array([vx,vy])*dt if abs(w)<1e-9 else np.array([np.sin(a)*vx-(1-np.cos(a))*vy,(1-np.cos(a))*vx+np.sin(a)*vy])/w
    return np.asarray(xy)+np.array([[c,-s],[s,c]])@local,yaw+a


def screen(candidates,xy,yaw,goal,footprint,occupied,resolution,dt=.05):
    xy=np.asarray(xy,float);goal=np.asarray(goal,float)
    if xy.shape!=(2,) or goal.shape!=(2,) or not np.isfinite(np.r_[xy,goal,yaw,dt]).all() or not 0<dt<=.1:raise ValueError('Invalid geometry state')
    results=[]
    for candidate in candidates:
        positions=[xy.copy(),xy.copy()];headings=[float(yaw),float(yaw)];blocked=None;lengths=[0.,0.]
        for step,row in enumerate(candidate['trace']):
            rolling,_=wheel_odometry(row['steer_reference_rad'],row['wheel_reference_rad_s'])
            for mode,twist in enumerate([np.asarray(row['requested_body_twist']),rolling]):
                if not motion_clear(positions[mode],headings[mode],twist,footprint,occupied,resolution,horizon=dt):
                    blocked={'step':step,'path_model':['requested','ideal_rolling'][mode]};break
                nxt,heading=advance(positions[mode],headings[mode],twist,dt)
                lengths[mode]+=float(np.linalg.norm(nxt-positions[mode]));positions[mode]=nxt;headings[mode]=heading
            if blocked is not None:break
        initial=float(np.linalg.norm(goal-xy))
        results.append({'goal_yaw_rate_rad_s':candidate['goal_yaw_rate_rad_s'],'reference_score':candidate['score'],'sampled_clear':blocked is None,'blocked':blocked,
                        'requested_goal_progress_m':initial-float(np.linalg.norm(goal-positions[0])),
                        'ideal_rolling_goal_progress_m':initial-float(np.linalg.norm(goal-positions[1])),
                        'path_lengths_m':lengths,'end_xy':[p.tolist() for p in positions]})
    admissible=[r for r in results if r['sampled_clear'] and r['ideal_rolling_goal_progress_m']>0]
    return {'candidates':results,'admissible_count':len(admissible),'selected':min(admissible,key=lambda r:r['reference_score']) if admissible else None,
            'scope':'Sampled configured occupancy only; positive endpoint distance reduction is a preliminary progress gate, not route/corridor tracking. Command and ideal rolling paths do not cover true transient motion. No braking-distance, PI, torque, slip or live lidar certification.'}
