"""Aligned interval motion audit; simulator pose is used only for evaluation."""
import argparse,json
from pathlib import Path
import numpy as np
from arm_practice.swerve import wheel_odometry


def audit(run):
    rows=json.loads((run/'sim_results.json').read_text())
    cmd=np.asarray([r['command'] for r in rows])
    xyz=np.asarray([r['actual_base'] for r in rows])
    yaw=np.unwrap(np.radians([r['actual_rpy_deg'][2] for r in rows]))
    dt=np.diff([r['stamp_ns'] for r in rows])/1e9
    delta=np.diff(xyz[:,:2],axis=0)/dt[:,None]
    middle=(yaw[1:]+yaw[:-1])/2
    actual=np.column_stack([np.cos(middle)*delta[:,0]+np.sin(middle)*delta[:,1],
                            -np.sin(middle)*delta[:,0]+np.cos(middle)*delta[:,1],np.diff(yaw)/dt])
    speed=np.linalg.norm(cmd[:,:2],axis=1)
    moving=speed[:-1]>.05
    error=actual-cmd[:-1]
    def metric(mask):
        if not mask.any():return {'samples':0}
        a=actual[mask];c=cmd[:-1][mask];e=error[mask]
        angle=np.abs(np.arctan2(a[:,0]*c[:,1]-a[:,1]*c[:,0],np.sum(a[:,:2]*c[:,:2],axis=1)))
        return {'samples':int(mask.sum()),'xy_rmse_m_s':float(np.sqrt(np.mean(np.sum(e[:,:2]**2,axis=1)))),
                'yaw_rmse_rad_s':float(np.sqrt(np.mean(e[:,2]**2))),
                'direction_error_deg_p95':float(np.percentile(np.degrees(angle),95)),
                'actual_speed_peak_m_s':float(np.linalg.norm(a[:,:2],axis=1).max())}
    steer=np.asarray([r['drive_telemetry']['steer_position_rad'] for r in rows])
    rates=np.max(abs(np.diff(steer,axis=0))/dt[:,None],axis=1)
    sharp=rates>2
    # Three command stages and physical response use different timestamps;
    # actual interval i->i+1 is paired with command i, never command i+1.
    drops=[{'frame':i,'video_seconds':i*.05,'previous_m_s':float(speed[i-1]),
            'lidar_guard':rows[i].get('lidar_guard'),'sensor_guard':rows[i]['motion_envelope']['reason']}
           for i in range(1,len(rows)) if speed[i]<1e-5 and speed[i-1]>.05]
    residuals=[]
    for r in rows:
        t=r['drive_telemetry'];_,res=wheel_odometry(t['steer_position_rad'],t['wheel_velocity_rad_s']);residuals.append(res)
    preview=[r.get('path_follower') or {} for r in rows]
    sideways=moving&(np.abs(cmd[:-1,1])>.03)
    actual_speed=np.linalg.norm(actual[:,:2],axis=1)
    slow_response=moving&(actual_speed<np.maximum(.01,.2*speed[:-1]))
    command_acceleration=np.diff(cmd[:,:2],axis=0)/dt[:,None]
    command_changes=np.linalg.norm(command_acceleration,axis=1)
    def events(mask):
        indices=np.flatnonzero(mask)
        return [{'start_frame':int(g[0]),'end_frame':int(g[-1]),'duration_s':float(dt[g].sum())}
                for g in np.split(indices,np.where(np.diff(indices)>1)[0]+1) if len(g)]
    stop_times=[i for i,r in enumerate(rows) if r['motion_envelope']['reason']=='invalid_sensor_or_time']
    sensor_ok=np.ones(len(dt),bool)
    for i in stop_times:sensor_ok[max(0,i-1):min(len(dt),i+10)]=False
    smooth_motion=moving&sensor_ok
    pure_turn=(speed[:-1]<.005)&(np.abs(cmd[:-1,2])>.03)
    route_active=np.array([bool(p and p.get('remaining_m',0)>.3) for p in preview[:-1]])
    active_twist=(speed[:-1]>.005)|(np.abs(cmd[:-1,2])>.03)
    report={'run':run.name,'frames':len(rows),'duration_sim_s':len(rows)*.05,
        'scope':'Simulator pose for evaluation only; RMSE includes commanded moving intervals, not the full fixed-direction qualification suite.',
        'moving':metric(moving),'fast_steering':metric(moving&sharp),'other_steering':metric(moving&~sharp),
        'fast_steering_frames':int((moving&sharp).sum()),'hard_command_drops':drops,
        'encoder_rolling_residual_p95_m_s':float(np.percentile(np.asarray(residuals)[:-1][moving],95)) if moving.any() else None,
        'complete':rows[-1].get('finished'), 'completed':rows[-1].get('completed'),
        'contacts':len(json.loads((run/'contacts.json').read_text())) if (run/'contacts.json').exists() else None,
        'max_abs_roll_pitch_deg':float(np.max(abs(np.asarray([r['actual_rpy_deg'][:2] for r in rows])))),
        'preview_veto_frames':sum(p.get('preview_reason')=='no_filtered_rollout_clear' for p in preview),
        'imu_invalid_frames_after_departure':sum(not r['imu_fused'] and r['transport_posture_ok'] for r in rows),
        'visual_missing_frames_after_departure':sum((not r['odom_received'] or r.get('tracking')!=1) and r['transport_posture_ok'] for r in rows),
        'lateral_command_intervals':metric(sideways),
        'near_stall_despite_nonzero_command':events(slow_response),
        'fast_steering_events':events(moving&sharp),
        'all_active_twist_fast_steering_events':events(active_twist&sharp),
        'turn_in_place_during_continuous_route':events(pure_turn&route_active),
        'all_active_twist':metric(active_twist),
        'moving_without_sensor_stop_recovery':metric(smooth_motion),
        'command_vector_acceleration_m_s2':{'p95':float(np.percentile(command_changes[smooth_motion],95)),'peak':float(np.max(command_changes[smooth_motion]))} if smooth_motion.any() else None,
        'post_filter_lidar_adjustments':int(sum(np.linalg.norm(np.asarray(r['command_stages']['after_envelope'])-np.asarray(r['command_stages']['after_lidar']))>1e-6 for r in rows)),
        'performance_accepted':False}
    (run/'holonomic_motion_audit.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('runs',nargs='+',type=Path);a=p.parse_args()
    print(json.dumps([audit(r) for r in a.runs],indent=2))
