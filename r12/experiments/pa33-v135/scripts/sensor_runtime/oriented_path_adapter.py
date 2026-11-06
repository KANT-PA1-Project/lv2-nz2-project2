"""Pure candidate adapter: sensor pose + current planned path + occupancy.

No station coordinates, fixture identities, simulator pose, clocks or ROS
state are read. Returned geometry is an offline planning candidate until a
caller supplies fresh sensors and retains all existing live control vetoes.
"""
import math
import numpy as np
from scipy.spatial import ConvexHull,cKDTree
from heading_corridor import plan_fixed_heading,_segment_clear
from swept_footprint import motion_clear
from path_smoothing import smooth_path


def wrap(x):return (x+np.pi)%(2*np.pi)-np.pi


def uncertainty_footprint(footprint,position_margin_m=.01,yaw_error_rad=np.deg2rad(2.)):
    """Conservative polygon for a bounded position and yaw tracking error.

    Rotated vertices are sampled at <=.5deg. The arc/chord sagitta is added
    to the position margin so interpolation between angle samples is covered.
    The square positional margin contains an isotropic circle of that radius.
    It augments the caller's existing CAD padding; none is removed.
    """
    foot=np.asarray(footprint,float)
    if foot.ndim!=2 or foot.shape[1]!=2 or len(foot)<3 or not np.isfinite(foot).all():raise ValueError('Finite convex footprint required')
    if not np.isfinite([position_margin_m,yaw_error_rad]).all() or position_margin_m<0 or not 0<=yaw_error_rad<=np.pi/4:raise ValueError('Invalid uncertainty bound')
    count=max(2,int(np.ceil(2*yaw_error_rad/np.deg2rad(.5)))+1)
    angles=np.linspace(-yaw_error_rad,yaw_error_rad,count);cloud=[]
    for a in angles:
        c,s=np.cos(a),np.sin(a);cloud.append(foot@np.array([[c,s],[-s,c]]))
    gap=float(np.max(np.diff(angles)));sagitta=float(np.linalg.norm(foot,axis=1).max()*(1-np.cos(gap/2)))
    margin=position_margin_m+sagitta;corners=np.array([[-margin,-margin],[-margin,margin],[margin,-margin],[margin,margin]])
    pts=(np.vstack(cloud)[:,None,:]+corners[None,:,:]).reshape(-1,2)
    return pts[ConvexHull(pts).vertices]


def valid_polyline(path,yaw,footprint,occupied,resolution):
    p=np.asarray(path,float)
    return bool(p.ndim==2 and p.shape[1]==2 and len(p)>=2 and np.isfinite(p).all() and all(_segment_clear(a,b,yaw,footprint,occupied,resolution) for a,b in zip(p[:-1],p[1:])))


def _path_corridor(path,shape,resolution,width):
    """Limit the search to the supplied planner path's neighborhood."""
    dense=[]
    for a,b in zip(path[:-1],path[1:]):dense.extend(np.linspace(a,b,max(2,int(np.ceil(np.linalg.norm(b-a)/(.5*resolution)))+1)))
    yy,xx=np.indices(shape);xy=np.column_stack(((xx.ravel()+.5)*resolution,(yy.ravel()+.5)*resolution))
    return (cKDTree(np.asarray(dense)).query(xy)[0]<=width).reshape(shape)


def steering_margin_deg(path,yaw):
    delta=np.diff(np.asarray(path,float),axis=0);delta=delta[np.linalg.norm(delta,axis=1)>1e-8]
    travel=np.arctan2(delta[:,1],delta[:,0]);raw=wrap(travel[:,None]-yaw-np.array([-np.pi/2,-np.pi/2,0.,np.pi]))
    angles=np.where(raw>np.pi/2,raw-np.pi,np.where(raw<-np.pi/2,raw+np.pi,raw))
    return float(np.rad2deg(np.pi/2-np.max(abs(angles)))) if len(delta) else 0.


def plan_oriented_path(current_path,sensor_pose,footprint,occupied,clearance,resolution,*,position_margin_m=.01,yaw_error_rad=np.deg2rad(2.),corridor_width_m=.20,heading_step_deg=5.,max_heading_change_rad=np.pi/2):
    """Choose a safe chassis heading and smooth a path to the same goal.

    Candidates are generated from the measured heading and an angular lattice,
    not from known station labels. Occupancy must use the same map frame as the
    sensor pose. Any freshness and map-version checks belong to the caller.
    This may take seconds: use an asynchronous worker and reject stale results.
    """
    path=np.asarray(current_path,float);pose=np.asarray(sensor_pose,float)
    if path.ndim!=2 or path.shape[1]!=2 or len(path)<2 or not np.isfinite(path).all() or pose.shape!=(3,) or not np.isfinite(pose).all():raise ValueError('Finite path and measured pose required')
    if not (0<heading_step_deg<=45 and 0<corridor_width_m<=1 and 0<max_heading_change_rad<=np.pi):raise ValueError('Invalid bounded search')
    foot=uncertainty_footprint(footprint,position_margin_m,yaw_error_rad)
    if not _segment_clear(pose[:2],pose[:2],pose[2],foot,occupied,resolution):return {'ok':False,'reason':'current_pose_margin_invalid','motion_authorized':False}
    # Retain the input path's geometric corridor and exact endpoint.
    path=np.vstack([pose[:2],path]) if np.linalg.norm(path[0]-pose[:2])>1e-7 else path
    allowed=_path_corridor(path,np.asarray(occupied).shape,resolution,corridor_width_m)
    candidates=np.unique(np.r_[pose[2],np.deg2rad(np.arange(-180,180,heading_step_deg))]);candidates=sorted(candidates,key=lambda h:abs(wrap(h-pose[2])))
    choices=[];rejected=[]
    for yaw in candidates:
        angle=float(wrap(yaw-pose[2]))
        if abs(angle)>max_heading_change_rad+1e-9:continue
        if not motion_clear(pose[:2],pose[2],[0,0,angle],foot,occupied,resolution,horizon=1.):
            rejected.append({'yaw_rad':float(yaw),'reason':'preparation_rotation_blocked'});continue
        plan=plan_fixed_heading(pose[:2],path[-1],float(yaw),foot,occupied,resolution,allowed_centers=allowed)
        if not plan['ok']:
            rejected.append({'yaw_rad':float(yaw),'reason':plan['reason']});continue
        # Use explicit selected yaw, then recheck every segment with uncertainty.
        smoothed=smooth_path(plan['path'],occupied,clearance,resolution,foot,yaw=float(yaw))
        if not valid_polyline(smoothed,yaw,foot,occupied,resolution):
            rejected.append({'yaw_rad':float(yaw),'reason':'smoothed_sweep_invalid'});continue
        dense=[]
        for a,b in zip(smoothed[:-1],smoothed[1:]):dense.extend(np.linspace(a,b,max(2,int(np.ceil(np.linalg.norm(np.asarray(b)-a)/(.5*resolution)))+1)))
        cells=np.floor(np.asarray(dense)/resolution).astype(int)
        if not allowed[cells[:,1],cells[:,0]].all():
            rejected.append({'yaw_rad':float(yaw),'reason':'smoothed_path_left_input_corridor'});continue
        length=float(np.linalg.norm(np.diff(smoothed,axis=0),axis=1).sum());branch=steering_margin_deg(smoothed,yaw)
        # A preference only. A safe but narrow steering margin remains visible.
        score=length+.10*abs(angle)+.025/max(.5,branch)
        choices.append({'yaw_rad':float(yaw),'path':smoothed,'length_m':length,'heading_change_rad':angle,'steering_endstop_margin_deg':branch,'score':score})
    if not choices:return {'ok':False,'reason':'no_heading_corridor','motion_authorized':False,'rejected':rejected}
    best=min(choices,key=lambda c:c['score'])
    return dict(best,ok=True,reason='candidate_ready',motion_authorized=False,position_margin_m=position_margin_m,yaw_error_rad=float(yaw_error_rad),robust_footprint=foot.tolist(),preparation_origin=pose[:2].tolist(),goal=path[-1].tolist(),rejected=rejected,candidate_count=len(choices),scope='Candidate geometry only; live lidar, sensors, torque and swept veto remain required')


def heading_gate(plan,sensor_pose,measured_twist,footprint,occupied,resolution,*,phase,heading_ready_rad=np.deg2rad(.5),stop_linear_m_s=.005,stop_yaw_rad_s=.01,max_yaw_rate=.20,observation_valid=True,transport_settled=True):
    """Pure state transition for an existing stopped→heading→translate cycle.

    Returns desired yaw rate or a translation permission, never wheel actions.
    The caller must apply the existing envelope/guards to *every* proposal.
    During tracking, leaving the validated yaw band stops and requests replan;
    it never reorients inside an uncertified narrow corridor.
    """
    if phase not in ('await_stop','prepare_heading','tracking'):raise ValueError('Unknown heading phase')
    pose=np.asarray(sensor_pose,float);twist=np.asarray(measured_twist,float)
    stopped={'phase':phase,'allow_translation':False,'desired_yaw_rate':0.,'replan':False}
    if not plan.get('ok') or pose.shape!=(3,) or twist.shape!=(3,) or not np.isfinite(np.r_[pose,twist]).all():return dict(stopped,reason='invalid_plan_or_feedback')
    if not observation_valid or not transport_settled:return dict(stopped,reason='freshness_or_transport_gate')
    foot=np.asarray(plan['robust_footprint']);yaw=float(plan['yaw_rad']);error=float(wrap(yaw-pose[2]))
    checked_yaw=yaw if phase=='tracking' else pose[2]
    if not _segment_clear(pose[:2],pose[:2],checked_yaw,foot,occupied,resolution):return dict(stopped,reason='current_footprint_margin_invalid',replan=True)
    if phase=='tracking':
        if abs(error)>plan['yaw_error_rad']:return dict(stopped,reason='validated_heading_band_exceeded',replan=True)
        return dict(stopped,allow_translation=True,desired_yaw_rate=float(np.clip(.65*error,-max_yaw_rate,max_yaw_rate)),reason='tracking_within_heading_band')
    if np.linalg.norm(pose[:2]-np.asarray(plan['preparation_origin']))>max(.01,plan['position_margin_m']):return dict(stopped,reason='preparation_pose_drift',replan=True)
    if np.linalg.norm(twist[:2])>stop_linear_m_s:return dict(stopped,phase='await_stop',reason='base_not_stopped')
    if not motion_clear(pose[:2],pose[2],[0,0,error],foot,occupied,resolution,horizon=1.):return dict(stopped,reason='preparation_rotation_blocked',replan=True)
    if abs(error)<=heading_ready_rad and abs(twist[2])<=stop_yaw_rad_s:return dict(stopped,phase='tracking',allow_translation=True,reason='heading_preparation_complete')
    return dict(stopped,phase='prepare_heading',desired_yaw_rate=float(np.clip(.65*error,-max_yaw_rate,max_yaw_rate)),reason='prepare_heading_at_stop')
