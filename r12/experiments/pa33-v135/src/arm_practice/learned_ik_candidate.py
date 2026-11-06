"""Prepare an endpoint candidate from fresh, same-home observations; never command it."""
import numpy as np
from .frame_packet_identity import check_live_frame_provenance
from .ik_start_state import check_start_state
from .learned_ik import finish

def prepare_candidate(model,target,frame,packet,*,frame_index,run_id,now_ns,base_twist,base_received_ns,margin_rad=.025):
    def reject(reason,**extra):return dict(accepted=False,reason=reason,execution_authorized=False,**extra)
    provenance=check_live_frame_provenance(frame,packet,frame_index,run_id,now_ns,250_000_000)
    if not provenance['matched']:return reject(provenance['reason'],provenance=provenance)
    if type(base_received_ns) is not int or not 0<=now_ns-base_received_ns<=250_000_000:
        return reject('missing_stale_or_future_base_feedback')
    d=model.d
    start=check_start_state(d['home'],packet.get('arm_measured_position',[]),packet.get('arm_measured_velocity',[]),now=now_ns/1e9,state_stamp=packet['frame_capture_monotonic_ns']/1e9,base_twist=base_twist,lower=d['lower'],upper=d['upper'])
    if not start['eligible_for_path_validation']:return reject(start['reason'],start_state=start)
    if not np.isfinite(margin_rad) or margin_rad<0:return reject('invalid_margin')
    lo=np.asarray(d['lower'])+margin_rad;hi=np.asarray(d['upper'])-margin_rad
    if np.any(lo[:4]>=hi[:4]):return reject('invalid_margin')
    solution=model.solve(target)
    if not solution['accepted']:return reject(solution['reason'])
    q,error,n=finish(model.fk,np.asarray(target),np.asarray(solution['q']),lo,hi,d['home'])
    if error>d['tolerance_mm']:return reject('interior_endpoint_error',error_mm=error)
    return dict(accepted=True,reason='requires_path_and_environment_validation',execution_authorized=False,trajectory_validated=False,q=q.tolist(),error_mm=error,solver_nfev=n,proposal_error_mm=solution['proposal_error_mm'],margin_rad=margin_rad,start_state=start,provenance=provenance)

def prepare_from_sensor_packet(model,target,frame,packet,*,frame_index,run_id,now_ns):
    """Use named sensor-derived feedback, never the command or stopped Boolean."""
    base=packet.get('base_sensor_feedback',{})
    if (not base.get('valid') or base.get('source')!='wheel_encoder_xy_dual_imu_yaw'
        or base.get('run_id')!=run_id or base.get('stamp_ns')!=packet.get('stamp_ns')
        or base.get('capture_monotonic_ns')!=packet.get('frame_capture_monotonic_ns')):
        return dict(accepted=False,reason='invalid_base_sensor_provenance',execution_authorized=False)
    return prepare_candidate(model,target,frame,packet,frame_index=frame_index,run_id=run_id,now_ns=now_ns,base_twist=base.get('twist'),base_received_ns=base.get('capture_monotonic_ns'))
