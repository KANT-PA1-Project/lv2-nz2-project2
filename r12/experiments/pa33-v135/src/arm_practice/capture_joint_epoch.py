"""Conservative same-physics-step acquisition/encoder join for a paused render.
This verifies host read timing and a paused physics interval only. GPU annotator
pixels may belong to an earlier render; image-to-physics association is not
proved here. This is never an arm-clearance certificate.
"""
import numpy as np

def validate_capture_epoch(frame, *, received_monotonic_ns, now_monotonic_ns, max_age_ns=500_000_000):
    result={"matched":False,"motion_authorized":False,"image_capture_association_verified":False}
    try:
        keys=("capture_monotonic_ns","encoder_read_start_monotonic_ns","encoder_read_end_monotonic_ns","arm_feedback_capture_monotonic_ns","render_start_monotonic_ns","render_return_monotonic_ns","readback_return_monotonic_ns","physics_step_before","physics_step_after")
        d={}
        for key in keys:
            a=np.asarray(frame[key])
            if a.shape!=() or a.dtype.kind not in "iu":raise ValueError("invalid_integer_metadata:"+key)
            d[key]=int(a)
        if any(type(x) is not int for x in [received_monotonic_ns,now_monotonic_ns,max_age_ns]) or max_age_ns<=0:raise ValueError("invalid_local_clock")
        c=d["capture_monotonic_ns"];s=d["encoder_read_start_monotonic_ns"];e=d["encoder_read_end_monotonic_ns"]
        if not 0<=c==s<=e<=d["render_start_monotonic_ns"]<=d["render_return_monotonic_ns"]<=d["readback_return_monotonic_ns"]<=received_monotonic_ns<=now_monotonic_ns:raise ValueError("nonmonotonic_acquisition")
        if d["arm_feedback_capture_monotonic_ns"]!=(s+e)//2:raise ValueError("encoder_midpoint_mismatch")
        if e-s>1_000_000 or d["render_start_monotonic_ns"]-s>1_000_000:raise ValueError("encoder_capture_uncertainty_over_1ms")
        if d["physics_step_before"]<0 or d["physics_step_after"]<0:raise ValueError("invalid_physics_step")
        if d["physics_step_before"]!=d["physics_step_after"]:raise ValueError("physics_advanced_during_render")
        q=np.asarray(frame["arm_encoder"],float);after=np.asarray(frame["arm_encoder_after_render"],float)
        if q.shape!=(5,) or after.shape!=(5,) or not np.isfinite(q).all() or not np.isfinite(after).all() or not np.array_equal(q,after):raise ValueError("encoder_changed_during_render")
        age=now_monotonic_ns-c
        if age>max_age_ns:return dict(result,reason="stale_complete_acquisition",age_ns=age)
        return dict(result,matched=True,reason="same_step_encoder_render_join_only",age_ns=age,encoder_read_duration_ns=e-s,encoder_to_render_start_ns=d["render_start_monotonic_ns"]-s)
    except (KeyError,TypeError,ValueError,OverflowError) as error:return dict(result,reason=str(error))

EPOCH_FIELDS=("capture_monotonic_ns","encoder_read_start_monotonic_ns","encoder_read_end_monotonic_ns","arm_feedback_capture_monotonic_ns","render_start_monotonic_ns","render_return_monotonic_ns","readback_return_monotonic_ns","physics_step_before","physics_step_after","arm_encoder","arm_encoder_after_render")

def joined_arm_feedback(packet, feedback, *, received_monotonic_ns, now_monotonic_ns):
    """Propagate only producer evidence matched to the actual packet encoder."""
    result={"arm_feedback_capture_monotonic_ns":None,"arm_feedback_live_freshness_verified":False,"arm_capture_epoch":None}
    try:
        epoch=packet['arm_capture_epoch']
        verdict=validate_capture_epoch(epoch,received_monotonic_ns=received_monotonic_ns,now_monotonic_ns=now_monotonic_ns)
        if not verdict['matched']:raise ValueError(verdict['reason'])
        # An image may be usable for 500 ms; live actuator feedback may not.
        if verdict['age_ns']>100_000_000:raise ValueError('actuator_feedback_older_than_100ms')
        if not feedback.get('arm_feedback_valid'):raise ValueError('invalid_packet_encoder')
        if epoch['capture_monotonic_ns']!=packet['frame_capture_monotonic_ns']:raise ValueError('packet_capture_mismatch')
        if epoch['run_id']!=packet['capture_run_id'] or epoch['stamp_ns']!=packet['stamp_ns']:raise ValueError('packet_epoch_identity_mismatch')
        if not np.array_equal(np.asarray(epoch['arm_encoder']),np.asarray(feedback['arm_measured_position'])):raise ValueError('packet_encoder_mismatch')
        return dict(result,arm_feedback_capture_monotonic_ns=epoch['arm_feedback_capture_monotonic_ns'],arm_feedback_live_freshness_verified=True,arm_capture_epoch=epoch,arm_capture_join=verdict)
    except (KeyError,TypeError,ValueError) as error:return dict(result,arm_feedback_valid=False,arm_capture_join=dict(matched=False,reason=str(error),motion_authorized=False))

def require_frame_epoch(frame,packet,*,now_monotonic_ns):
    """Archive and ROS result must carry exactly the same joint-read evidence."""
    verdict=validate_capture_epoch(frame,received_monotonic_ns=packet['packet_received_monotonic_ns'],now_monotonic_ns=now_monotonic_ns)
    if not verdict['matched']:raise ValueError('capture_epoch:'+verdict['reason'])
    if verdict['age_ns']>100_000_000:raise ValueError('actuator_feedback_older_than_100ms')
    epoch=packet['arm_capture_epoch']
    for key in EPOCH_FIELDS:
        if not np.array_equal(np.asarray(frame[key]),np.asarray(epoch[key])):raise ValueError('frame_epoch_mismatch:'+key)
    if packet.get('arm_feedback_live_freshness_verified') is not True or packet['arm_feedback_capture_monotonic_ns']!=epoch['arm_feedback_capture_monotonic_ns']:raise ValueError('encoder_epoch_not_verified')
    if epoch['run_id']!=packet['capture_run_id'] or epoch['stamp_ns']!=packet['stamp_ns']:raise ValueError('result_epoch_identity_mismatch')
    if not np.array_equal(np.asarray(epoch['arm_encoder']),np.asarray(packet['arm_measured_position'])):raise ValueError('result_epoch_encoder_mismatch')
    return verdict
