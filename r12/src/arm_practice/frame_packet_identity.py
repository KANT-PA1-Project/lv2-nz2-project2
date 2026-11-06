"""Archive frame/packet identity validation, never a live-motion authorization."""
import numpy as np

def check_frame_packet_identity(frame, packet, expected_frame_index):
    result={'matched':False,'live_freshness_verified':False,'execution_authorized':False}
    try:
        index=np.asarray(frame['frame_index']);stamp=np.asarray(frame['stamp_ns']);clock=np.asarray(frame['clock_domain'])
        if index.shape!=() or stamp.shape!=() or clock.shape!=() or index.dtype.kind not in 'iu' or stamp.dtype.kind not in 'iu':
            return dict(result,reason='invalid_metadata')
        index=int(index);stamp=int(stamp)
        if index<0 or str(clock)!='synthetic_packet_20hz':return dict(result,reason='unsupported_clock_or_index')
        packet_stamp=packet['stamp_ns'];arm_stamp=packet['arm_feedback_stamp_ns']
        if type(packet_stamp) is not int or type(arm_stamp) is not int:return dict(result,reason='invalid_packet_stamp')
        if index!=expected_frame_index or stamp!=1_000_000_000+index*50_000_000 or stamp!=packet_stamp or stamp!=arm_stamp:
            return dict(result,reason='frame_packet_mismatch')
        if not packet.get('arm_feedback_valid',False):return dict(result,reason='invalid_arm_feedback')
        return dict(result,matched=True,reason='archive_packet_identity_only',frame_index=index,stamp_ns=stamp)
    except (KeyError,TypeError,ValueError,OverflowError):
        return dict(result,reason='missing_or_invalid_metadata')

def check_live_frame_provenance(frame, packet, expected_frame_index, expected_run_id, now_ns, max_age_ns):
    """Same-host monotonic clock only; freshness never authorizes motion."""
    result=check_frame_packet_identity(frame,packet,expected_frame_index)
    if not result['matched']:return result
    result=dict(result,matched=False)
    try:
        run=str(np.asarray(frame['run_id']).item());capture=np.asarray(frame['capture_monotonic_ns'])
        if not expected_run_id or run!=expected_run_id or packet.get('capture_run_id')!=run:
            return dict(result,reason='run_mismatch')
        if capture.shape!=() or capture.dtype.kind not in 'iu':return dict(result,reason='invalid_capture_time')
        capture=int(capture);received=packet['packet_received_monotonic_ns']
        if any(type(v) is not int for v in [now_ns,max_age_ns,received,packet['frame_capture_monotonic_ns']]) or max_age_ns<=0:
            return dict(result,reason='invalid_clock')
        if capture!=packet['frame_capture_monotonic_ns'] or not 0<=capture<=received<=now_ns:
            return dict(result,reason='clock_mismatch')
        age=now_ns-capture
        if age>max_age_ns:return dict(result,reason='stale_frame',age_ns=age)
        return dict(result,matched=True,live_freshness_verified=True,reason='same_host_provenance_only',age_ns=age)
    except (KeyError,TypeError,ValueError,OverflowError):
        return dict(result,reason='missing_or_invalid_provenance')
