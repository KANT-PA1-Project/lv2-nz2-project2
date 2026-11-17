"""Pre-adaptation structural/timing gate; never authenticates a source or authorizes motion."""
import numpy as np


def check_raw_odometry(record, *, expected_stamp_ns, last_consumed_stamp_ns,
                       now_ns, max_receipt_age_ns, expected_frame_id='odom'):
    result = dict(structurally_eligible=False, source_session_verified=False,
                  capture_freshness_verified=False, execution_authorized=False)
    try:
        for value in (expected_stamp_ns, now_ns, max_receipt_age_ns):
            if type(value) is not int or value < 0:
                raise ValueError('invalid_time_argument')
        if last_consumed_stamp_ns is not None and (type(last_consumed_stamp_ns) is not int or last_consumed_stamp_ns < 0):
            raise ValueError('invalid_last_stamp')
        stamp, receipt = record['stamp_ns'], record['received_monotonic_ns']
        if type(stamp) is not int or type(receipt) is not int or stamp < 0 or receipt < 0:
            raise ValueError('invalid_record_time')
        if stamp != expected_stamp_ns:
            raise ValueError('stamp_mismatch')
        if last_consumed_stamp_ns is not None and stamp <= last_consumed_stamp_ns:
            raise ValueError('reused_or_regressed_stamp')
        if not 0 <= now_ns-receipt <= max_receipt_age_ns:
            raise ValueError('receipt_age_invalid')
        if record['source_topic'] != '/visual_slam/tracking/odometry' or record['unmodified'] is not True:
            raise ValueError('raw_source_required')
        if not expected_frame_id or record['frame_id'] != expected_frame_id or record['child_frame_id'] != 'base_link':
            raise ValueError('frame_mismatch')
        pos=np.asarray(record['position'],float);q=np.asarray(record['quaternion_xyzw'],float)
        if pos.shape!=(3,) or q.shape!=(4,) or not np.isfinite(pos).all() or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>.01:
            raise ValueError('invalid_pose')
        cov=np.asarray(record['covariance'],float)
        if cov.shape!=(36,):
            raise ValueError('invalid_covariance_shape')
        cov=cov.reshape(6,6)
        if not np.isfinite(cov).all() or np.any(np.diag(cov)<0) or not np.allclose(cov,cov.T,rtol=0,atol=1e-10) or np.linalg.eigvalsh(cov).min() < -1e-10:
            raise ValueError('invalid_covariance')
        return dict(result,structurally_eligible=True,reason='structure_and_receipt_age_only')
    except (KeyError,TypeError,ValueError,OverflowError,np.linalg.LinAlgError) as exc:
        return dict(result,reason=str(exc))
