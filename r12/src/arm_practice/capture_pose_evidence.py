"""Isolated exact-stamp upper-camera schema check, not localization certification."""
import re
import numpy as np


def check_capture_pose_evidence(bundle, *, expected_calibration_sha256):
    result = dict(schema_consistent=False, pose_join_verified=False,
                  current_observation_eligible=False, execution_authorized=False)
    try:
        f, p, c, e = (bundle[k] for k in ('frame', 'raw_odometry', 'calibration', 'extrinsic'))
        if not isinstance(expected_calibration_sha256, str) or not re.fullmatch('[0-9a-f]{64}', expected_calibration_sha256):
            raise ValueError('invalid_expected_calibration')
        for k in ('run_id', 'clock_epoch'):
            if type(f[k]) is not str or not f[k] or p[k] != f[k] or e[k] != f[k]:
                raise ValueError('session_mismatch')
        if f['clock_domain'] != 'synthetic_packet_20hz' or p['clock_domain'] != f['clock_domain']:
            raise ValueError('clock_domain_mismatch')
        if type(f['frame_index']) is not int or f['frame_index'] < 0:
            raise ValueError('invalid_frame_index')
        if any(type(x) is not int or x < 0 for x in (f['stamp_ns'], f['capture_monotonic_ns'], p['stamp_ns'], p['received_monotonic_ns'], e['stamp_ns'])):
            raise ValueError('invalid_time')
        if f['stamp_ns'] != 1000000000 + 50000000*f['frame_index'] or p['stamp_ns'] != f['stamp_ns'] or e['stamp_ns'] != f['stamp_ns']:
            raise ValueError('stamp_mismatch')
        if p['received_monotonic_ns'] < f['capture_monotonic_ns']:
            raise ValueError('clock_order')
        if f['camera'] not in ('r07_camera1', 'r07_camera2', 'r07_camera3'):
            raise ValueError('unsupported_camera_arm_pending')
        if c['camera'] != f['camera'] or e['child_frame_id'] != f['camera']+'_left_optical':
            raise ValueError('camera_mismatch')
        if c['sha256'] != expected_calibration_sha256 or e['calibration_sha256'] != c['sha256']:
            raise ValueError('calibration_mismatch')
        resolution=f['resolution']
        if not isinstance(resolution,list) or len(resolution)!=2 or any(type(x) is not int or x<=0 for x in resolution) or c['resolution']!=resolution:
            raise ValueError('resolution_mismatch')
        K=np.asarray(c['K'],float)
        if K.shape!=(3,3) or not np.isfinite(K).all() or K[0,0]<=0 or K[1,1]<=0 or not np.allclose(K[2],[0,0,1]):
            raise ValueError('invalid_intrinsics')
        if p['source_topic']!='/visual_slam/tracking/odometry' or p['tracking_valid'] is not True or p['unmodified'] is not True:
            raise ValueError('raw_tracking_required')
        if p['frame_id']!=c['pose_frame_id'] or not p['frame_id'] or p['child_frame_id']!='base_link' or e['parent_frame_id']!='base_link':
            raise ValueError('coordinate_frame_mismatch')
        q=np.asarray(p['quaternion_xyzw'],float);pos=np.asarray(p['position'],float);cov=np.asarray(p['covariance'],float)
        if q.shape!=(4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>1e-6 or pos.shape!=(3,) or not np.isfinite(pos).all():
            raise ValueError('invalid_pose')
        if cov.shape == (36,):
            cov = cov.reshape(6,6)
        if cov.shape!=(6,6) or not np.isfinite(cov).all() or not np.allclose(cov,cov.T,atol=1e-10) or np.linalg.eigvalsh(cov).min() < -1e-10:
            raise ValueError('invalid_covariance')
        T=np.asarray(e['T_parent_child'],float)
        if T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0,0,0,1]) or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-6) or abs(np.linalg.det(T[:3,:3])-1)>1e-6:
            raise ValueError('invalid_extrinsic')
        return dict(result,schema_consistent=True,reason='exact_stamp_schema_only_source_authenticity_and_uncertainty_unverified')
    except (KeyError,TypeError,ValueError,OverflowError,np.linalg.LinAlgError) as exc:
        return dict(result,reason=str(exc))
