"""Sensor-only hand-eye candidate; no fixture target or nominal mount input.

Inputs: archived verified RGB/depth, marker registry size, encoder FK to
arm_link5, and schema intrinsics. Never imports the evaluation-only module
or reads scene truth / nominal camera extrinsics. Not physical calibration.
"""
from pathlib import Path
import hashlib
import json
import sys
import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

BASE=Path(__file__).resolve().parent
SOURCE=BASE/'source'
sys.path[:0]=[str(SOURCE/'src'),str(SOURCE/'scripts/sensor_runtime')]
from arm_practice.kinematics import forward_transform
from arm_practice.mission_markers import marker_spec


def matrix(x):
    T=np.eye(4);T[:3,:3]=Rotation.from_rotvec(x[:3]).as_matrix();T[:3,3]=x[3:]
    return T


def vector(T):
    return np.r_[Rotation.from_matrix(T[:3,:3]).as_rotvec(),T[:3,3]]


def metrics(pairs,X,Y):
    errors=[np.linalg.inv(Y)@B@X@C for B,C in pairs]
    translation=np.array([np.linalg.norm(T[:3,3]) for T in errors])
    angle=np.array([Rotation.from_matrix(T[:3,:3]).magnitude() for T in errors])
    return dict(samples=len(pairs),translation_rms_m=float(np.sqrt(np.mean(translation**2))),
                translation_max_m=float(translation.max()),rotation_rms_rad=float(np.sqrt(np.mean(angle**2))),
                rotation_max_rad=float(angle.max()))


def fit_handeye(train,holdout):
    if len(train)<6 or len(holdout)<2:raise ValueError('insufficient_train_or_holdout_views')
    for B,C in train+holdout:
        for T in (B,C):
            if T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0,0,0,1]):raise ValueError('invalid_pose')
            if not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-6) or np.linalg.det(T[:3,:3])<.999999:raise ValueError('invalid_rotation')
    axes=np.array([Rotation.from_matrix((np.linalg.inv(train[0][0])@B)[:3,:3]).as_rotvec() for B,_ in train[1:]])
    excitation=np.linalg.svd(axes,compute_uv=False)
    if len(excitation)<2 or excitation[1]<.02:raise ValueError('insufficient_nonparallel_rotation_excitation')
    R,t=cv2.calibrateHandEye([B[:3,:3] for B,C in train],[B[:3,3] for B,C in train],
                            [C[:3,:3] for B,C in train],[C[:3,3] for B,C in train],method=cv2.CALIB_HAND_EYE_PARK)
    X=np.eye(4);X[:3,:3]=R;X[:3,3]=np.asarray(t).reshape(3)
    if not np.isfinite(X).all():raise ValueError('handeye_initialization_failed')
    Ys=[B@X@C for B,C in train]
    Y=np.eye(4);Y[:3,:3]=Rotation.from_matrix([T[:3,:3] for T in Ys]).mean().as_matrix();Y[:3,3]=np.mean([T[:3,3] for T in Ys],axis=0)
    def residual(x):
        a=matrix(x[:6]);b=matrix(x[6:]);v=[]
        for B,C in train:
            T=np.linalg.inv(b)@B@a@C
            v.extend(T[:3,3]/.001)
            v.extend(Rotation.from_matrix(T[:3,:3]).as_rotvec()/.01)
        return np.asarray(v)
    result=least_squares(residual,np.r_[vector(X),vector(Y)],loss='soft_l1',max_nfev=1000,
                         xtol=1e-12,ftol=1e-12,gtol=1e-12)
    if not result.success or not np.isfinite(result.x).all():raise ValueError('fit_did_not_converge')
    singular=np.linalg.svd(result.jac,compute_uv=False)
    if singular[-1]<1e-7:raise ValueError('rank_deficient_fit')
    X=matrix(result.x[:6]);Y=matrix(result.x[6:])
    return dict(T_link_camera_candidate=X.tolist(),T_base_tag_nuisance=Y.tolist(),
                excitation_singular_values_rad=excitation.tolist(),fit_jacobian_singular_values=singular.tolist(),
                train_residuals=metrics(train,X,Y),holdout_residuals=metrics(holdout,X,Y),
                optimizer_nfev=result.nfev,optimizer_converged=True)


def observed_tag_pose(rgb,depth,corners,intrinsics,black_size):
    fx,fy,cx,cy=intrinsics;corners=np.asarray(corners,float)
    center=corners.mean(axis=0);mask=np.zeros(depth.shape,np.uint8)
    cv2.fillConvexPoly(mask,np.rint(center+.5*(corners-center)).astype(np.int32),1)
    yy,xx=np.nonzero(mask);z=depth[yy,xx].astype(float)
    valid=np.isfinite(z)&(z>.01)&(z<30)
    if len(z)<25 or valid.mean()<.95:raise ValueError('insufficient_depth')
    yy,xx,z=yy[valid],xx[valid],z[valid]
    points=np.c_[(xx-cx)*z/fx,(yy-cy)*z/fy,z];median=np.median(points,axis=0)
    _,_,vt=np.linalg.svd(points-median,full_matrices=False);inliers=abs((points-median)@vt[-1])<.002
    if inliers.mean()<.95:raise ValueError('nonplanar_tag_surface')
    centroid=points[inliers].mean(axis=0);_,_,vt=np.linalg.svd(points[inliers]-centroid,full_matrices=False);normal=vt[-1]
    rays=np.c_[(corners[:,0]-cx)/fx,(corners[:,1]-cy)/fy,np.ones(4)]
    denominators=rays@normal
    if np.any(abs(denominators)<.2):raise ValueError('grazing_tag')
    observed=rays*((normal@centroid)/denominators)[:,None]
    half=black_size/2
    canonical=np.array([[-half,half,0],[half,half,0],[half,-half,0],[-half,-half,0]])
    center3=observed.mean(axis=0);U,_,vt=np.linalg.svd(canonical.T@(observed-center3))
    R=vt.T@U.T
    if np.linalg.det(R)<0:vt[-1]*=-1;R=vt.T@U.T
    T=np.eye(4);T[:3,:3]=R;T[:3,3]=center3
    fit_points=(R@canonical.T).T+center3
    rms=float(np.sqrt(np.mean(np.sum((fit_points-observed)**2,axis=1))))
    return T,rms


def load_views():
    index=json.loads((BASE/'calibration_dataset_index.json').read_text())
    data=[];identities=set();seen_q={};pinned_model=None;pinned_schema=None
    for view in index['views']:
        run=BASE/view['run'];source=run/'source'
        provenance=json.loads((run/'provenance.json').read_text())['source_sha256']
        for name in ('configs/mixed_camera_schema.json','models/r12/r12_arm.urdf','src/arm_practice/kinematics.py'):
            if hashlib.sha256((source/name).read_bytes()).hexdigest()!=provenance[name]:raise ValueError('calibration_source_changed:'+name)
        model_hash=provenance['models/r12/r12_arm.urdf'];schema_hash=provenance['configs/mixed_camera_schema.json']
        if pinned_model not in (None,model_hash) or pinned_schema not in (None,schema_hash):raise ValueError('mixed_model_or_schema_calibration_data')
        pinned_model=model_hash;pinned_schema=schema_hash
        supervisor=json.loads((run/'supervisor_result.json').read_text())
        if supervisor['renderer_exit_code']!=0:raise ValueError('failed_source_run')
        rows=json.loads((run/'producer_capture_results.json').read_text())
        row=rows[-1];proposal=row['live_view_proposal']
        if not row.get('captured') or not row.get('verification',{}).get('matched') or not proposal['accepted']:raise ValueError('unverified_archived_view')
        evidence=proposal.get('identity_evidence')
        # Baseline predates full evidence; its observed corners are re-detected.
        body=row['certificate']['body'];camera='arm_camera_left_optical';depth_name='arm_camera_depth'
        with np.load(run/f"rgbd_{row['frame']:02d}.npz",allow_pickle=False) as arrays:
            rgb=arrays[camera];depth=arrays[depth_name]
            if hashlib.sha256(rgb.tobytes()).hexdigest()!=body['pixel_sha256'][camera] or hashlib.sha256(depth.tobytes()).hexdigest()!=body['depth_sha256'][depth_name]:raise ValueError('archived_pixels_modified')
            from arm_practice.tag_detection import detect_tag_evidence
            detector=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11))
            decoded=detect_tag_evidence(cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR),detector)
            if decoded['ambiguous'] or len(decoded['tags'])!=1:raise ValueError('ambiguous_calibration_marker')
            tag=decoded['tags'][0];spec=marker_spec(tag['id'])
            if spec['role']!='puck':raise ValueError('unexpected_marker_role')
            identities.add(tag['id'])
            schema=json.loads((source/'configs/mixed_camera_schema.json').read_text())
            if body['model_sha256']!=model_hash or body['calibration_sha256']!=schema['calibration_sha256']:raise ValueError('capture_model_or_schema_mismatch')
            C,corner_rms=observed_tag_pose(rgb,depth,tag['corners'],schema['cameras'][camera]['intrinsics'],spec['black_size_m'])
        q=body['encoder_history']['encoder_q'];key=tuple(q)
        if key in seen_q:raise ValueError('repeated_or_split_leaking_joint_pose')
        seen_q[key]=view['split']
        joints={f'arm_joint{i+1}':float(x) for i,x in enumerate(q)}
        B=forward_transform(source/'models/r12/r12_arm.urdf','arm_link5',joints)
        data.append(dict(run=run.name,split=view['split'],q=q,B=B.tolist(),C=C.tolist(),
                         surface_corner_fit_rms_m=corner_rms,
                         rgb_sha256=body['pixel_sha256'][camera],depth_sha256=body['depth_sha256'][depth_name]))
    if len(identities)!=1:raise ValueError('calibration_identity_disagreement')
    return data


def main():
    cv2.setNumThreads(1)
    data=load_views()
    train=[(np.asarray(d['B']),np.asarray(d['C'])) for d in data if d['split']=='train']
    holdout=[(np.asarray(d['B']),np.asarray(d['C'])) for d in data if d['split']=='holdout']
    report=fit_handeye(train,holdout)
    report.update(scope='simulation sensor-only hand-eye candidate with held-out stationary poses',
                  inputs=data,fixture_truth_used_in_fit=False,nominal_camera_extrinsics_used_in_fit=False,
                  physical_calibration=False,pad_calibrated=False,calibration_accepted=False,
                  motion_authorized=False,error_bound_validated=False,
                  limitation='simulation-only residuals; not physical calibration, independent pad measurement, or a general sensor error bound')
    (BASE/'simulation_camera_candidate.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='inputs'},indent=2))


if __name__=='__main__':main()
