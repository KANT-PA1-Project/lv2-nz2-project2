"""Pure live-slice diagnostics and initial candidate geometry revalidation.

Never inserts free cells, changes a source request revision, or authorizes
motion. A revalidation certificate is only geometry evidence for a separate
freshness/generation guard. Exact-body initial-volume exceptions are not
implemented; unknown cells remain blocked, including under the robot.
"""
import hashlib
import json
import math
import numpy as np
from swept_footprint import _samples, motion_clear, convex_cell_cover
from oriented_path_adapter import uncertainty_footprint, valid_polyline, wrap


def _hash(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def slice_snapshot(metadata, raw_data):
    """Validate and fingerprint actual raw slice; do not trust a supplied mask."""
    frame=metadata['frame'];width=int(metadata['width']);height=int(metadata['height'])
    resolution=float(metadata['resolution']);origin=np.asarray(metadata['origin'],float)
    if frame!='map' or min(width,height)<2 or width*height>2_000_000:raise ValueError('slice_frame_or_shape')
    if origin.shape!=(3,) or not np.isfinite(origin).all() or not 0<resolution<=.25:raise ValueError('slice_geometry')
    unknown=np.frombuffer(bytes.fromhex(metadata['unknown_value_bits']),dtype=np.float32)
    if unknown.shape!=(1,):raise ValueError('slice_unknown_encoding')
    raw=np.asarray(raw_data,dtype=np.float32).copy()
    if raw.shape!=(height,width):raise ValueError('slice_array_shape')
    known=np.isfinite(raw)&(raw!=unknown[0]);occupied=~known|(raw<=resolution/math.sqrt(2))
    geometry=dict(frame=frame,width=width,height=height,resolution=resolution,
        origin=origin.tolist(),axis_order='row_y_col_x',
        occupied_sha256=hashlib.sha256(occupied.tobytes()).hexdigest(),
        unknown_sha256=hashlib.sha256((~known).tobytes()).hexdigest())
    raw.flags.writeable=False;known.flags.writeable=False;occupied.flags.writeable=False
    return dict(raw=raw,known=known,occupied=occupied,resolution=resolution,origin=origin,
        frame=frame,revision=_hash(geometry),geometry=geometry,
        raw_sha256=hashlib.sha256(raw.tobytes()).hexdigest())


def footprint_evidence(pose, footprint, grid):
    """Full closed-cell footprint intersection with raw-slice failure reasons."""
    pose=np.asarray(pose,float);foot=np.asarray(footprint,float)
    if pose.shape!=(3,) or not np.isfinite(pose).all():raise ValueError('sensor_pose_invalid')
    c,s=np.cos(pose[2]),np.sin(pose[2]);world=foot@np.array([[c,s],[-s,c]])+pose[:2]
    # Every intersecting cell is retained, including a small corner overlap.
    cells=convex_cell_cover(world-grid['origin'][:2],float(grid['resolution']))
    h,w=grid['occupied'].shape;inside=(cells[:,0]>=0)&(cells[:,0]<w)&(cells[:,1]>=0)&(cells[:,1]<h)
    ci=cells[inside];known=grid['known'][ci[:,1],ci[:,0]];occ=grid['occupied'][ci[:,1],ci[:,0]]
    outside=int((~inside).sum());unknown=int((~known).sum());blocked=int((known&occ).sum())
    reason='observed_free'
    if outside:reason='footprint_outside_live_slice'
    elif unknown:reason='footprint_unobserved'
    elif blocked:reason='footprint_known_occupied'
    return dict(clear=not(outside or unknown or blocked),reason=reason,cells=len(cells),
        outside_cells=outside,unknown_cells=unknown,known_occupied_cells=blocked,
        known_free_cells=int((known&~occ).sum()))


def revalidate_initial_geometry(plan, source_request, latest_grid, sensor_pose,
                                *, max_point_queries=5_000_000):
    """Recheck initial preparation and unchanged world path on the newest grid.

    Only initial (not already moving) candidate acceptance is supported. Caller
    must retain original request/result pairing and independently apply the
    original 5s plan / .15s sensor / 1s map clocks, goal/path/footprint generations,
    stopped/transport conditions and every downstream actuator/sensor veto.
    """
    def result(ok,reason,**extra):
        return dict(geometry_revalidated=ok,reason=reason,motion_authorized=False,**extra)
    try:
        if not isinstance(plan,dict) or plan.get('ok') is not True:
            return result(False,'source_plan_invalid',source_reason=plan.get('reason') if isinstance(plan,dict) else None)
        path=np.asarray(plan['path'],float);pose=np.asarray(sensor_pose,float)
        source=np.asarray(source_request['sensor_pose'],float);origin=np.asarray(plan['preparation_origin'],float)
        yaw=float(plan['yaw_rad']);margin=float(source_request['position_margin_m']);band=float(source_request['yaw_error_rad'])
        if path.ndim!=2 or path.shape[1]!=2 or not 2<=len(path)<=10000 or pose.shape!=(3,) or source.shape!=(3,) or origin.shape!=(2,):raise ValueError('candidate_shape')
        if not np.isfinite(np.r_[path.ravel(),pose,source,origin,yaw,margin,band]).all():raise ValueError('candidate_nonfinite')
        if margin<=0 or not 0<band<=math.pi/4:raise ValueError('uncertainty_invalid')
        if abs(float(plan['position_margin_m'])-margin)>1e-12 or abs(float(plan['yaw_error_rad'])-band)>1e-12:raise ValueError('candidate_uncertainty_mismatch')
        if np.linalg.norm(origin-source[:2])>1e-8 or np.linalg.norm(path[0]-source[:2])>1e-8:raise ValueError('candidate_start_changed')
        if np.linalg.norm(path[-1]-np.asarray(source_request['path'][-1]))>1e-8:raise ValueError('candidate_goal_changed')
        if np.linalg.norm(pose[:2]-source[:2])>margin or abs(wrap(pose[2]-source[2]))>band:return result(False,'preparation_pose_drift')
        foot=uncertainty_footprint(source_request['footprint'],margin,band)
        supplied=np.asarray(plan['robust_footprint'],float)
        if supplied.shape!=foot.shape or not np.allclose(supplied,foot,rtol=0,atol=1e-10):raise ValueError('candidate_footprint_mismatch')
        evidence=footprint_evidence(pose,foot,latest_grid)
        if not evidence['clear']:return result(False,evidence['reason'],footprint=evidence)
        res=latest_grid['resolution'];offset=latest_grid['origin'][:2]
        count=len(_samples(tuple(map(tuple,foot)),res));angle=wrap(yaw-pose[2]);radius=np.linalg.norm(foot,axis=1).max()
        estimates=max(2,int(math.ceil(abs(angle)*radius/(res*.5)))+1)
        estimates+=max(2,int(math.ceil(abs(wrap(yaw-source[2]))*radius/(res*.5)))+1)
        estimates+=max(2,int(math.ceil(np.linalg.norm(pose[:2]-origin)/(res*.5)))+1)
        estimates+=sum(max(2,int(math.ceil(np.linalg.norm(b-a)/(res*.5)))+1) for a,b in zip(path[:-1],path[1:]))
        if estimates*count>max_point_queries:return result(False,'bounded_revalidation_budget',estimated_point_queries=estimates*count)
        if not motion_clear(pose[:2]-offset,pose[2],[0,0,angle],foot,latest_grid['occupied'],res,horizon=1.):
            return result(False,'latest_preparation_rotation_blocked')
        # Check the original preparation origin too; movement within1cm must
        # not turn the untested start-to-path connector into an exception.
        if not motion_clear(origin-offset,source[2],[0,0,wrap(yaw-source[2])],foot,latest_grid['occupied'],res,horizon=1.):
            return result(False,'latest_source_preparation_blocked')
        if not valid_polyline(np.asarray([pose[:2],origin])-offset,yaw,foot,latest_grid['occupied'],res):
            return result(False,'latest_start_connector_blocked')
        if not valid_polyline(path-offset,yaw,foot,latest_grid['occupied'],res):
            return result(False,'latest_path_sweep_blocked')
        certificate=dict(source_request_id=source_request['request_id'],
            source_map_revision=source_request['map_revision'],
            latest_map_revision=latest_grid['revision'],latest_raw_sha256=latest_grid['raw_sha256'],
            goal_index=source_request['goal_index'],path_revision=source_request['path_revision'],
            footprint_revision=source_request['footprint_revision'],sensor_pose=pose.tolist(),
            candidate_geometry_sha256=_hash(dict(path=path.tolist(),yaw=yaw,footprint=foot.tolist())),
            geometry_scope='initial_preparation_rotation_connector_and_full_world_path',
            original_request_created_monotonic_s=source_request['created_monotonic_s'],
            latest_map_geometry=latest_grid['geometry'])
        return result(True,'latest_geometry_rechecked',certificate=certificate)
    except (ValueError,TypeError,KeyError,OverflowError,IndexError) as error:
        return result(False,'invalid_revalidation_input:'+str(error))
