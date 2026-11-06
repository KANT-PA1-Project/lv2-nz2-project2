"""Compare archived viewpoints for wrist path visibility, without authorizing motion.

One representative per 2cm voxel is diagnostic only. Historical free rays are
unioned under a static-scene assumption; live freshness/registration uncertainty
and whole swept-volume clearance remain unverified.
"""
import argparse
import hashlib
import itertools
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'scripts/sensor_runtime')]
from camera_rig import PreparedCameraRig
from audit_r12_usd_arm_path import CollisionModel


def map_transform(pose):
    x, y, yaw = pose
    c, s = np.cos(yaw), np.sin(yaw)
    transform = np.eye(4)
    transform[:2, :2] = [[c, -s], [s, c]]
    transform[:2, 3] = [x, y]
    return transform


def free_rays(points, camera, depth):
    pc = (points-camera[:3, 3])@camera[:3, :3]
    z = pc[:, 2]
    valid = z > .03
    uv = np.zeros((len(points), 2), int)
    uv[valid] = np.rint(400*pc[valid, :2]/z[valid, None]+[320, 240]).astype(int)
    valid &= (uv[:, 0] >= 0) & (uv[:, 0] < 640) & (uv[:, 1] >= 0) & (uv[:, 1] < 480)
    ids = np.where(valid)[0]
    d = depth[uv[ids, 1], uv[ids, 0]]
    free = np.zeros(len(points), bool)
    free[ids] = np.isfinite(d) & (d > .03) & (d < 10) & (z[ids] < d-.03)
    return free


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--path-audit', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--explain-unknown', action='store_true')
    args = parser.parse_args()
    path = json.loads(args.path_audit.read_text())
    search = json.loads(Path(path['source_search']).read_text())
    ref_frame = Path(search['frame'])
    ref_meta = json.loads(ref_frame.with_suffix('.json').read_text())
    rows = json.loads((ref_frame.parent.parent/'ros_results.json').read_text())
    by_stamp = {row['stamp_ns']: row for row in rows}
    ref = by_stamp[ref_meta['stamp_ns']]
    world_to_ref = np.linalg.inv(map_transform(ref['estimated_map_pose']))
    model, rig = CollisionModel(), PreparedCameraRig(ROOT)
    cfg = json.loads((ROOT/'configs/r12_robot.json').read_text())
    camera_cfg = json.loads((ROOT/'configs/sensors/camera_rig.json').read_text())
    if camera_cfg['intrinsics'] != [400, 400, 320, 240] or camera_cfg['resolution'] != [640, 480]:
        raise ValueError('Unsupported camera calibration')
    for name, digest in search['robot_configuration_sha256'].items():
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest() != digest:
            raise ValueError('Robot configuration changed')
    grids = {}
    for name, body, _, _ in model.boxes:
        if body != 'base_link':
            half = model.sizes[name]/2 + .03
            grids[name] = np.asarray(list(itertools.product(*[
                np.linspace(-h, h, int(np.ceil(2*h/.02))+1) for h in half])))
    points = []
    for record in path['samples']:
        transforms = model.transforms(record['q'])
        for name, body, local, _ in model.boxes:
            if body != 'base_link':
                matrix = transforms[body]@local
                points.append(grids[name]@matrix[:3, :3].T+matrix[:3, 3])
    points = np.concatenate(points)
    _, indices = np.unique(np.floor(points/.02).astype(int), axis=0, return_index=True)
    points = points[indices]
    inside = np.zeros(len(points), bool)
    start = model.transforms(ref_meta['arm_measured_position'])
    for name, body, local, _ in model.boxes:
        matrix = start[body]@local
        inside |= np.all(np.abs((points-matrix[:3, 3])@matrix[:3, :3]) <= model.sizes[name]/2, axis=1)
    points = points[~inside]
    union = np.zeros(len(points), bool)
    views, current_free = [], None
    for file in sorted(ref_frame.parent.glob('*.npz')):
        if not file.stem.isdigit() or int(file.stem) > ref_meta['frame_index']:
            continue
        meta = json.loads(file.with_suffix('.json').read_text())
        if hashlib.sha256(file.read_bytes()).hexdigest() != meta['sha256']:
            raise ValueError('Frame checksum mismatch')
        row = by_stamp[meta['stamp_ns']]
        if meta['run_id'] != ref_meta['run_id'] or row['capture_run_id'] != meta['run_id']:
            raise ValueError('Run identity mismatch')
        if not row['encoder_valid'] or row['tracking'] != 1 or not row['arm_feedback_valid']:
            continue
        transforms = rig.transforms(dict(zip(cfg['joint_names'], meta['arm_measured_position'])))
        relative = world_to_ref@map_transform(row['estimated_map_pose'])
        free = np.zeros(len(points), bool)
        with np.load(file, allow_pickle=False) as data:
            for key in ('run_id', 'frame_index', 'stamp_ns'):
                if data[key].item() != meta[key]:
                    raise ValueError('Capture metadata mismatch')
            for name in ('r07_camera1', 'r07_camera2', 'r07_camera3', 'arm_camera'):
                depth = data['depth'] if name == 'r07_camera1' else data[name+'_depth']
                free |= free_rays(points, relative@transforms[name+'_left_optical'], depth)
        new = free & ~union
        union |= free
        views.append(dict(frame_index=meta['frame_index'], frame_sha256=meta['sha256'],
                          observed_free_representatives=int(free.sum()),
                          newly_observed_free_representatives=int(new.sum())))
        if file == ref_frame:
            current_free = int(free.sum())
    result = dict(reference_frame=ref_meta['frame_index'],
                  representative_points=len(points), reference_only_observed_free=current_free,
                  historical_union_observed_free=int(union.sum()),
                  historical_union_unconfirmed=int((~union).sum()), views=views,
                  source_path_audit=str(args.path_audit.resolve()),
                  execution_authorized=False, clearance_verified=False, runtime_integrated=False,
                  assumptions=['Static scene and planar estimated map poses, base Z=0; no fixture or PhysX world pose.',
                               '2cm voxel representatives do not cover the full volume or continuous sweep.',
                               'Archived free-ray union has no current freshness, covariance or dynamic-obstacle guarantee.'])
    if args.explain_unknown:
        unknown = points[~union]
        explanations = []
        for point in unknown:
            nearest = []
            for name, body, local, _ in model.boxes:
                matrix = start[body]@local
                distance = np.linalg.norm(np.maximum(
                    np.abs((point-matrix[:3, 3])@matrix[:3, :3])-model.sizes[name]/2, 0))
                nearest.append(dict(collider=name, body=body, distance_m=float(distance)))
            nearest.sort(key=lambda x: x['distance_m'])
            explanations.append(dict(point_reference_base_m=point.tolist(),
                                     nearest_start_colliders=nearest[:3],
                                     swept_colliders={}, view_reasons={},
                                     current_camera_evidence=[]))
        for record in path['samples']:
            transforms = model.transforms(record['q'])
            for name, body, local, _ in model.boxes:
                if body == 'base_link':
                    continue
                matrix = transforms[body]@local
                ids = np.where(np.all(np.abs((unknown-matrix[:3, 3])@matrix[:3, :3])
                                      <= model.sizes[name]/2+.030000001, axis=1))[0]
                for index in ids:
                    entry = explanations[index]['swept_colliders'].setdefault(
                        name, dict(body=body, samples=0, first_direction=record['direction'],
                                   first_sample=record['sample']))
                    entry['samples'] += 1
        # Read only already validated views. Keep missing depth/FOV separate
        # from observed surfaces; none of these classifications clear space.
        for view in views:
            file = ref_frame.parent / f"{view['frame_index']:06d}.npz"
            meta = json.loads(file.with_suffix('.json').read_text())
            row = by_stamp[meta['stamp_ns']]
            transforms = rig.transforms(dict(zip(cfg['joint_names'], meta['arm_measured_position'])))
            relative = world_to_ref@map_transform(row['estimated_map_pose'])
            with np.load(file, allow_pickle=False) as data:
                for name in ('r07_camera1', 'r07_camera2', 'r07_camera3', 'arm_camera'):
                    camera = relative@transforms[name+'_left_optical']
                    depth = data['depth'] if name == 'r07_camera1' else data[name+'_depth']
                    pc = (unknown-camera[:3, 3])@camera[:3, :3]
                    for index, point in enumerate(pc):
                        evidence = dict(camera=name, optical_depth_m=float(point[2]))
                        if point[2] <= .03:
                            reason = 'behind_or_too_near'
                        else:
                            uv = np.rint(400*point[:2]/point[2]+[320, 240]).astype(int)
                            evidence['pixel'] = uv.tolist()
                            if not (0 <= uv[0] < 640 and 0 <= uv[1] < 480):
                                reason = 'outside_fov'
                            else:
                                d = float(depth[uv[1], uv[0]])
                                if not np.isfinite(d) or not .03 < d < 10:
                                    reason = 'invalid_or_masked_depth'
                                else:
                                    evidence['surface_depth_m'] = d
                                    evidence['surface_minus_probe_m'] = d-float(point[2])
                                    reason = 'occluded_or_inside_3cm_depth_margin' if point[2] >= d-.03 else 'free'
                        if reason == 'free':
                            raise ValueError('Unknown explanation disagrees with free-ray union')
                        evidence['reason'] = reason
                        counts = explanations[index]['view_reasons']
                        counts[reason] = counts.get(reason, 0)+1
                        if file == ref_frame:
                            explanations[index]['current_camera_evidence'].append(evidence)
        result['unknown_explanations'] = explanations
    with args.output.open('x') as handle:
        json.dump(result, handle, indent=2)
        handle.write('\n')
    print(json.dumps({key: value for key, value in result.items() if key != 'views'}))


if __name__ == '__main__':
    main()
