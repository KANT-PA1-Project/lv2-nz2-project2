"""Replay sampled wrist approach/return against archived sensor depth and robot CAD.

Unknown space remains unconfirmed. Passing samples cannot certify the complete
swept volume, depth uncertainty or live safety; this tool never commands motion.
"""
import argparse
import hashlib
import itertools
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src'), str(ROOT / 'scripts/sensor_runtime')]
from camera_rig import PreparedCameraRig
from audit_r12_usd_arm_path import CollisionModel


def bounded_joint_samples(start, goal, joint_step=.005):
    count = int(np.ceil(np.max(np.abs(goal-start)) / joint_step))
    return [start + np.clip(goal-start, -joint_step*i, joint_step*i)
            for i in range(count+1)]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--search', type=Path, required=True)
    ap.add_argument('--candidate', type=int, default=0)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    search = json.loads(args.search.read_text())
    frame = Path(search['frame'])
    meta = json.loads(frame.with_suffix('.json').read_text())
    digest = hashlib.sha256(frame.read_bytes()).hexdigest()
    if digest != search['frame_sha256'] or digest != meta['sha256']:
        raise ValueError('Frame identity mismatch')
    for name, digest in search['robot_configuration_sha256'].items():
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest() != digest:
            raise ValueError('Robot configuration changed: ' + name)
    with np.load(frame, allow_pickle=False) as archive:
        data = {key: archive[key] for key in archive.files}
    for key in ('run_id', 'frame_index', 'stamp_ns'):
        if data[key].item() != meta[key] or (key in search and meta[key] != search[key]):
            raise ValueError('Sensor identity mismatch: ' + key)
    q0 = np.asarray(meta['arm_measured_position'], float)
    if not 0 <= args.candidate < len(search['ranked_endpoint_candidates']):
        raise ValueError('Invalid candidate index')
    q1 = np.asarray(search['ranked_endpoint_candidates'][args.candidate]['q'], float)
    cfg = json.loads((ROOT/'configs/r12_robot.json').read_text())
    camera_cfg = json.loads((ROOT/'configs/sensors/camera_rig.json').read_text())
    if camera_cfg['resolution'] != [640, 480] or camera_cfg['intrinsics'] != [400, 400, 320, 240]:
        raise ValueError('Unsupported camera geometry for this diagnostic')
    if any(q.shape != (5,) or not np.isfinite(q).all() or
           np.any(q < cfg['lower']) or np.any(q > cfg['upper']) for q in (q0, q1)):
        raise ValueError('Invalid arm posture')
    if meta['arm_feedback_stamp_ns'] != meta['stamp_ns']:
        raise ValueError('Encoder/capture stamp mismatch')
    source = next(row for row in json.loads((frame.parent.parent/'ros_results.json').read_text())
                  if row['stamp_ns'] == meta['stamp_ns'] and row['capture_run_id'] == meta['run_id'])
    if not source['base_stopped'] or not source['arm_feedback_valid']:
        raise ValueError('Archive is not a settled observation')
    gripper = json.loads((ROOT/'configs/r12_gripper.json').read_text())
    nominal_aperture = gripper['cad_zero_aperture_m'] + 2*.02768613
    if abs(source['gripper_aperture_m']-nominal_aperture) > .0001:
        raise ValueError('Archive gripper aperture does not match CollisionModel open fingers')
    rig = PreparedCameraRig(ROOT).transforms(dict(zip(cfg['joint_names'], q0)))
    cameras, clouds = [], []
    for name in ('r07_camera1', 'r07_camera2', 'r07_camera3', 'arm_camera'):
        depth = data['depth'] if name == 'r07_camera1' else data[name+'_depth']
        camera = rig[name+'_left_optical']
        cameras.append((camera, depth))
        v, u = np.where(np.isfinite(depth) & (depth > .03) & (depth < 10))
        z = depth[v, u]
        cloud = np.column_stack(((u-320)*z/400, (v-240)*z/400, z))
        clouds.append(cloud @ camera[:3, :3].T + camera[:3, 3])
    cloud = np.concatenate(clouds)
    _, indices = np.unique(np.floor(cloud/.005).astype(np.int64), axis=0, return_index=True)
    cloud = cloud[indices]
    tree = cKDTree(cloud)
    model = CollisionModel()
    initial = model.transforms(q0)
    initial_boxes = [(initial[body]@local, model.sizes[name]/2)
                     for name, body, local, _ in model.boxes]
    probes = {}
    for name, body, _, _ in model.boxes:
        if body == 'base_link':
            continue
        half = model.sizes[name]/2 + .03
        axes = [np.linspace(-h, h, int(np.ceil(2*h/.02))+1) for h in half]
        probes[name] = np.asarray(list(itertools.product(*axes)))
    records, collisions, near_surfaces = [], [], []
    totals = dict(observed_free_external=0, unconfirmed_external=0, inside_start_model=0)
    min_surface = .2
    # Return has independent bounded-rate interpolation. Reversing the outward
    # samples would not reproduce different joints' arrival times.
    paths = [('approach', bounded_joint_samples(q0, q1)),
             ('return', bounded_joint_samples(q1, q0))]
    for direction, samples in paths:
        for index, q in enumerate(samples):
            hits = model.hits(q)
            if hits:
                collisions.append(dict(direction=direction, sample=index, pairs=hits))
            transforms = model.transforms(q)
            counts = dict.fromkeys(totals, 0)
            for name, body, local, _ in model.boxes:
                if body == 'base_link':
                    continue
                transform = transforms[body]@local
                half = model.sizes[name]/2
                ids = tree.query_ball_point(transform[:3, 3], np.linalg.norm(half)+.2)
                if ids:
                    distances = np.linalg.norm(np.maximum(
                        np.abs((cloud[ids]-transform[:3, 3])@transform[:3, :3])-half, 0), axis=1)
                    distance = float(distances.min())
                    min_surface = min(min_surface, distance)
                    if distance < .03 and len(near_surfaces) < 30:
                        near_surfaces.append(dict(direction=direction, sample=index,
                                                  collider=name, distance_m=distance))
                points = probes[name]@transform[:3, :3].T + transform[:3, 3]
                free = np.zeros(len(points), bool)
                for camera, depth in cameras:
                    pc = (points-camera[:3, 3])@camera[:3, :3]
                    z = pc[:, 2]
                    valid = z > .03
                    uv = np.zeros((len(points), 2), int)
                    uv[valid] = np.rint(400*pc[valid, :2]/z[valid, None]+[320, 240]).astype(int)
                    valid &= (uv[:, 0] >= 0) & (uv[:, 0] < 640) & (uv[:, 1] >= 0) & (uv[:, 1] < 480)
                    ix = np.where(valid)[0]
                    d = depth[uv[ix, 1], uv[ix, 0]]
                    free[ix] |= np.isfinite(d) & (d > .03) & (d < 10) & (z[ix] < d-.03)
                inside = np.zeros(len(points), bool)
                for box, half_start in initial_boxes:
                    inside |= np.all(np.abs((points-box[:3, 3])@box[:3, :3]) <= half_start, axis=1)
                counts['inside_start_model'] += int(inside.sum())
                counts['observed_free_external'] += int((free & ~inside).sum())
                counts['unconfirmed_external'] += int((~free & ~inside).sum())
            for key, value in counts.items():
                totals[key] += value
            records.append(dict(direction=direction, sample=index, q=q.tolist(), **counts))
    result = dict(source_search=str(args.search.resolve()), candidate=args.candidate,
                  frame_sha256=search['frame_sha256'], measured_gripper_aperture_m=source['gripper_aperture_m'],
                  nominal_gripper_aperture_m=nominal_aperture, sampled_poses=len(records),
                  self_collision_samples=collisions, observed_surface_proximity_examples=near_surfaces,
                  observed_surface_min_distance_capped_at_0_2_m=min_surface,
                  probe_counts=totals, samples=records, execution_authorized=False,
                  clearance_verified=False, runtime_integrated=False,
                  sampling=dict(max_joint_step_rad=.005, volume_grid_spacing_m=.02,
                                collider_expansion_m=.03, depth_margin_m=.03, surface_voxel_m=.005),
                  scope='Offline sensor-only depth with encoder FK and robot CAD; no fixture/PhysX world poses.',
                  limitations=['Unknown probes are not free space; starting-model interior is reported separately.',
                               'Grid/path sampling is not continuous swept-volume proof.',
                               'Proximity points may include residual self returns; no automatic masking or clearance acceptance.',
                               'Archived evidence and provisional calibration cannot authorize current arm motion.'])
    with args.output.open('x') as handle:
        json.dump(result, handle, indent=2)
        handle.write('\n')
    print(json.dumps(dict(sampled_poses=len(records), self_collision_samples=len(collisions),
                          probe_counts=totals, min_surface_distance_m=min_surface,
                          execution_authorized=False)))


if __name__ == '__main__':
    main()
