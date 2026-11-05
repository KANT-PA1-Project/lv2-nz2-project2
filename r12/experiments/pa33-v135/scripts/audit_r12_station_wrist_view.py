"""Offline RGB-D view search; never grants motion or confirms a puck identity.

Inputs are one preserved sensor frame and its capture-time arm encoder record.
Robot CAD is used for kinematics/self-collision only. No fixture poses, object
labels or simulated base truth are read. Environment/swept clearance is absent.
"""
import argparse
import hashlib
import itertools
import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src'), str(ROOT / 'scripts/sensor_runtime')]
from camera_rig import PreparedCameraRig
from audit_r12_usd_arm_path import CollisionModel


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frame', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    meta = json.loads(args.frame.with_suffix('.json').read_text())
    digest = hashlib.sha256(args.frame.read_bytes()).hexdigest()
    if digest != meta['sha256']:
        raise ValueError('Archived frame checksum mismatch')
    with np.load(args.frame, allow_pickle=False) as archive:
        data = {key: archive[key] for key in archive.files}
    for key in ('run_id', 'frame_index', 'stamp_ns'):
        if data[key].item() != meta[key]:
            raise ValueError('Archive metadata mismatch: ' + key)
    if meta['arm_feedback_stamp_ns'] != meta['stamp_ns']:
        raise ValueError('Arm encoder/frame stamp mismatch')
    q0 = np.asarray(meta['arm_measured_position'], float)
    if q0.shape != (5,) or not np.isfinite(q0).all():
        raise ValueError('Invalid measured arm posture')
    cfg = json.loads((ROOT / 'configs/r12_robot.json').read_text())
    rig = PreparedCameraRig(ROOT)
    transforms = rig.transforms(dict(zip(cfg['joint_names'], q0)))
    sensor_stats, hypotheses = {}, []
    for name in ('r07_camera1', 'r07_camera2', 'r07_camera3', 'arm_camera'):
        rgb = data['rgb'][0] if name == 'r07_camera1' else data[name + '_left_optical']
        depth = data['depth'] if name == 'r07_camera1' else data[name + '_depth']
        camera = transforms[name + '_left_optical']
        valid = np.isfinite(depth) & (depth > .03) & (depth < 10)
        sensor_stats[name] = dict(rgb_std=float(rgb.std()),
                                 valid_depth_fraction=float(valid.mean()),
                                 optical_forward_base=camera[:3, 2].tolist(),
                                 camera_origin_base=camera[:3, 3].tolist())
        if name == 'arm_camera':
            continue
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        bright = (hsv[:, :, 1] > 140) & (hsv[:, :, 2] > 60)
        colors = {'R': (hsv[:, :, 0] < 10) | (hsv[:, :, 0] > 170),
                  'G': (hsv[:, :, 0] >= 40) & (hsv[:, :, 0] <= 90)}
        for color, hue in colors.items():
            n, labels, stats, _ = cv2.connectedComponentsWithStats(
                (bright & hue & valid).astype(np.uint8), connectivity=8)
            for index in range(1, n):
                x, y, w, h, area = (int(v) for v in stats[index])
                # Ignore tiny or clipped regions; colored structure can still
                # pass these filters, so hypotheses never become object IDs.
                if not 25 <= area <= 5000 or x == 0 or y == 0 or x+w == 640 or y+h == 480:
                    continue
                v, u = np.where(labels == index)
                z = depth[v, u]
                camera_points = np.column_stack(((u-320)*z/400, (v-240)*z/400, z))
                center = np.median(camera_points, axis=0)
                point = camera[:3, :3] @ center + camera[:3, 3]
                hypotheses.append(dict(camera=name, color_hypothesis=color,
                                       bbox=[x, y, w, h], pixels=area,
                                       surface_base_m=point.tolist(),
                                       depth_median_m=float(np.median(z)),
                                       depth_iqr_m=float(np.percentile(z, 75)-np.percentile(z, 25)),
                                       role='anonymous_color_depth_region_not_object_or_id'))
    model = CollisionModel()
    axes = [np.linspace(lo, hi, count) for lo, hi, count in
            zip(cfg['lower'], cfg['upper'], (9, 5, 5, 5, 3))]
    endpoints = []
    tested, visible, free_self = 0, 0, 0
    for values in itertools.product(*axes):
        q = np.asarray(values)
        camera = rig.transforms(dict(zip(cfg['joint_names'], q)))['arm_camera_left_optical']
        tested += 1
        candidates = []
        for index, target in enumerate(hypotheses):
            pc = camera[:3, :3].T @ (np.asarray(target['surface_base_m'])-camera[:3, 3])
            if not .12 < pc[2] < .6:
                continue
            uv = 400*pc[:2]/pc[2] + [320, 240]
            if 30 < uv[0] < 610 and 30 < uv[1] < 450:
                candidates.append(dict(hypothesis_index=index, pixel=uv.tolist(),
                                       optical_depth_m=float(pc[2]),
                                       nominal_20mm_feature_pixels=float(8/pc[2]),
                                       score=float(np.linalg.norm((uv-[320, 240])/[320, 240]))))
        if not candidates:
            continue
        visible += 1
        if model.hits(q):
            continue
        free_self += 1
        endpoints.extend(dict(q=q.tolist(), **target) for target in candidates)
    endpoints.sort(key=lambda row: row['score'])
    result = dict(frame=str(args.frame.resolve()), frame_sha256=digest,
                  run_id=meta['run_id'], frame_index=meta['frame_index'],
                  arm_measured_position=q0.tolist(), sensor_statistics=sensor_stats,
                  color_depth_hypotheses=hypotheses, tested_postures=tested,
                  postures_with_target_in_fov=visible, self_collision_free_endpoints=free_self,
                  ranked_endpoint_candidates=endpoints[:10], execution_authorized=False,
                  runtime_integrated=False, identity_confirmed=False,
                  required_next=['Validate arm/gripper CAD + 3cm environmental clearance and swept approach/return paths.',
                                 'Reacquire candidate from fresh wrist images after verified base/arm settling.'],
                  limitations=['Archived evidence cannot establish current freshness.',
                               'Color regions are not trained object detections or decoded IDs.',
                               'Grid endpoints do not validate paths, occlusion or environmental clearance.',
                               'Robot model joint ranges and provisional camera calibration are not hardware certification.',
                               'CollisionModel uses nominal open gripper geometry; current aperture agreement must be validated before motion.'])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as handle:
        json.dump(result, handle, indent=2)
        handle.write('\n')
    print(json.dumps({key: result[key] for key in ('frame_index', 'tested_postures',
                     'postures_with_target_in_fov', 'self_collision_free_endpoints', 'execution_authorized')}))


if __name__ == '__main__':
    main()
