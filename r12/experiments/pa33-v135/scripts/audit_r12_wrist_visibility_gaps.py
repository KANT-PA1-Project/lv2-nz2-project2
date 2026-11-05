"""Explain archived wrist volume visibility gaps without granting motion."""
import argparse
import hashlib
import itertools
import json
from pathlib import Path
import numpy as np
from audit_r12_wrist_multiview import ROOT, CollisionModel, PreparedCameraRig, map_transform


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--refined-audit', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError('Preserve earlier evidence')
    refined = json.loads(args.refined_audit.read_text())
    volume = json.loads(Path(refined['source_volume_audit']).read_text())
    path = json.loads(Path(volume['source_path_audit']).read_text())
    search = json.loads(Path(path['source_search']).read_text())
    frame = Path(search['frame'])
    meta = json.loads(frame.with_suffix('.json').read_text())
    if hashlib.sha256(frame.read_bytes()).hexdigest() != search['frame_sha256']:
        raise ValueError('Reference changed')
    for name, digest in search['robot_configuration_sha256'].items():
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest() != digest:
            raise ValueError('Robot configuration changed')
    with np.load(args.refined_audit.with_suffix('.npz'), allow_pickle=False) as data:
        centers = data['unconfirmed_centers']
        width = float(data['cell_width_m'])
    signs = np.asarray(list(itertools.product([-1, 1], repeat=3)))
    corners = centers[:, None, :] + signs[None, :, :]*width/2
    rows = {r['stamp_ns']: r for r in json.loads((frame.parent.parent/'ros_results.json').read_text())}
    reference_inverse = np.linalg.inv(map_transform(rows[meta['stamp_ns']]['estimated_map_pose']))
    rig = PreparedCameraRig(ROOT)
    config = json.loads((ROOT/'configs/r12_robot.json').read_text())
    evidence = [[] for _ in centers]
    sources = []
    for source in refined['sources']:
        capture = frame.parent/f"{source['frame']:06d}.npz"
        fm = json.loads(capture.with_suffix('.json').read_text())
        if hashlib.sha256(capture.read_bytes()).hexdigest() != source['sha256'] or fm['run_id'] != meta['run_id']:
            raise ValueError('Archived input changed')
        row = rows[fm['stamp_ns']]
        if not (row['encoder_valid'] and row['arm_feedback_valid'] and row['tracking'] == 1):
            raise ValueError('Invalid archived sensors')
        if row['capture_run_id'] != fm['run_id'] or fm['arm_feedback_stamp_ns'] != fm['stamp_ns']:
            raise ValueError('Capture association changed')
        if not np.allclose(fm['arm_measured_position'], row['arm_measured_position'], atol=1e-10, rtol=0):
            raise ValueError('Encoder association changed')
        transforms = rig.transforms(dict(zip(config['joint_names'], fm['arm_measured_position'])))
        relative = reference_inverse @ map_transform(row['estimated_map_pose'])
        with np.load(capture, allow_pickle=False) as data:
            for key in ('run_id', 'frame_index', 'stamp_ns'):
                if data[key].item() != fm[key]:
                    raise ValueError('Capture bundle changed')
            for name in ('r07_camera1', 'r07_camera2', 'r07_camera3', 'arm_camera'):
                matrix = relative @ transforms[name+'_left_optical']
                pc = (corners-matrix[:3, 3]) @ matrix[:3, :3]
                depth = data['depth' if name == 'r07_camera1' else name+'_depth']
                for index, cube in enumerate(pc):
                    if np.min(cube[:, 2]) <= .03:
                        continue
                    uv = 400*cube[:, :2]/cube[:, 2, None] + [320, 240]
                    lower = np.floor(uv.min(axis=0)).astype(int)
                    upper = np.ceil(uv.max(axis=0)).astype(int)
                    if np.any(lower < 0) or np.any(upper >= [640, 480]):
                        continue
                    rectangle = depth[lower[1]:upper[1]+1, lower[0]:upper[0]+1]
                    valid = np.isfinite(rectangle) & (rectangle > .03) & (rectangle < 10)
                    item = dict(frame=fm['frame_index'], camera=name,
                                rectangle=[lower.tolist(), upper.tolist()],
                                valid_fraction=float(valid.mean()))
                    if valid.all():
                        item.update(minimum_depth_m=float(rectangle.min()),
                                    farthest_corner_z_m=float(cube[:, 2].max()),
                                    depth_clearance_m=float(rectangle.min()-cube[:, 2].max()))
                    evidence[index].append(item)
        sources.append(source)
    model = CollisionModel()
    initial = model.transforms(meta['arm_measured_position'])
    initial_distance = np.full(len(centers), np.inf)
    nearest_body = [None]*len(centers)
    for name, body, local, _ in model.boxes:
        transform = initial[body] @ local
        local_points = (centers-transform[:3, 3]) @ transform[:3, :3]
        distance = np.linalg.norm(np.maximum(np.abs(local_points)-model.sizes[name]/2, 0), axis=1)
        for i in np.flatnonzero(distance < initial_distance):
            initial_distance[i] = distance[i]
            nearest_body[i] = body
    result = []
    for i, views in enumerate(evidence):
        valid = [v for v in views if 'depth_clearance_m' in v]
        best = max(valid, key=lambda v: v['depth_clearance_m']) if valid else None
        result.append(dict(center_m=centers[i].tolist(), initial_cad_center_distance_m=float(initial_distance[i]),
                           initial_nearest_body=nearest_body[i], views_in_fov=len(views),
                           views_with_whole_valid_depth=len(valid), best_whole_valid_view=best,
                           largest_valid_fraction=max((v['valid_fraction'] for v in views), default=0.)))
    report = dict(source_refined_audit=str(args.refined_audit.resolve()), sources=sources,
                  cell_width_m=width, cells=result, execution_authorized=False,
                  runtime_integrated=False, clearance_verified=False,
                  source_code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  limitations=['Archived planar registration and provisional calibration remain assumptions.',
                               'A depth clearance below3cm does not pass the existing whole-cell check.',
                               'Initial-CAD center distances explain geometry, not full-cell or continuous clearance.'])
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    gaps = [v['best_whole_valid_view']['depth_clearance_m'] for v in result if v['best_whole_valid_view']]
    print(json.dumps(dict(cells=len(result), whole_valid_depth_cells=len(gaps),
                         best_depth_clearance_range_m=[min(gaps), max(gaps)] if gaps else None,
                         nearest_bodies=sorted(set(nearest_body)), execution_authorized=False)))


if __name__ == '__main__':
    main()
