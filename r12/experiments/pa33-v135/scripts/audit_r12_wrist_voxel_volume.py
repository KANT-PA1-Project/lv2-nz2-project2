"""Offline whole-cell depth coverage of sampled arm poses; never authorizes motion."""
import argparse
import hashlib
import itertools
import json
from pathlib import Path

import numpy as np
from audit_r12_wrist_multiview import ROOT, CollisionModel, PreparedCameraRig, map_transform
from arm_practice.depth_volume import obb_volume_cells, depth_covers_voxels


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--path-audit', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--voxel-size', type=float, default=.01)
    args = parser.parse_args()
    if not .005 <= args.voxel_size <= .02:
        raise ValueError('Voxel size must be 5 to 20mm')
    if args.output.exists() or args.output.with_suffix('.npz').exists():
        raise FileExistsError('Preserve prior audit output')
    audit = json.loads(args.path_audit.read_text())
    search = json.loads(Path(audit['source_search']).read_text())
    reference = Path(search['frame'])
    reference_meta = json.loads(reference.with_suffix('.json').read_text())
    if hashlib.sha256(reference.read_bytes()).hexdigest() != search['frame_sha256']:
        raise ValueError('Reference checksum mismatch')
    for name, digest in search['robot_configuration_sha256'].items():
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest() != digest:
            raise ValueError('Robot configuration changed')
    config = json.loads((ROOT/'configs/r12_robot.json').read_text())
    camera_config = json.loads((ROOT/'configs/sensors/camera_rig.json').read_text())
    if camera_config['intrinsics'] != [400,400,320,240] or camera_config['resolution'] != [640,480]:
        raise ValueError('Unsupported camera calibration')
    rows = {row['stamp_ns']: row for row in json.loads(
        (reference.parent.parent/'ros_results.json').read_text())}
    world_to_reference = np.linalg.inv(map_transform(rows[reference_meta['stamp_ns']]['estimated_map_pose']))
    model, rig = CollisionModel(), PreparedCameraRig(ROOT)
    cells = np.empty((0,3), dtype=int)
    for sample in audit['samples']:
        transforms = model.transforms(sample['q'])
        additions = [obb_volume_cells(transforms[body]@local, model.sizes[name],
                                     args.voxel_size, .03)
                     for name,body,local,_ in model.boxes if body != 'base_link']
        cells = np.unique(np.concatenate([cells,*additions]), axis=0)
    centers = (cells+.5)*args.voxel_size
    signs = np.asarray(list(itertools.product([-1,1], repeat=3)))
    corners = centers[:,None,:]+signs[None,:,:]*args.voxel_size/2
    initial_self = np.zeros(len(cells), bool)
    start = model.transforms(reference_meta['arm_measured_position'])
    for name,body,local,_ in model.boxes:
        transform = start[body]@local
        local_corners = (corners-transform[:3,3])@transform[:3,:3]
        initial_self |= np.all(np.abs(local_corners) <= model.sizes[name]/2, axis=(1,2))
    external = centers[~initial_self]
    observed = np.zeros(len(external), bool)
    views, current_free = [], None
    for file in sorted(reference.parent.glob('*.npz')):
        if not file.stem.isdigit() or int(file.stem) > reference_meta['frame_index']:
            continue
        meta = json.loads(file.with_suffix('.json').read_text())
        if hashlib.sha256(file.read_bytes()).hexdigest() != meta['sha256']:
            raise ValueError('Frame checksum mismatch')
        row = rows[meta['stamp_ns']]
        if meta['run_id'] != reference_meta['run_id'] or row['capture_run_id'] != meta['run_id']:
            raise ValueError('Run identity mismatch')
        if not row['encoder_valid'] or row['tracking'] != 1 or not row['arm_feedback_valid']:
            continue
        transforms = rig.transforms(dict(zip(config['joint_names'], meta['arm_measured_position'])))
        relative = world_to_reference@map_transform(row['estimated_map_pose'])
        free = np.zeros(len(external), bool)
        counts = {}
        with np.load(file, allow_pickle=False) as frame:
            for key in ('run_id','frame_index','stamp_ns'):
                if frame[key].item() != meta[key]:
                    raise ValueError('Capture metadata mismatch')
            for name in ('r07_camera1','r07_camera2','r07_camera3','arm_camera'):
                depth = frame['depth' if name == 'r07_camera1' else name+'_depth']
                camera_free = depth_covers_voxels(external, args.voxel_size/2,
                    relative@transforms[name+'_left_optical'], depth)
                free |= camera_free
                counts[name] = int(camera_free.sum())
        new = free & ~observed
        observed |= free
        views.append(dict(frame=meta['frame_index'], sha256=meta['sha256'],
                          whole_cells_by_camera=counts, new_whole_cells=int(new.sum())))
        if file == reference:
            current_free = int(free.sum())
    unknown = external[~observed]
    np.savez_compressed(args.output.with_suffix('.npz'), external_centers=external,
                        historical_observed=observed, initial_self_centers=centers[initial_self])
    result = dict(source_path_audit=str(args.path_audit.resolve()), voxel_size_m=args.voxel_size,
        sampled_poses=len(audit['samples']), total_cells=len(cells),
        initial_self_interior_cells=int(initial_self.sum()), external_cells=len(external),
        reference_only_whole_cells=current_free, historical_union_whole_cells=int(observed.sum()),
        unconfirmed_cells=len(unknown), unknown_bounds_m=None if not len(unknown) else
        [unknown.min(axis=0).tolist(),unknown.max(axis=0).tolist()], views=views,
        execution_authorized=False, clearance_verified=False, runtime_integrated=False,
        source_code_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
                            for p in (Path(__file__),ROOT/'src/arm_practice/depth_volume.py')},
        limitations=['Whole-cell over-cover of CAD plus Euclidean 3cm at sampled poses only.',
            'Between-pose sweep and servo tracking uncertainty are NOT bounded.',
            'Whole projected rectangle tested with conservative minimum depth; pixel depth model assumed.',
            'Historical union assumes static scene and planar estimated map registration, base Z=0.',
            'Archived frames, provisional extrinsics and unbounded registration error cannot authorize live motion.'])
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k != 'views'}))


if __name__ == '__main__':
    main()
