"""Compare masked/raw sensor evidence for wrist clearance visibility gaps.

Archived depth and estimated planar registration are diagnostic evidence only.
Raw robot surfaces terminate rays normally; no missing pixel is filled as free.
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'scripts/sensor_runtime')]
from arm_practice.depth_volume import depth_covers_voxels
from arm_practice.result_journal import read_committed_rows
from camera_rig import PreparedCameraRig
from audit_r12_wrist_multiview import map_transform


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--refined-audit',type=Path,required=True)
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError('Preserve earlier result')
    refined=json.loads(args.refined_audit.read_text())
    volume=json.loads(Path(refined['source_volume_audit']).read_text())
    path=json.loads(Path(volume['source_path_audit']).read_text())
    search=json.loads(Path(path['source_search']).read_text())
    reference=Path(search['frame']);ref_meta=json.loads(reference.with_suffix('.json').read_text())
    if hashlib.sha256(reference.read_bytes()).hexdigest()!=search['frame_sha256']:
        raise ValueError('Reference checksum changed')
    for name,digest in search['robot_configuration_sha256'].items():
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=digest:
            raise ValueError('Robot configuration changed')
    ref_rows=json.loads((reference.parent.parent/'ros_results.json').read_text())
    ref_row=next(row for row in ref_rows if row['stamp_ns']==ref_meta['stamp_ns'])
    map_from_reference=map_transform(ref_row['estimated_map_pose'])
    with np.load(args.refined_audit.with_suffix('.npz'),allow_pickle=False) as data:
        centers=data['unconfirmed_centers'];half=float(data['cell_width_m'])/2
    rows=(json.loads((args.run/'ros_results.json').read_text())
          if (args.run/'ros_results.json').exists()
          else read_committed_rows(args.run/'ros_results_progress.json'))
    by_stamp={row['stamp_ns']:row for row in rows}
    cfg=json.loads((ROOT/'configs/r12_robot.json').read_text());rig=PreparedCameraRig(ROOT)
    covered={key:np.zeros(len(centers),bool) for key in ['masked','raw']}
    records=[];run_id=None
    for capture in sorted((args.run/'observation_history').glob('*.npz')):
        meta=json.loads(capture.with_suffix('.json').read_text());row=by_stamp.get(meta['stamp_ns'])
        if row is None:continue
        if hashlib.sha256(capture.read_bytes()).hexdigest()!=meta['sha256']:
            raise ValueError('Frame checksum mismatch')
        if run_id is None:run_id=meta['run_id']
        if run_id!=meta['run_id'] or row['capture_run_id']!=run_id:
            raise ValueError('Mixed run')
        if not (row['arm_feedback_valid'] and row['encoder_valid'] and row['tracking']==1):continue
        if (meta['arm_feedback_stamp_ns']!=meta['stamp_ns'] or
                not np.array_equal(meta['arm_measured_position'],row['arm_measured_position'])):
            raise ValueError('Encoder/capture association mismatch')
        transforms=rig.transforms(dict(zip(cfg['joint_names'],meta['arm_measured_position'])))
        reference_from_base=np.linalg.inv(map_from_reference)@map_transform(row['estimated_map_pose'])
        with np.load(capture,allow_pickle=False) as data:
            for key in ['run_id','frame_index','stamp_ns']:
                if data[key].item()!=meta[key]:raise ValueError('Capture identity mismatch')
            for name in ['r07_camera1','r07_camera2','r07_camera3','arm_camera']:
                masked=data['depth' if name=='r07_camera1' else name+'_depth']
                raw=data[name+'_depth_unmasked']
                if masked.shape!=raw.shape or raw.shape!=(480,640):raise ValueError('Depth shape mismatch')
                unchanged=(masked!=0)&np.isfinite(masked)
                if not np.array_equal(masked[unchanged],raw[unchanged]):
                    raise ValueError('Nonmasked depth changed')
                matrix=reference_from_base@transforms[name+'_left_optical']
                counts={}
                for key,depth in [('masked',masked),('raw',raw)]:
                    free=depth_covers_voxels(centers,half,matrix,depth,exact_rectangle=True)
                    covered[key]|=free;counts[key]=int(free.sum())
                records.append(dict(frame=meta['frame_index'],camera=name,
                    original_gap_cells_observed=counts,
                    raw_valid_pixels_removed_by_mask=int(np.sum((masked==0)&np.isfinite(raw)&(raw>.03)&(raw<10)))))
    result=dict(source_refined_audit=str(args.refined_audit.resolve()),run=str(args.run.resolve()),
        capture_run_id=run_id,reference_gap_cells=len(centers),
        masked_covered_cells=int(covered['masked'].sum()),raw_covered_cells=int(covered['raw'].sum()),
        raw_unconfirmed_cells=int((~covered['raw']).sum()),records=records,
        execution_authorized=False,clearance_verified=False,
        source_code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        limitations=['Only the previous sampled-path visibility gaps, not full continuous swept volume.',
            'Different runs are compared using estimated map poses under a static-scene assumption.',
            'Archived freshness, provisional calibration and registration uncertainty do not authorize motion.'])
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ['records','limitations']}))


if __name__=='__main__':main()
