"""Subdivide archived wrist coverage gaps without treating unknown space as free.

Only sampled poses are covered. No live clearance or motion permission results.
"""
import argparse
import hashlib
import itertools
import json
from pathlib import Path
import fcl
import numpy as np
from audit_r12_wrist_multiview import ROOT, CollisionModel, PreparedCameraRig, map_transform
from arm_practice.depth_volume import depth_covers_voxels


def intersects_clearance(centers, width, boxes, clearance=.03):
    needed=np.zeros(len(centers),bool)
    objects=[fcl.CollisionObject(fcl.Box(width,width,width),fcl.Transform(np.eye(3),p))
             for p in centers]
    queries=0
    for matrix,size in boxes:
        ids=np.flatnonzero(~needed)
        local=(centers[ids]-matrix[:3,3])@matrix[:3,:3]
        distance=np.linalg.norm(np.maximum(np.abs(local)-size/2,0),axis=1)
        ids=ids[distance<=clearance+np.sqrt(3)*width/2+1e-9]
        if not len(ids):
            continue
        obj=fcl.CollisionObject(fcl.Box(*size),fcl.Transform(matrix[:3,:3],matrix[:3,3]))
        for i in ids:
            d=fcl.distance(obj,objects[i],fcl.DistanceRequest(),fcl.DistanceResult())
            if not np.isfinite(d):
                raise ValueError('Nonfinite FCL result')
            needed[i]=d<=clearance+1e-7
            queries+=1
    return needed,queries


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--volume-audit',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--levels',type=int,choices=[1,2,3],default=2)
    parser.add_argument('--exact-rectangle',action='store_true')
    args=parser.parse_args()
    if args.output.exists() or args.output.with_suffix('.npz').exists():
        raise FileExistsError('Preserve previous audits')
    report=json.loads(args.volume_audit.read_text())
    audit=json.loads(Path(report['source_path_audit']).read_text())
    search=json.loads(Path(audit['source_search']).read_text())
    for name,digest in search['robot_configuration_sha256'].items():
        if hashlib.sha256((ROOT/name).read_bytes()).hexdigest()!=digest:
            raise ValueError('Robot configuration changed')
    reference=Path(search['frame']);meta=json.loads(reference.with_suffix('.json').read_text())
    if hashlib.sha256(reference.read_bytes()).hexdigest()!=search['frame_sha256']:
        raise ValueError('Reference checksum mismatch')
    rows={r['stamp_ns']:r for r in json.loads((reference.parent.parent/'ros_results.json').read_text())}
    world_to_reference=np.linalg.inv(map_transform(rows[meta['stamp_ns']]['estimated_map_pose']))
    rig=PreparedCameraRig(ROOT)
    config=json.loads((ROOT/'configs/r12_robot.json').read_text())
    cameras=[];sources=[]
    for file in sorted(reference.parent.glob('*.npz')):
        if not file.stem.isdigit() or int(file.stem)>meta['frame_index']:
            continue
        fm=json.loads(file.with_suffix('.json').read_text());row=rows[fm['stamp_ns']]
        if hashlib.sha256(file.read_bytes()).hexdigest()!=fm['sha256']:
            raise ValueError('Frame checksum mismatch')
        if fm['run_id']!=meta['run_id'] or row['capture_run_id']!=fm['run_id']:
            raise ValueError('Capture run mismatch')
        if not(row['encoder_valid'] and row['arm_feedback_valid'] and row['tracking']==1):
            continue
        if fm['arm_feedback_stamp_ns']!=fm['stamp_ns'] or not np.allclose(
                fm['arm_measured_position'],row['arm_measured_position'],atol=1e-10,rtol=0):
            raise ValueError('Encoder pose/capture association mismatch')
        transforms=rig.transforms(dict(zip(config['joint_names'],fm['arm_measured_position'])))
        relative=world_to_reference@map_transform(row['estimated_map_pose'])
        with np.load(file,allow_pickle=False) as data:
            for key in ('run_id','frame_index','stamp_ns'):
                if data[key].item()!=fm[key]:
                    raise ValueError('Capture bundle mismatch')
            for name in ('r07_camera1','r07_camera2','r07_camera3','arm_camera'):
                cameras.append((relative@transforms[name+'_left_optical'],
                    data['depth' if name=='r07_camera1' else name+'_depth']))
        sources.append(dict(frame=fm['frame_index'],sha256=fm['sha256']))
    model=CollisionModel();boxes=[]
    for sample in audit['samples']:
        transforms=model.transforms(sample['q'])
        boxes.extend((transforms[body]@local,model.sizes[name])
                     for name,body,local,_ in model.boxes if body!='base_link')
    with np.load(args.volume_audit.with_suffix('.npz'),allow_pickle=False) as data:
        unknown=data['external_centers'][~data['historical_observed']]
    width=report['voxel_size_m'];levels=[]
    signs=np.asarray(list(itertools.product([-1,1],repeat=3)))
    initial=model.transforms(meta['arm_measured_position'])
    for level in range(args.levels):
        centers=(unknown[:,None,:]+signs[None,:,:]*width/4).reshape(-1,3)
        width/=2
        corners=centers[:,None,:]+signs[None,:,:]*width/2
        inside=np.zeros(len(centers),bool)
        for name,body,local,_ in model.boxes:
            matrix=initial[body]@local
            inside|=np.all(np.abs((corners-matrix[:3,3])@matrix[:3,:3])<=model.sizes[name]/2,axis=(1,2))
        outside_self=centers[~inside]
        needed,queries=intersects_clearance(outside_self,width,boxes)
        required=outside_self[needed]
        free=np.zeros(len(required),bool)
        for matrix,depth in cameras:
            ids=np.flatnonzero(~free)
            if not len(ids):
                break
            free[ids]=depth_covers_voxels(required[ids],width/2,matrix,depth,
                                        exact_rectangle=args.exact_rectangle)
        unknown=required[~free]
        levels.append(dict(level=level+1,cell_width_m=width,subcells=len(centers),
            whole_initial_self_cells=int(inside.sum()),outside_sampled_clearance_cells=int((~needed).sum()),
            sampled_clearance_cells=len(required),observed_whole_cells=int(free.sum()),
            unconfirmed_cells=len(unknown),unconfirmed_volume_upper_bound_m3=len(unknown)*width**3,
            exact_distance_queries=queries))
        if not len(unknown):
            break
    np.savez_compressed(args.output.with_suffix('.npz'),unconfirmed_centers=unknown,cell_width_m=width)
    result=dict(source_volume_audit=str(args.volume_audit.resolve()),sources=sources,levels=levels,
        depth_query='exact_rectangle' if args.exact_rectangle else 'enclosing_square',
        source_code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        execution_authorized=False,clearance_verified=False,runtime_integrated=False,
        limitations=['Each retained cube needs complete projected pixel coverage in at least one archived view.',
            'Cubes outside sampled CAD+3cm or wholly in initial CAD are accounted separately, not marked observed.',
            'Between-pose sweep and servo tracking remain unbounded.',
            'Static scene, planar estimated map registration and provisional calibration assumed; no live permission.'])
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='sources'}))


if __name__=='__main__':
    main()
