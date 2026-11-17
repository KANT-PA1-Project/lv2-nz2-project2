"""Classify whole-cell coverage gaps against sampled CAD+3cm using FCL distance.

This can remove discretization over-cover from a diagnostic count. It cannot
authorize motion or certify the between-sample swept volume.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import fcl
from audit_r12_usd_arm_path import CollisionModel


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--volume-audit',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    report=json.loads(args.volume_audit.read_text())
    path=json.loads(Path(report['source_path_audit']).read_text())
    search=json.loads(Path(path['source_search']).read_text())
    root=Path(__file__).resolve().parents[1]
    for name,digest in search['robot_configuration_sha256'].items():
        if hashlib.sha256((root/name).read_bytes()).hexdigest()!=digest:
            raise ValueError('Robot configuration changed')
    width=report['voxel_size_m']
    with np.load(args.volume_audit.with_suffix('.npz'),allow_pickle=False) as data:
        points=data['external_centers'][~data['historical_observed']]
    if len(points)!=report['unconfirmed_cells']:
        raise ValueError('Cell count mismatch')
    # Analytic disjoint/touching/overlapping cube fixtures verify FCL units/sign.
    a=fcl.CollisionObject(fcl.Box(1,1,1))
    fixture_distances=[]
    for x,expected in [(2.,1.),(1.,0.),(.5,0.)]:
        b=fcl.CollisionObject(fcl.Box(1,1,1),fcl.Transform(np.eye(3),[x,0,0]))
        distance=max(0.,fcl.distance(a,b,fcl.DistanceRequest(),fcl.DistanceResult()))
        if not np.isclose(distance,expected,atol=1e-7):
            raise ValueError('FCL distance fixture failed')
        fixture_distances.append(distance)
    objects=[fcl.CollisionObject(fcl.Box(width,width,width),fcl.Transform(np.eye(3),point))
             for point in points]
    minimum=np.full(len(points),np.inf)
    nearest=[None]*len(points)
    calls=0
    model=CollisionModel()
    for record in path['samples']:
        transforms=model.transforms(record['q'])
        for name,body,local,obj in model.boxes:
            if body=='base_link':
                continue
            transform=transforms[body]@local
            local_points=(points-transform[:3,3])@transform[:3,:3]
            point_dist=np.linalg.norm(np.maximum(np.abs(local_points)-model.sizes[name]/2,0),axis=1)
            # Distance to a set is1-Lipschitz; other cells cannot intersect3cm.
            candidate=np.flatnonzero(point_dist<=.03+np.sqrt(3)*width/2+1e-9)
            obj.setTransform(fcl.Transform(transform[:3,:3],transform[:3,3]))
            for i in candidate:
                distance=max(0.,fcl.distance(obj,objects[i],fcl.DistanceRequest(),fcl.DistanceResult()))
                calls+=1
                if distance<minimum[i]:
                    minimum[i]=distance
                    nearest[i]=dict(collider=name,direction=record['direction'],sample=record['sample'])
    needed=minimum<=.03+1e-7
    result=dict(source_volume_audit=str(args.volume_audit.resolve()),unknown_whole_cells=len(points),
        intersect_sampled_3cm_volume=int(needed.sum()),discretization_only_cells=int((~needed).sum()),
        exact_distance_queries=calls,analytic_distance_fixture_results_m=fixture_distances,
        cells=[dict(center_m=point.tolist(),intersects_sampled_clearance=bool(required),
                    distance_m=None if not np.isfinite(distance) else float(distance),nearest_sample=sample)
               for point,required,distance,sample in zip(points,needed,minimum,nearest)],
        execution_authorized=False,clearance_verified=False,runtime_integrated=False,
        limitations=['Sampled-pose distance only; continuous motion and servo errors unbounded.',
                     'Remaining cells are unobserved, not demonstrated occupied.',
                     'Historical image union does not establish current scene clearance.'])
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='cells'}))


if __name__=='__main__':
    main()
