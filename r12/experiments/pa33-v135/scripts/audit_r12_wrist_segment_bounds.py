"""Bound nominal between-sample collider motion; no environment or live permission."""
import argparse
import hashlib
import itertools
import json
from pathlib import Path
import numpy as np
from audit_r12_usd_arm_path import CollisionModel
from arm_practice.kinematic_displacement import collider_joint_radii


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--path-audit', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError('Preserve previous audits')
    path = json.loads(args.path_audit.read_text())
    search = json.loads(Path(path['source_search']).read_text())
    root = Path(__file__).resolve().parents[1]
    for name, digest in search['robot_configuration_sha256'].items():
        if hashlib.sha256((root/name).read_bytes()).hexdigest() != digest:
            raise ValueError('Robot configuration changed')
    model = CollisionModel()
    names = [f'arm_joint{i+1}' for i in range(5)]
    fixed = {'gripper_left_joint': .02768613, 'gripper_right_joint': -.02768613}
    radius = np.asarray([collider_joint_radii(model.joints, body, local, model.sizes[name], names, fixed)
                         for name, body, local, _ in model.boxes])
    signs = np.asarray(list(itertools.product([-1, 1], repeat=3)))
    local_corners = [signs*model.sizes[name]/2 for name, _, _, _ in model.boxes]
    def corners(q):
        transforms = model.transforms(q)
        result = []
        for (_, body, local, _), points in zip(model.boxes, local_corners):
            matrix = transforms[body] @ local
            result.append(points @ matrix[:3, :3].T + matrix[:3, 3])
        return np.asarray(result)
    rng = np.random.default_rng(421)
    maximum_per_collider = np.zeros(len(model.boxes))
    checks, worst_ratio, records = 0, 0., []
    for a, b in zip(path['samples'], path['samples'][1:]):
        if a['direction'] != b['direction']:
            continue
        start, end = np.asarray(a['q']), np.asarray(b['q'])
        excursion = np.abs(end-start)
        bound = radius @ excursion
        maximum_per_collider = np.maximum(maximum_per_collider, bound)
        initial = corners(start)
        # Numeric regressions exercise independently moving joints inside the
        # interval box; the guarantee comes from the radius/angle inequality.
        for fraction in [np.ones(5), np.full(5, .5), rng.random(5), rng.random(5)]:
            displacement = np.linalg.norm(corners(start+(end-start)*fraction)-initial, axis=2).max(axis=1)
            if np.any(displacement > bound+1e-9):
                raise ValueError('Kinematic displacement exceeds analytic bound')
            nonzero = bound > 1e-12
            if nonzero.any():
                worst_ratio = max(worst_ratio, float((displacement[nonzero]/bound[nonzero]).max()))
            checks += len(model.boxes)*8
        records.append(dict(direction=a['direction'], start_sample=a['sample'], end_sample=b['sample'],
                            maximum_joint_excursion_rad=excursion.tolist(),
                            maximum_point_displacement_bound_m=float(bound.max())))
    result = dict(source_path_audit=str(args.path_audit.resolve()), intervals=len(records),
                  active_colliders=len(model.boxes), fixed_prismatic_positions=fixed,
                  maximum_point_displacement_bound_m=float(maximum_per_collider.max()),
                  corner_regression_checks=checks, maximum_observed_displacement_to_bound_ratio=worst_ratio,
                  intervals_detail=records,
                  collider_bounds=[dict(collider=name, body=body, joint_radii_m=r.tolist(),
                                        maximum_segment_bound_m=float(d))
                                   for (name, body, _, _), r, d in zip(model.boxes, radius, maximum_per_collider)],
                  execution_authorized=False, clearance_verified=False, runtime_integrated=False,
                  source_code_sha256={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest()
                                      for p in [Path(__file__), root/'src/arm_practice/kinematic_displacement.py']},
                  limitations=['Only fixed-base CAD with fixed open fingers and joints remaining inside each nominal interval box.',
                               'The environment test must expand each reference collider by3cm plus its segment bound.',
                               'Self-pair distances need both colliders displacement bounds; this report does not test them.',
                               'Servo overshoot, encoder uncertainty, base drift, current depth and calibration errors remain unbounded.'])
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k: result[k] for k in ['intervals', 'active_colliders', 'maximum_point_displacement_bound_m',
                                            'corner_regression_checks', 'maximum_observed_displacement_to_bound_ratio',
                                            'execution_authorized']}))


if __name__ == '__main__':
    main()
