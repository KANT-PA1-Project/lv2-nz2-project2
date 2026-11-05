"""Conservative self-separation check between nominal wrist path samples.

Pair distances at each interval start exceed the sum of both point-displacement
bounds, or the interval stays unconfirmed. Environment clearance is separate.
"""
import argparse
import hashlib
import json
from pathlib import Path
import fcl
import numpy as np
from audit_r12_usd_arm_path import CollisionModel
from arm_practice.kinematic_displacement import collider_joint_radii


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--path-audit', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError('Preserve earlier results')
    path = json.loads(args.path_audit.read_text())
    search = json.loads(Path(path['source_search']).read_text())
    root = Path(__file__).resolve().parents[1]
    for name, digest in search['robot_configuration_sha256'].items():
        if hashlib.sha256((root/name).read_bytes()).hexdigest() != digest:
            raise ValueError('Robot CAD configuration changed')
    model = CollisionModel()
    fixed = {'gripper_left_joint': .02768613, 'gripper_right_joint': -.02768613}
    radii = np.asarray([collider_joint_radii(model.joints, body, local, model.sizes[name],
                                           [f'arm_joint{i+1}' for i in range(5)], fixed)
                        for name, body, local, _ in model.boxes])
    pairs = np.asarray([(i, j) for i, (_, body, _, _) in enumerate(model.boxes)
                        for j in range(i+1, len(model.boxes))
                        if body != model.boxes[j][1] and frozenset((body, model.boxes[j][1])) not in model.skip])
    left, right = pairs.T
    half = np.asarray([model.sizes[name]/2 for name, _, _, _ in model.boxes])
    intervals, unresolved, exact_calls, broad_certified = [], [], 0, 0
    minimum_certified_margin = np.inf
    for start, end in zip(path['samples'], path['samples'][1:]):
        if start['direction'] != end['direction']:
            continue
        transforms = model.transforms(start['q'])
        matrices = np.asarray([transforms[body] @ local for _, body, local, _ in model.boxes])
        centers = matrices[:, :3, 3]
        extents = np.einsum('nij,nj->ni', np.abs(matrices[:, :3, :3]), half)
        aabb_distance = np.linalg.norm(np.maximum(np.abs(centers[left]-centers[right])
                                                 -extents[left]-extents[right], 0), axis=1)
        motion = radii @ np.abs(np.asarray(end['q'])-np.asarray(start['q']))
        budget = motion[left]+motion[right]
        margin = aabb_distance-budget
        # AABB supersets yield a lower distance bound; only ambiguous pairs need FCL.
        candidates = np.flatnonzero(margin <= 1e-7)
        broad_certified += len(pairs)-len(candidates)
        for i, (_, _, _, obj) in enumerate(model.boxes):
            matrix = matrices[i]
            obj.setTransform(fcl.Transform(matrix[:3, :3], matrix[:3, 3]))
        bad = []
        for pair_index in candidates:
            i, j = pairs[pair_index]
            distance = fcl.distance(model.boxes[i][3], model.boxes[j][3],
                                    fcl.DistanceRequest(), fcl.DistanceResult())
            if not np.isfinite(distance):
                raise ValueError('Nonfinite FCL separation')
            margin[pair_index] = max(0., distance)-budget[pair_index]
            exact_calls += 1
            if margin[pair_index] <= 1e-7:
                bad.append(dict(colliders=[model.boxes[i][0], model.boxes[j][0]],
                                initial_separation_m=float(max(0., distance)),
                                combined_displacement_bound_m=float(budget[pair_index]),
                                separation_lower_bound_m=float(margin[pair_index])))
        if len(margin):
            minimum_certified_margin = min(minimum_certified_margin, float(margin.min()))
        record = dict(direction=start['direction'], start_sample=start['sample'],
                      end_sample=end['sample'], pairs=len(pairs), exact_queries=len(candidates),
                      nominal_interval_self_separation_verified=not bad)
        intervals.append(record)
        if bad:
            unresolved.append({**record, 'unconfirmed_pairs': bad})
    result = dict(source_path_audit=str(args.path_audit.resolve()), intervals=len(intervals),
                  enabled_nonadjacent_pairs_per_interval=len(pairs),
                  nominal_self_separation_verified=not unresolved,
                  unconfirmed_intervals=len(unresolved), exact_distance_queries=exact_calls,
                  aabb_certified_pair_intervals=broad_certified,
                  minimum_distance_lower_bound_after_motion_m=float(minimum_certified_margin),
                  intervals_detail=intervals, unresolved=unresolved,
                  fixed_prismatic_positions=fixed, collision_disabled_adjacent_pairs_excluded=True,
                  execution_authorized=False, clearance_verified=False, runtime_integrated=False,
                  source_code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  limitations=['Self-intersection only, using existing CAD collision-enabled pairs; environment CAD+3cm test is separate.',
                               'Assumes fixed base and open fingers, and every joint remains within each prescribed interval box.',
                               'No servo overshoot, encoder uncertainty, live environment or contact-offset certificate.'])
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k: result[k] for k in ['intervals', 'enabled_nonadjacent_pairs_per_interval',
                                            'nominal_self_separation_verified', 'unconfirmed_intervals',
                                            'exact_distance_queries', 'minimum_distance_lower_bound_after_motion_m',
                                            'execution_authorized']}))


if __name__ == '__main__':
    main()
