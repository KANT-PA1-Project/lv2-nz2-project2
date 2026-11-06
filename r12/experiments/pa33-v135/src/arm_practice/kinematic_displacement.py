"""Conservative point displacement bounds for a fixed-base serial robot CAD.

For each ancestor revolute joint, triangle inequalities bound every collider
point's distance from the joint origin, independent of downstream rotations.
Summing radius * absolute joint excursion bounds displacement from a reference
configuration. This also covers non-simultaneous joint motion within the stated
joint box. It does not bound encoder error, base motion or servo overshoot.
"""
import numpy as np


def _rigid(matrix):
    matrix = np.asarray(matrix, float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError('Expected a finite4x4 rigid transform')
    rotation = matrix[:3, :3]
    if (not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-9, rtol=0) or
            not np.allclose(rotation.T@rotation, np.eye(3), atol=1e-7, rtol=0) or
            not np.isclose(np.linalg.det(rotation), 1., atol=1e-7, rtol=0)):
        raise ValueError('Scaled or nonrigid transform')
    return matrix


def collider_joint_radii(joints, body, local_transform, size, joint_names,
                         fixed_prismatic_positions, *, root_body='base_link'):
    """Return radius bounds in joint_names order; fixed prismatic offsets included.

    Joint tuples follow CollisionModel: name,parent,child,L0,L1,axis,prismatic.
    Every revolute ancestor must appear in joint_names. Unknown motion is rejected.
    """
    names = list(joint_names)
    if len(set(names)) != len(names) or not names:
        raise ValueError('Joint names must be unique and nonempty')
    local = _rigid(local_transform)
    size = np.asarray(size, float)
    if size.shape != (3,) or not np.isfinite(size).all() or np.any(size <= 0):
        raise ValueError('Invalid collider size')
    parents = {}
    for item in joints:
        name, parent, child, l0, l1, axis, prismatic = item
        if child in parents or axis not in ('X', 'Y', 'Z'):
            raise ValueError('Ambiguous child or unsupported joint axis')
        parents[child] = (name, parent, child, _rigid(l0), _rigid(l1), axis, bool(prismatic))
    chain, seen = [], set()
    while body != root_body:
        if body in seen or body not in parents:
            raise ValueError('Disconnected or cyclic joint chain')
        seen.add(body)
        joint = parents[body]
        chain.append(joint)
        body = joint[1]
    chain.reverse()
    radii = np.zeros(len(names))
    # Collider origin offset + enclosing sphere radius; invariant under rotation.
    downstream_radius = np.linalg.norm(local[:3, 3]) + np.linalg.norm(size)/2
    for name, parent, child, l0, l1, axis, prismatic in reversed(chain):
        downstream_radius += np.linalg.norm(l1[:3, 3])
        if prismatic:
            value = fixed_prismatic_positions.get(name)
            if name in names or value is None or not np.isfinite(value):
                raise ValueError('Prismatic ancestor needs a finite fixed position')
            downstream_radius += abs(float(value))
        else:
            if name not in names:
                raise ValueError('Unbounded revolute ancestor')
            radii[names.index(name)] = downstream_radius
        downstream_radius += np.linalg.norm(l0[:3, 3])
    return radii


def point_displacement_bound(radii, maximum_joint_excursion):
    """Radius sum bound from one endpoint; no half-step or linear-motion assumption."""
    radii, excursion = np.asarray(radii, float), np.asarray(maximum_joint_excursion, float)
    if (radii.ndim != 1 or radii.shape != excursion.shape or not len(radii) or
            not np.isfinite(radii).all() or not np.isfinite(excursion).all() or
            np.any(radii < 0) or np.any(excursion < 0)):
        raise ValueError('Radius and excursion vectors must be finite and nonnegative')
    return float(radii @ excursion)
