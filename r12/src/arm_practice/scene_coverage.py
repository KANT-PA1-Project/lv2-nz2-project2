"""Compare resolved collider inventories; coverage alone never proves clearance.

The caller must enumerate native active colliders, including invisible geometry.
This pure audit does not attest that enumeration or geometric bounds are correct.
"""

def audit_scene_coverage(native_paths, represented_paths, *, unresolved_references=(), reference_audit_complete=False):
    def paths(values):
        if isinstance(values, (str, bytes)):
            raise ValueError('Expected collider path collection')
        values = list(values)
        if any(not isinstance(p, str) or not p.startswith('/') or p == '/' for p in values):
            raise ValueError('Expected absolute non-root prim paths')
        if len(set(values)) != len(values):
            raise ValueError('Duplicate collider paths')
        return set(values)
    actual, represented = paths(native_paths), paths(represented_paths)
    if type(reference_audit_complete) is not bool:
        raise ValueError('Reference audit completeness must be explicit boolean')
    if isinstance(unresolved_references, (str, bytes)):
        raise ValueError('Expected unresolved reference collection')
    unresolved = list(unresolved_references)
    missing = sorted(actual - represented)
    return {'native_active_collider_count': len(actual),
            'represented_collider_count': len(represented),
            'unrepresented_paths': missing,
            'extra_representation_paths': sorted(represented - actual),
            'unresolved_references': unresolved,
            'reference_audit_complete': reference_audit_complete,
            'coverage_pass': reference_audit_complete and bool(actual) and not missing and not unresolved,
            'motion_authorized': False,
            'scope': 'inventory comparison only; no geometry/clearance certification'}


def inspect_usd_inventory(scene_path):
    """Read composed USD and all dependencies without running simulation.

    Missing textures also prevent an overall scene-reference pass. Composition
    errors and unresolved assets stay separate for diagnosis. Instance proxies
    are traversed so instanced collider geometry is not silently omitted.
    """
    from pxr import Usd, UsdPhysics, UsdUtils
    stage = Usd.Stage.Open(str(scene_path))
    if stage is None:
        raise ValueError('Cannot open USD scene')
    layers, assets, unresolved = UsdUtils.ComputeAllDependencies(str(scene_path))
    errors = [str(error) for error in stage.GetCompositionErrors()]
    native = sorted(str(prim.GetPath()) for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies())
                    if prim.HasAPI(UsdPhysics.CollisionAPI)
                    and UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is not False)
    return {'native_paths': native, 'composition_errors': errors,
            'unresolved_references': sorted(map(str, unresolved)),
            'layer_paths': sorted(str(layer.realPath) for layer in layers),
            'reference_audit_complete': True,
            'references_pass': not errors and not unresolved,
            'motion_authorized': False}
