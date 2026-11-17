"""Offline conservative joint-box audit; never a fresh control authorization.

At q_start each SAT separation is a distance lower bound. For each body,
r * sum(abs(dq)) bounds every vertex displacement under its ancestor
rotations (triangle inequality, arc >= chord). Subtract both bodies' bounds
from separation to cover ALL intermediate configurations in the joint box.
A failed bound is inconclusive, never evidence of collision or permission.
"""
from pathlib import Path
import itertools
import json
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd, UsdGeom, UsdPhysics

BASE = Path(__file__).resolve().parent
RUN = BASE / "runs/fresh-view-04"
SOURCE = RUN / "source"


def gap(A, a, B, b):
    axes = np.r_[A[:3,:3].T, B[:3,:3].T,
                 np.cross(A[:3,:3].T[:,None,:],B[:3,:3].T[None,:,:]).reshape(-1,3)]
    norms = np.linalg.norm(axes,axis=1)
    axes = axes[norms>1e-10] / norms[norms>1e-10,None]
    delta = abs(axes @ (B[:3,3]-A[:3,3]))
    ra = abs(axes @ A[:3,:3]) @ a
    rb = abs(axes @ B[:3,:3]) @ b
    return float(max(0., np.max(delta-ra-rb)))


def audit(source, q_start, q_end, tracking=.001, obstacle_spheres=()):
    SOURCE = Path(source)
    begin = time.monotonic()
    stage = Usd.Stage.Open(str(SOURCE / "models/r12/r12_mobile_gripper.usda"))
    cache = UsdGeom.XformCache()
    geom = json.loads((SOURCE / "cad_geometry.json").read_text())
    body_names = {"base_link", *[j["child"] for j in geom["joints"]]}
    fixed = {"base_link"}
    # These eight joints are explicitly welded in this stationary renderer.
    fixed_edges = []
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Joint) and ("wheel_joint" in prim.GetName() or "steer_joint" in prim.GetName()):
            j = UsdPhysics.Joint(prim)
            fixed_edges.append((j.GetBody0Rel().GetTargets()[0].name,
                                j.GetBody1Rel().GetTargets()[0].name))
    for _ in range(len(fixed_edges)):
        for parent,child in fixed_edges:
            if parent in fixed:fixed.add(child)
    if len(fixed_edges)!=8 or len(fixed)!=9:raise ValueError("fixed_base_assembly_mismatch")
    body_names.update(fixed)
    skip = set(); boxes = []; unmapped = []
    for p in stage.Traverse():
        if p.IsA(UsdPhysics.Joint):
            j = UsdPhysics.Joint(p)
            a = j.GetBody0Rel().GetTargets(); b = j.GetBody1Rel().GetTargets()
            if a and b and not j.GetCollisionEnabledAttr().Get():
                skip.add(frozenset([a[0].name,b[0].name]))
        if not p.HasAPI(UsdPhysics.CollisionAPI) or not UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get():
            continue
        body = p.GetParent()
        while body and body.GetName() not in body_names:
            body = body.GetParent()
        if not body:
            unmapped.append(str(p.GetPath())); continue
        if not (p.IsA(UsdGeom.Cube) or p.IsA(UsdGeom.Cylinder)):
            raise ValueError("unsupported_collider:"+str(p.GetPath()))
        M = np.asarray(cache.ComputeRelativeTransform(p,body)[0]).T.copy()
        scale = np.linalg.norm(M[:3,:3],axis=0)
        if np.any(scale<=0):raise ValueError("zero_collider_scale")
        M[:3,:3] /= scale
        if not np.allclose(M[:3,:3].T@M[:3,:3],np.eye(3),atol=1e-8):
            raise ValueError("nonorthogonal_collider")
        if p.IsA(UsdGeom.Cube):
            half = scale * float(UsdGeom.Cube(p).GetSizeAttr().Get())/2
        else:
            shape = UsdGeom.Cylinder(p)
            half = np.full(3,float(shape.GetRadiusAttr().Get()))
            half["XYZ".index(str(shape.GetAxisAttr().Get()))] = float(shape.GetHeightAttr().Get())/2
            half *= scale
        boxes.append((str(p.GetPath()),body.GetName(),M,half))
    q0 = np.asarray(q_start,float); q1 = np.asarray(q_end,float)
    if q0.shape!=(5,) or q1.shape!=(5,) or not np.isfinite([*q0,*q1,tracking]).all() or tracking<0:
        raise ValueError('invalid_joint_envelope')
    values = {f"arm_joint{i+1}":float(v) for i,v in enumerate(q0)}
    values.update(geom["gripper_open_positions"])
    base_inverse = np.linalg.inv(np.asarray(cache.GetLocalToWorldTransform(stage.GetPrimAtPath("/Robot/base_link"))).T)
    T = {b:base_inverse@np.asarray(cache.GetLocalToWorldTransform(stage.GetPrimAtPath("/Robot/"+b))).T for b in fixed}
    chains = {b:[] for b in fixed}; lengths = {b:0. for b in fixed}
    for joint in geom["joints"]:
        motion = np.eye(4); axis = np.eye(3)["XYZ".index(joint["axis"])]
        value = values[joint["name"]]
        if joint["prismatic"]:motion[:3,3] = axis*value
        else:motion[:3,:3] = Rotation.from_rotvec(axis*value).as_matrix()
        l0 = np.asarray(joint["l0"]); l1 = np.asarray(joint["l1"])
        child = joint["child"]; parent = joint["parent"]
        T[child] = T[parent]@l0@motion@np.linalg.inv(l1)
        chains[child] = chains[parent]+[joint]
        lengths[child] = lengths[parent]+np.linalg.norm(l0[:3,3])+np.linalg.norm(l1[:3,3])
        if joint["prismatic"]:lengths[child] += abs(value)+.001
    radii = {b:0. for b in body_names}
    for _,body,local,half in boxes:
        radii[body] = max(radii[body], lengths[body]+np.linalg.norm(local[:3,3])+np.linalg.norm(half))
    displacement = {}
    for body in body_names:
        rotation_total = sum(abs(q1[int(j["name"][-1])-1]-q0[int(j["name"][-1])-1])+tracking
                             for j in chains[body] if not j["prismatic"])
        linear_total = sum(.001 for j in chains[body] if j["prismatic"])
        displacement[body] = radii[body]*rotation_total+linear_total
    placed = [(name,body,T[body]@local,half) for name,body,local,half in boxes]
    # Repository uses external_cad_margin_m >= .03 for environment clearance.
    # Self pairs require positive conservative separation; no contact exception.
    margin = 0.; count = 0; failed = 0; worst = None; by_pair = {}
    for (name,body,A,a),(name2,body2,B,b) in itertools.combinations(placed,2):
        if body==body2 or frozenset([body,body2]) in skip:continue
        if body in fixed and body2 in fixed:continue
        count += 1
        initial_gap = gap(A,a,B,b)
        lower = initial_gap-displacement[body]-displacement[body2]
        key = " / ".join(sorted([body,body2]))
        by_pair[key] = min(by_pair.get(key,float("inf")),lower)
        if worst is None or lower<worst["lower_bound_m"]:
            worst = dict(colliders=[name,name2],bodies=[body,body2],
                         initial_separation_lower_bound_m=initial_gap,
                         lower_bound_m=lower)
        if lower<=margin:
            failed += 1
    environment_lower = None
    for center,radius in obstacle_spheres:
        center=np.asarray(center,float)
        if center.shape!=(3,) or not np.isfinite(center).all() or not np.isfinite(radius) or radius<0:raise ValueError('invalid_obstacle_sphere')
        for _,body,M,half in placed:
            local=M[:3,:3].T@(center-M[:3,3])
            lower=float(np.linalg.norm(np.maximum(abs(local)-half,0))-radius-displacement[body])
            environment_lower=lower if environment_lower is None else min(environment_lower,lower)
    result = dict(scope="conservative joint-box geometry diagnostic; not current control approval",
                  fresh_for_live_control=False,motion_authorized=False,calibration_accepted=False,
                  target_or_fixture_coordinates_used=bool(obstacle_spheres),coordinate_use_scope="collision preflight only; never a sensor target",
                  collider_count=len(boxes),unmapped_colliders=unmapped,
                  self_pairs_checked=count,self_required_separation_m=margin,
                  external_required_margin_m=.03,
                  fixed_assembly_bodies=sorted(fixed),
                  fixed_internal_pairs_excluded=True,
                  tracking_envelope_rad=tracking,gripper_tracking_envelope_m=.001,
                  q_start_rad=q0.tolist(),q_end_rad=q1.tolist(),
                  model_body_frames_base={name:value.tolist() for name,value in T.items()},
                  collider_paths=[name for name,_,_,_ in boxes],
                  rejected_pair_count=failed,worst_pair=worst,
                  body_pair_lower_bounds_m=by_pair,body_vertex_displacement_bounds_m=displacement,
                  all_self_bounds_proven=not unmapped and failed==0,
                  environment_bounds_proven=environment_lower is not None and environment_lower>=.03,
                  supplied_environment_spheres=len(obstacle_spheres),environment_lower_bound_m=environment_lower,
                  limitation="self only in welded stationary base assembly; no environment, frame-alignment or live native tracking certification",
                  duration_s=time.monotonic()-begin)
    return result


def main():
    proposal=json.loads((RUN/'live_view_02.json').read_text())
    captures=json.loads((RUN/'producer_capture_results.json').read_text())
    step=captures[2]['native_step']
    tick=next(x for x in json.loads((RUN/'physics_ticks.json').read_text()) if x['step']==step)
    result=audit(SOURCE,tick['q'][:5],proposal['candidate_q_rad'])
    result['input_capture_step']=step
    (BASE/'swept_path_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('body_pair_lower_bounds_m','body_vertex_displacement_bounds_m')},indent=2))


if __name__ == "__main__":
    main()
