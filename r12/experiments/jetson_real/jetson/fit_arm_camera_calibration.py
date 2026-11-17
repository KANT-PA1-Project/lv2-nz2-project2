#!/usr/bin/env python3
"""Fit and cross-check a provisional camera correction from fixed-tag observations."""
import argparse
from collections import defaultdict
import itertools
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from single_puck_probe import camera_transform


def transform(parameters):
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_rotvec(parameters[3:]).as_matrix()
    matrix[:3, 3] = parameters[:3]
    return matrix


def phase_observations(report, allow_pnp, urdf):
    # A puck can carry the same tag ID on several faces. Match one fixed face.
    normals = {}
    camera_rotations = {}
    for sample in report["samples"]:
        tag = sample["tag"]
        if not tag or (not allow_pnp and not tag.get("depth_valid")):
            continue
        key = (tag["stamp_sec"], tag["stamp_nanosec"])
        if key in normals:
            continue
        angles = tuple(sample["status"]["position_deg"])
        if angles not in camera_rotations:
            camera_rotations[angles] = camera_transform(urdf, "gripper", angles)[:3, :3]
        normals[key] = camera_rotations[angles] @ Rotation.from_quat(
            tag["quaternion_xyzw"]).as_matrix()[:, 2]
    if not normals:
        return []
    vectors = np.asarray(list(normals.values()))
    similarity = vectors @ vectors.T
    dominant = vectors[np.argmax((similarity > 0.9).sum(axis=1))]
    matching = {key for key, normal in normals.items() if normal @ dominant > 0.9}
    grouped = defaultdict(list)
    for sample in report["samples"]:
        grouped[sample["phase"]].append(sample)
    rows = []
    for name, samples in grouped.items():
        fresh = [sample for sample in samples if sample["tag"] and
                 (sample["tag"]["stamp_sec"], sample["tag"]["stamp_nanosec"]) in matching and
                 sample["time"] - sample["tag"]["received"] < 0.4]
        if not allow_pnp:
            fresh = [sample for sample in fresh if sample["tag"].get("depth_valid")]
        if not fresh:
            continue
        end = fresh[-1]["time"]
        recent = [sample for sample in fresh if sample["time"] >= end - 1.0]
        if len(recent) < 5:
            continue
        tags = {tuple((sample["tag"]["stamp_sec"], sample["tag"]["stamp_nanosec"])):
                sample["tag"] for sample in recent}
        if len(tags) < 2:
            continue
        positions = np.array([tag["position_m"] for tag in tags.values()])
        rotations = Rotation.from_quat([tag["quaternion_xyzw"] for tag in tags.values()])
        pose = np.eye(4)
        pose[:3, :3] = rotations.mean().as_matrix()
        pose[:3, 3] = np.median(positions, axis=0)
        rows.append(dict(name=name, camera_tag=pose,
                         motor_deg=np.median([sample["status"]["position_deg"]
                                              for sample in recent], axis=0),
                         tag_position_spread_mm=(np.ptp(positions, axis=0) * 1000).tolist(),
                         frames=len(tags)))
    return rows


def fit(rows, urdf, signs):
    cameras = np.stack([camera_transform(urdf, "gripper",
                        row["motor_deg"] * -np.asarray(signs + (1,))) for row in rows])
    tags = np.stack([row["camera_tag"] for row in rows])
    nominal = cameras @ tags
    position = np.median(nominal[:, :3, 3], axis=0)
    rotation = Rotation.from_matrix(nominal[:, :3, :3]).mean().as_rotvec()
    initial = np.r_[np.zeros(6), position, rotation]
    training = np.array(["_-1" not in row["name"] for row in rows])
    if training.sum() < 4 or (~training).sum() < 2:
        raise ValueError("Need at least four training and two held-out poses")

    def residual(parameters):
        correction, fixed_tag = transform(parameters[:6]), transform(parameters[6:])
        predicted = cameras[training] @ correction @ tags[training]
        translation = (predicted[:, :3, 3] - fixed_tag[:3, 3]) / 0.003
        relative = fixed_tag[:3, :3].T @ predicted[:, :3, :3]
        angular = Rotation.from_matrix(relative).as_rotvec() / np.deg2rad(3.0)
        return np.c_[translation, angular].ravel()

    lower = np.r_[[-0.1] * 3, [-np.pi / 4] * 3, [-np.inf] * 6]
    upper = np.r_[[0.1] * 3, [np.pi / 4] * 3, [np.inf] * 6]
    solution = least_squares(residual, initial, bounds=(lower, upper),
                             loss="soft_l1", max_nfev=150)
    correction, fixed_tag = transform(solution.x[:6]), transform(solution.x[6:])
    predicted = cameras @ correction @ tags
    position_error = np.linalg.norm(predicted[:, :3, 3] - fixed_tag[:3, 3], axis=1)
    # The nominal result has its own training-set fixed-point estimate.
    nominal_center = np.median(nominal[training, :3, 3], axis=0)
    nominal_error = np.linalg.norm(nominal[:, :3, 3] - nominal_center, axis=1)
    singular_values = np.linalg.svd(solution.jac, compute_uv=False)
    condition = float(singular_values[0] / max(singular_values[-1], 1e-15))
    return dict(signs=list(signs), correction=correction.tolist(),
                correction_translation_m=solution.x[:3].tolist(),
                correction_rotation_deg=np.rad2deg(solution.x[3:6]).tolist(),
                training_rms_mm=float(np.sqrt(np.mean(position_error[training] ** 2)) * 1000),
                held_out_rms_mm=float(np.sqrt(np.mean(position_error[~training] ** 2)) * 1000),
                nominal_held_out_rms_mm=float(np.sqrt(np.mean(nominal_error[~training] ** 2)) * 1000),
                held_out_max_mm=float(position_error[~training].max() * 1000),
                parameter_condition_number=condition,
                correction_at_bound=bool(np.any(np.abs(solution.x[:3]) > 0.099)
                    or np.any(np.abs(solution.x[3:6]) > np.pi / 4 - 0.01)),
                optimizer_converged=bool(solution.success))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report")
    parser.add_argument("--urdf", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--allow-pnp", action="store_true")
    parser.add_argument("--search-signs", action="store_true")
    args = parser.parse_args()
    rows = phase_observations(json.loads(Path(args.report).read_text()), args.allow_pnp, args.urdf)
    if len(rows) < 6:
        raise ValueError(f"Insufficient independent poses: {len(rows)}")
    hypotheses = itertools.product((-1, 1), repeat=5) if args.search_signs else [(-1,) * 5]
    candidates = [fit(rows, args.urdf, tuple(signs)) for signs in hypotheses]
    candidates.sort(key=lambda candidate: candidate["held_out_rms_mm"])
    result = dict(provisional=True, runtime_applied=False, physical_tcp_verified=False,
                  observations="PnP" if args.allow_pnp else "aligned_RGBD",
                  poses=[dict(name=row["name"], motor_deg=row["motor_deg"].tolist(),
                              frames=row["frames"],
                              tag_position_spread_mm=row["tag_position_spread_mm"])
                         for row in rows], candidates=candidates)
    Path(args.output).write_text(json.dumps(result, indent=2))
    print(json.dumps(dict(pose_count=len(rows), best=candidates[0]), indent=2))


if __name__ == "__main__":
    main()
