"""Recheck the operator problem package using CPU-only rule and geometry checks."""
import argparse
from collections import Counter
import csv
import hashlib
from itertools import combinations
import json
from pathlib import Path
import runpy
import sys


def require(condition, message):
    if not condition:
        raise ValueError(message)


def load_json(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


def validate(root, team_repo):
    signals = load_json(root / "signals.json")
    arena = load_json(root / "arena.json")
    answers = load_json(root / "answers.json")
    review = load_json(root / "review.json")
    provenance = load_json(root / "sources.json")
    with (root / "placement.csv").open(encoding="utf-8-sig", newline="") as stream:
        rows = list(csv.DictReader(stream))
    pucks = []
    for row in rows:
        for field in ("id", "x_mm", "y_mm", "bottom_z_mm", "top_z_mm", "tag_id"):
            row[field] = int(row[field])
        pucks.append({key: row[key] for key in ("id", "color", "shape")})
    by_id = {p["id"]: p for p in pucks}
    require(len(rows) == 12 and len(by_id) == 12, "Expected 12 distinct puck IDs")
    require(set(by_id) == set(range(12)), "This Q1 package uses IDs 0 through 11")
    require(all(p["color"] in ("R", "G") and p["shape"] in ("cylinder", "cuboid") for p in pucks), "Invalid puck attribute")
    sequence = signals["sequence"]
    counts = dict(Counter(sequence))
    require(len(sequence) == 12 and set(counts) == set("RYG"), "Expected 12 R/Y/G flashes")
    require(all(1 <= value <= 6 for value in counts.values()), "Each count must be 1 through 6")
    require(counts == signals["counts"] == answers["counts"], "Declared counts disagree")
    require(signals["barcode"] == answers["barcode"], "Barcode disagreement")
    orders = {"P2-01": "ASC", "P2-03": "ASC", "P2-02": "DESC", "P2-04": "DESC"}
    require(orders.get(signals["barcode"]) == signals["sort_order"], "Invalid barcode direction")
    require(signals["tie_break_order"] == list("RYG"), "Invalid tie rule")
    require((signals["on_s"], signals["off_s"], signals["end_off_s"]) == (1, 1, 3), "Invalid v5 timing")
    expected_duration = 11 * (signals["on_s"] + signals["off_s"]) + signals["on_s"] + signals["end_off_s"]
    require(signals["expected_end_from_first_onset_s"] == expected_duration, "Final OFF duration is counted from last lamp turn-off")
    for source in provenance["source_snapshots"]:
        path = root / source["snapshot"]
        require(hashlib.sha256(path.read_bytes()).hexdigest() == source["sha256"], "Source snapshot changed: " + str(path))
    original = load_json(root / "sources/mission.json")
    shape_names = {"원기둥": "cylinder", "직육면체": "cuboid"}
    expected_inventory = [{"id": p["id"], "color": p["color"], "shape": shape_names[p["shape"]]} for p in original["pucks"]]
    require(pucks == expected_inventory, "Q1 inventory differs from source register")
    with (root / "sources/placement.csv").open(encoding="utf-8-sig", newline="") as stream:
        source_pucks = {int(r["tag"]): r for r in csv.DictReader(stream) if r["object"] == "퍽"}
    boxes = {box["id"]: box for box in arena["source_boxes"]}
    require(len(boxes) == 6 and len(set(boxes)) == 6, "Expected six source boxes")
    for row in rows:
        box = boxes[row["source_box"]]
        slot_offset = {"L": -40, "R": 40}[row["slot"]]
        require([row["x_mm"], row["y_mm"]] == [box["center_xy_mm"][0] + slot_offset, box["center_xy_mm"][1]], "Invalid initial position")
        source_row = source_pucks[row["id"]]
        require(all(row[k] == int(source_row[k]) for k in ("x_mm", "y_mm", "bottom_z_mm", "top_z_mm")), "Placement differs from provided CSV")
        require(row["bottom_z_mm"] == 70 and row["top_z_mm"] == 130 and row["posture"] == "up", "Invalid puck height or posture")
    for box in boxes.values():
        occupants = [r for r in rows if r["source_box"] == box["id"]]
        require(len(occupants) == 2 and {r["slot"] for r in occupants} == {"L", "R"}, "Expected two distinct slots per box")
        require(sorted(r["id"] for r in occupants) == sorted(box["puck_ids"]), "Box ID list disagrees")
    feasible = []
    for chosen in combinations(pucks, counts["Y"]):
        if len({p["shape"] for p in chosen}) != 1:
            continue
        chosen_ids = {p["id"] for p in chosen}
        remaining = [p for p in pucks if p["id"] not in chosen_ids]
        if Counter(p["color"] for p in remaining) == Counter(R=counts["R"], G=counts["G"]):
            feasible.append({"Y": sorted(chosen_ids), **{c: sorted(p["id"] for p in remaining if p["color"] == c) for c in "RG"}})
    require(bool(feasible), "No feasible complete assignment")
    require(arena["patterns"] == original["patterns"], "Dice patterns differ from provided reference")
    require(arena["grid_pitch_mm"] == 60 and arena["position_tolerance_mm"] == 10 and arena["stability_s"] == 2, "Invalid v5 placement criteria")
    descending = signals["sort_order"] == "DESC"
    groups = sorted("RYG", key=lambda c: -counts[c] if descending else counts[c])
    require([loc["group"] for loc in answers["locations"]] == groups, "Wrong destination group order")
    assignment = {loc["group"]: sorted(loc["ids"]) for loc in answers["locations"]}
    require(assignment in feasible, "Answer allocation violates complete assignment")
    provided_solver = runpy.run_path(str(root / "sources/validate_problem.py"))["solve"]
    provided_assignment = provided_solver(original["pucks"], counts)
    require(provided_assignment in feasible, "Provided validator found an inconsistent allocation")
    destinations = {d["id"]: d for d in arena["destinations"]}
    for index, loc in enumerate(answers["locations"], 1):
        require(loc["location"] == "L" + str(index), "Wrong destination location")
        require(loc["count"] == counts[loc["group"]], "Wrong answer count")
        require(sorted(t["id"] for t in loc["targets"]) == sorted(loc["ids"]), "Target IDs disagree")
        center = destinations[loc["location"]]["grid_center_xy_mm"]
        offsets = {tuple(t["center_mm"][axis] - center[axis] for axis in (0, 1)) for t in loc["targets"]}
        required_offsets = {tuple(60 * v for v in point) for point in arena["patterns"][str(loc["count"]) ]}
        require(offsets == required_offsets, "Wrong dice pattern or orientation")
        require(all(t["center_mm"][2] == 100 and t["bottom_z_mm"] == 70 and t["top_z_mm"] == 130 for t in loc["targets"]), "Wrong target height")
    official_solver = runpy.run_path(str(root / "sources/validate_problem.py"))["solve"]
    official = official_solver(original["pucks"], counts)
    require(official in feasible, "Provided solver disagrees")
    sys.path.insert(0, str(team_repo / "r12/src"))
    from arm_practice.mission import plan
    team = plan(sequence, signals["barcode"], pucks)
    team_assignment = {loc["group"]: sorted(t["id"] for t in loc["targets"]) for loc in team["locations"]}
    require(team_assignment in feasible, "Team planner found an inconsistent allocation")
    coordinates_agree = True
    for actual, expected in zip(team["locations"], answers["locations"]):
        actual_targets = {t["id"]: [int(round(v * 1000)) for v in t["center_m"]] for t in actual["targets"]}
        expected_targets = {t["id"]: t["center_mm"] for t in expected["targets"]}
        coordinates_agree = coordinates_agree and actual_targets == expected_targets
        require(all(t["center_mm"][2] == 100 and t["bottom_z_mm"] == 70 and t["top_z_mm"] == 130 for t in expected["targets"]), "Invalid answer target height")
    team_sources_match = all(hashlib.sha256((team_repo / p["repo_relative_path"]).read_bytes()).hexdigest() == p["sha256"] for p in provenance["team_files"])
    practice = load_json(team_repo / "r12/configs/mission/practice.json")
    practice_mismatches = [p["id"] for p in practice["pucks"] if p != by_id[p["id"]]]
    input_files = ["signals.json", "placement.csv", "arena.json", "answers.json", "review.json", "sources.json"]
    pending = [key for key in ("actual_inventory_confirmation", "physical_setup_confirmation", "other_team_review", "return_zone_confirmation", "destination_tag_mount_confirmation") if review[key]["status"] != "confirmed"]
    return {"problem_id": signals["problem_id"], "evidence_kind": "offline_rule_and_geometry_validation", "logical_pass": True,
            "submission_ready": not pending and review["submission_tag"]["status"] == "created", "pending": pending,
            "counts": counts, "destination_groups": groups, "distinct_pucks": len(by_id),
            "valid_group_allocation_count": len(feasible), "all_feasible_group_allocations": feasible,
            "source_inventory_matches": True, "source_placement_matches": True,
            "provided_solver_agrees": True, "team_planner_allocation_agrees": team_assignment == assignment,
            "team_planner_coordinates_agree": coordinates_agree, "team_source_hashes_match": team_sources_match,
            "synthetic_practice_attribute_mismatch_ids": practice_mismatches,
            "physical_execution_performed": False,
            "input_sha256": {name: hashlib.sha256((root / name).read_bytes()).hexdigest() for name in input_files}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument("--team-repo", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    try:
        report = validate(args.root, args.team_repo)
    except (ValueError, KeyError, OSError, ImportError) as error:
        print(json.dumps({"logical_pass": False, "error": str(error)}, ensure_ascii=False))
        return 1
    rendered = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
