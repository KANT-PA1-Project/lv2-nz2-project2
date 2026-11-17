"""Offline evidence report for a staged v136 run. Never starts native processes."""
import argparse
import json
from pathlib import Path
import sys


def report(plan_path):
    from run_isaac_validation import verify_plan
    plan = verify_plan(plan_path)
    area = Path(plan_path).resolve().parent
    run = Path(plan["output"])
    errors = []
    def read(name):
        try:
            return json.loads((run / name).read_text())
        except (OSError, ValueError) as error:
            errors.append(name + ":" + str(error))
            return None
    shutdown = read("shutdown.json")
    rejections = read("runtime_rejections.json")
    required = {"invalid_frame", "unowned_capture", "source_mismatch", "cross_thread"}
    if (not rejections or rejections.get("epoch_unchanged") is not True
            or {r.get("case") for r in rejections.get("cases", []) if r.get("rejected") is True} != required):
        errors.append("runtime_owner_rejections_incomplete")
    observations = read("observations.json") or []
    sys.path.insert(0, str(area / "bundle/src"))
    from arm_practice.native_rgbd.ack_metrics import analyze, stats
    try:
        ack = analyze(run / "observer/producer_ack.jsonl")
    except (OSError, ValueError) as error:
        ack = dict(valid=False, errors=[str(error)], samples=[], summary=None)
    phases = {"warmup": [], "measurement": []}
    for row in observations:
        phase = row.get("phase")
        if phase not in phases:
            errors.append("unknown_observation_phase")
            continue
        result = row.get("result", {})
        ros = result.get("ROS", {})
        decoded = result.get("decoded", {})
        if result.get("captured") is not True:
            errors.append("uncaptured_frame:" + str(row.get("frame")))
            continue
        if ros.get("all_received") is not True or ros.get("errors") or ros.get("raw_bytes_hash_verified") != 12:
            errors.append("ROS_or_raw_hash_failure:" + str(row["frame"]))
        if decoded.get("live_source_authenticated") is not True or decoded.get("raw_hashes_verified") != 12:
            errors.append("decoder_authentication_failure:" + str(row["frame"]))
        if decoded.get("camera_fresh_at_completion") is not True:
            errors.append("decoder_capture_stale:" + str(row["frame"]))
        if phase == "measurement" and (ros.get("expected") != 31 or ros.get("received") != 31):
            errors.append("measurement_required_31_samples_mismatch:" + str(row["frame"]))
        phases[phase].append(ros.get("capture_stamp_ns"))
    for phase, count in (("warmup", 2), ("measurement", 8)):
        if len(phases[phase]) != count:
            errors.append(phase + "_count_mismatch")
    if not ack["valid"]:
        errors.extend("ACK:" + str(e) for e in ack["errors"])
    measurements = [sample["producer_ACK_ms"] for sample in ack["samples"]
                    if sample["capture_stamp_ns"] in phases["measurement"]]
    if len(measurements) != 8:
        errors.append("measurement_ACK_count_mismatch")
    coverage_rows = [row.get("coverage_ROS") for row in observations]
    coverage_required = plan.get("coverage_required", False) or any(coverage_rows)
    coverage_pass = len(coverage_rows)==10 and all(row and row.get("all_received")is True
        and row.get("expected")==row.get("received")==12 and not row.get("errors")
        for row in coverage_rows)
    coverage_faults_pass = False
    if coverage_required:
        if not coverage_pass:
            errors.append("expanded_sensor_ROS_incomplete")
        faults = read("coverage_native_rejections.json") or []
        coverage_faults_pass = {row.get("case") for row in faults if row.get("published_samples")==0
            and row.get("kind")=="sensor_coverage_rejected"} == {
            "source_mismatch","hash_mismatch","nonce_replay","stale_lidar"}
        if not coverage_faults_pass:
            errors.append("coverage_native_IPC_rejections_incomplete")
    sensor_checks_pass = not errors
    child_normal_exit = bool(shutdown) and shutdown.get("child_exit_codes") == {
        "decoder": 0, "ros": 0, "priority": 0}
    if not shutdown or shutdown.get("observation_bench_pass") is not True:
        errors.append("native_shutdown_or_bench_failed")
    try:
        supervisor = json.loads((area / "supervisor_exit.json").read_text())
    except (OSError, ValueError) as error:
        supervisor = {}
        errors.append("supervisor_exit_missing:" + str(error))
    if supervisor.get("returncode") != 0 or supervisor.get("failure") is not None:
        errors.append("native_parent_process_failed")
    if supervisor.get("sources_unchanged") is not True or supervisor.get("plan_unchanged") is not True:
        errors.append("native_source_or_plan_changed")
    result = dict(scope="offline_native_observation_evidence_not_full_pa33_acceptance",
        run_id=plan["run_id"], observation_bench_pass=not errors,
        native_sensor_observations_pass=sensor_checks_pass,
        child_normal_exit=child_normal_exit,
        parent_process_exit_code=supervisor.get("returncode"),
        errors=errors, attempted=len(observations),
        fixed_phases={"warmup": 2, "measurement": 8},
        ACK_all_attempts=ack, ACK_measurement_only=stats(measurements),
        full_sensor_ROS_verified=sensor_checks_pass and coverage_pass,
        coverage_IPC_fault_injection_verified=coverage_faults_pass,
        native_fault_injection_verified=False,
        full_pa33_acceptance=False, mission_success=False, performance_accepted=False)
    (area / "verification.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    args = parser.parse_args()
    result = report(args.plan)
    print(json.dumps({k:v for k,v in result.items() if k != "ACK_all_attempts"}, indent=2))
    raise SystemExit(0 if result["observation_bench_pass"] else 2)


if __name__ == "__main__":
    main()
