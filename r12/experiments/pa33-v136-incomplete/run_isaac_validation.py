"""Check a staged validation plan; --execute explicitly starts the native trial."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

from prepare_isaac_validation import digest


def verify_plan(path):
    path = Path(path).resolve(strict=True)
    plan = json.loads(path.read_text())
    if plan.get("schema") != "v136-isaac-validation-plan-v1":
        raise ValueError("invalid_validation_plan")
    area = path.parent
    if Path(plan["output"]) != area / "run" or Path(plan["bundle"]["root"]) != area / "bundle":
        raise ValueError("validation_path_mismatch")
    for name, expected in plan["inputs"].items():
        target = (area / name).resolve(strict=True)
        if target.parent != area or digest(target) != expected:
            raise ValueError("staged_input_changed:" + name)
    asset_root = Path(plan["asset_root"]).resolve(strict=True)
    for name, expected in plan["assets"].items():
        target = (asset_root / name).resolve(strict=True)
        if not target.is_relative_to(asset_root) or digest(target) != expected:
            raise ValueError("shared_asset_changed:" + name)
    bundle = area / "bundle"
    manifest_path = bundle / "native_entrypoint_manifest.json"
    if digest(manifest_path) != plan["bundle"]["source_sha256"]:
        raise ValueError("bundle_manifest_changed")
    manifest = json.loads(manifest_path.read_text())
    for name, entry in manifest["files"].items():
        target = (bundle / name).resolve(strict=True)
        if not target.is_relative_to(bundle) or digest(target) != entry["sha256"]:
            raise ValueError("bundle_file_changed:" + name)
    fixed = dict(physics_hz=960, initial_record_steps=960, interval_steps=48,
                 warmup_epochs=2, measurement_epochs=8, attempts=10,
                 wall_budget_s=300, free_gpu_reserve_mib=1024, minimum_free_disk_mib=2048)
    if any(type(plan.get(key)) is not int or plan[key] != value for key,value in fixed.items()):
        raise ValueError("validation_protocol_changed_review_required")
    if plan.get("motion_authorized") is not False or plan.get("seed") != 0:
        raise ValueError("stationary_plan_required")
    return plan


def gpu_state():
    result = subprocess.run(["nvidia-smi", "--query-gpu=index,memory.free,utilization.gpu",
                             "--format=csv,noheader,nounits"],
                            check=True, text=True, capture_output=True, timeout=10)
    rows = [[int(x.strip()) for x in line.split(",")] for line in result.stdout.splitlines()]
    row = next(row for row in rows if row[0] == 0)
    return dict(gpu=0, free_mib=row[1], utilization_percent=row[2])


def other_isaac_processes():
    found = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdecimal() or int(entry.name) == os.getpid():
            continue
        try:
            parts = (entry / "cmdline").read_bytes().split(b"\0")
            executable = parts[0].decode(errors="replace")
            # Inspect executables, not arbitrary command text from editors/agents.
            if ".venv-isaacsim/" in executable or Path(executable).name in ("kit", "isaac-sim"):
                found.append(int(entry.name))
        except (OSError, IndexError):
            continue
    return found


def fixed_environment(plan, area):
    env = {key:value for key,value in os.environ.items()
           if not key.startswith(("R12_", "FLEET_", "ROS_", "CUDA_", "PYTHON"))}
    env.update(PYTHONDONTWRITEBYTECODE="1", PYTHONUNBUFFERED="1",
        CUDA_VISIBLE_DEVICES="0", ROS_DOMAIN_ID="33",
        ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST", R12_GUI="0",
        R12_STATIONARY_CAPTURE="1", R12_MISSION="0", R12_SENSOR_MOTION="0",
        R12_MISSION_PERCEPTION="0", R12_PHYSICS_HZ="960",
        R12_PREPARED_CAMERA_RIG="1", R12_ANNOTATOR_DEVICE="cpu",
        R12_FORCE_AA_OFF="1", R12_V136_SUPERVISED=plan["run_id"],
        R12_RUNTIME_CONFIG=str(area / "runtime.json"))
    return env


def terminate_owned(process):
    if process.poll() is not None:
        # A failed constructor can leave a child after its parent has exited.
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        process.wait(timeout=5)
        return
    try:
        process.wait(timeout=15)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        process.wait(timeout=5)
        return
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait(timeout=5)


def execute(path, plan):
    area = Path(path).resolve().parent
    if Path(plan["output"]).exists():
        raise FileExistsError("run_output_already_exists_prepare_new_plan")
    # This lock supplements process checks; it never kills an unrelated job.
    lock = Path("/tmp") / ("r12-native-" + str(os.getuid()) + ".lock")
    with lock.open("a+") as handle:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        active = other_isaac_processes()
        if active:
            raise RuntimeError("existing_isaac_processes:" + str(active))
        if shutil.disk_usage(area).free < plan["minimum_free_disk_mib"] * 1024**2:
            raise RuntimeError("insufficient_free_disk")
        gpu = gpu_state()
        if gpu["free_mib"] < plan["free_gpu_reserve_mib"]:
            raise RuntimeError("insufficient_gpu_reserve")
        # The system ROS interpreter needs numpy/scipy/rclpy. Test imports only,
        # at explicit execution time, before starting Isaac.
        config = json.loads((area / "runtime.json").read_text())
        subprocess.run(["bash", "-c", 'source "$1" && exec "$2" -c "$3"', "v136",
            config["ros_setup"], config["ros_python"],
            "import numpy, scipy, rclpy, sensor_msgs, geometry_msgs, tf2_msgs, rosgraph_msgs"],
            env=fixed_environment(plan, area), check=True, timeout=30)
        command = [plan["isaac_python"], str(area / "isaac_validation_runtime.py"),
                   "--plan", str(Path(path).resolve())]
        before = digest(path)
        started = time.monotonic()
        reason = None
        process = None
        try:
            with (area / "native.log").open("x") as log, (area / "resource_samples.jsonl").open("x") as samples:
                process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                    env=fixed_environment(plan, area), cwd=area, start_new_session=True)
                (area / "supervisor_pid.json").write_text(json.dumps(dict(
                    supervisor=os.getpid(), native=process.pid, process_group=process.pid,
                    plan_sha256=before, started_monotonic_ns=time.monotonic_ns())))
                while process.poll() is None:
                    elapsed = time.monotonic() - started
                    gpu = gpu_state()
                    samples.write(json.dumps(dict(elapsed_s=elapsed, **gpu)) + "\n")
                    samples.flush()
                    if elapsed > plan["wall_budget_s"]:
                        reason = "wall_budget_exceeded"
                    elif gpu["free_mib"] < plan["free_gpu_reserve_mib"]:
                        reason = "gpu_reserve_breached"
                    elif shutil.disk_usage(area).free < plan["minimum_free_disk_mib"] * 1024**2:
                        reason = "disk_reserve_breached"
                    elif (area / "STOP").exists():
                        reason = "supervisor_stop_file"
                    if reason:
                        break
                    time.sleep(2)
        except BaseException as error:
            reason = type(error).__name__ + ":" + str(error)
        finally:
            if process is not None:
                terminate_owned(process)
            try:
                verify_plan(path)
                sources_unchanged = True
            except Exception as error:
                sources_unchanged = False
                reason = reason or "source_changed:" + str(error)
            result = dict(sources_unchanged=sources_unchanged,
                returncode=process.returncode if process else None,
                failure=reason, elapsed_s=time.monotonic()-started,
                plan_unchanged=digest(path) == before, mission_success=False)
            (area / "supervisor_exit.json").write_text(json.dumps(result, indent=2) + "\n")
        from verify_isaac_validation import report
        evidence = report(path)
        if reason or result["returncode"] != 0 or not result["plan_unchanged"]:
            raise RuntimeError("native_validation_failed:" + json.dumps(result))
        if not evidence["observation_bench_pass"]:
            raise RuntimeError("native_evidence_failed:" + str(evidence["errors"]))
        print(json.dumps(result, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--execute", action="store_true",
                        help="start Isaac/decoder/ROS; omit for CPU file checks only")
    args = parser.parse_args()
    plan = verify_plan(args.plan)
    if args.execute:
        execute(args.plan, plan)
    else:
        from preflight_runtime import preflight
        area = args.plan.resolve().parent
        report = preflight(area / "bundle", area / "runtime.json",
                           Path(plan["asset_root"]) / "MODEL_MANIFEST.json")
        report.update(plan_sha256=digest(args.plan), assets_pinned=len(plan["assets"]),
                      native_started=False, launch_prepared=True, native_verified=False)
        (area / "readiness.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
