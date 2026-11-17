"""One local isolated experiment, locked; no remote or physical executors."""
from pathlib import Path
import fcntl
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time

BASE = Path(__file__).resolve().parent
RUN = BASE / "runs" / "fresh-view-04"
LOCK = Path(os.environ.get("PA24_EXPERIMENT_LOCK",str(Path.home()/"isaac-fleet-runs/pa24-r12-experiment.lock")))
PYTHON = os.environ.get("PA24_ISAAC_PYTHON",sys.executable)
MANIFEST_OVERRIDES = {}
ADMISSION_REPORT = None


def main():
    LOCK.parent.mkdir(parents=True,exist_ok=True)
    with LOCK.open("a+") as handle:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        active = []
        for entry in Path("/proc").iterdir():
            if not entry.name.isdigit() or int(entry.name) == os.getpid():
                continue
            try:
                cmd = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace")
            except (OSError, PermissionError):
                continue
            # Only inspect process command names. Never read process environments.
            if any(x in cmd for x in ("render_axis.py", "isaacsim.simulation_app", "isaac-sim.sh")):
                active.append({"pid": int(entry.name), "command": cmd[:400]})
        if active:
            raise RuntimeError("existing Isaac renderer(s); no process terminated: " + json.dumps(active))
        gpu = subprocess.check_output(["nvidia-smi", "--query-gpu=index,memory.free,memory.total", "--format=csv,noheader,nounits"], text=True)
        gpu0 = next(line for line in gpu.splitlines() if line.split(",")[0].strip() == "0")
        if float(gpu0.split(",")[1]) < 1024:
            raise RuntimeError("insufficient GPU0 reserve")
        disk = shutil.disk_usage(BASE)
        if disk.free < 1024*1024*1024:
            raise RuntimeError("insufficient disk reserve")
        RUN.mkdir(parents=True, exist_ok=False)
        shutil.copytree(BASE / "source", RUN / "source", symlinks=True,
                        ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
        source = RUN / "source"
        for name in ('models','assets'):
            runtime_link=source/name
            original_link=BASE/'source'/name
            if runtime_link.is_symlink():
                target=original_link.resolve(strict=True)
                runtime_link.unlink();runtime_link.symlink_to(target,target_is_directory=True)
        if MANIFEST_OVERRIDES:
            manifest=json.loads((source/'run_manifest.json').read_text())
            manifest.update(MANIFEST_OVERRIDES)
            (source/'run_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
        if ADMISSION_REPORT is not None:
            (RUN/'static_pose_admission.json').write_text(json.dumps(ADMISSION_REPORT,indent=2)+'\n')
        hashes = {str(p.relative_to(source)): hashlib.sha256(p.read_bytes()).hexdigest()
                  for p in source.rglob("*") if p.is_file() and not p.is_symlink()}
        for name in ("models/r12/r12_arm.urdf", "models/r12/r12_mobile_gripper.usda",
                     "models/navigation/mission_scene.usda"):
            hashes[name] = hashlib.sha256((source / name).read_bytes()).hexdigest()
        provenance = dict(created_unix_s=time.time(), source_sha256=hashes,
                          baseline="all640-confirm-02", seed=2427, local_only=True,
                          gpu0_preflight=gpu0, disk_free_bytes=disk.free,
                          wall_budget_s=360, disk_budget_mib=500,
                          minimum_gpu_free_mib=1024, max_renders=4, max_ready_ns=200000000,
                          scope="new fresh live proposal only; no movement dispatched",
                          motion_authorized=False, calibration_accepted=False)
        (RUN / "provenance.json").write_text(json.dumps(provenance, indent=2)+"\n")
        env = dict(os.environ, FLEET_RUN_DIR=str(RUN), FLEET_RUN_ID="pa24-"+RUN.name)
        result = subprocess.run([PYTHON, str(source / "supervise.py")], env=env)
        (RUN / "launcher_result.json").write_text(json.dumps(dict(
            supervisor_exit_code=result.returncode,
            source_unchanged=all(hashlib.sha256((source / n).read_bytes()).hexdigest()==h
                                 for n,h in hashes.items()),
            motion_authorized=False), indent=2)+"\n")
        return result.returncode


if __name__ == "__main__":
    sys.exit(main())
