"""Prepare/reproduce a single locked local headless observation with the common rig."""
import argparse
import importlib.util
import json
from pathlib import Path
import shutil
import sys

def prepare(root, output):
    root=Path(root).resolve();output=Path(output).resolve()
    baseline=root/"experiments/pa24-live-rgbd-20261106"
    output.mkdir(parents=True,exist_ok=False)
    source=output/"source"
    shutil.copytree(baseline/"source",source,symlinks=True,
                    ignore=shutil.ignore_patterns("__pycache__","*.pyc"))
    for name in ("models","assets"):
        (source/name).unlink()
        (source/name).symlink_to(root/name,target_is_directory=True)
    for name in ("src/arm_practice/native_rgbd/mixed_camera_schema.py",
                 "src/arm_practice/native_rgbd/trusted_capture_ipc.py",
                 "configs/sensors/camera_rig.json","configs/mixed_camera_schema.json",
                 "scripts/sensor_runtime/camera_rig.py","configs/r12_tcp_calibration.json"):
        shutil.copy2(root/name,source/name)
    manifest=json.loads((source/"run_manifest.json").read_text())
    manifest.update(q=[0,-.25,.1,-.05,0],origin_link5_m=[.04263,0,.05244],
        scope="CAD down30 uniform640 fresh observation",calibration_view=True,
        identity_reobserve_challenge=False,live_motion_diagnostic=False,
        bounded_gpu_load=False,frames_per_phase=3,motion_authorized=False)
    (source/"run_manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
    return baseline,output

if __name__=="__main__":
    p=argparse.ArgumentParser()
    p.add_argument("--r12-root",type=Path,default=Path(__file__).resolve().parents[1])
    p.add_argument("--output",type=Path,required=True)
    p.add_argument("--prepare-only",action="store_true")
    a=p.parse_args();baseline,output=prepare(a.r12_root,a.output)
    if a.prepare_only:
        print(output)
    else:
        spec=importlib.util.spec_from_file_location("down30_launcher",baseline/"launch_live_view.py")
        launch=importlib.util.module_from_spec(spec);spec.loader.exec_module(launch)
        launch.BASE=output;launch.RUN=output/"runs/down30-uniform640-01"
        launch.PYTHON=sys.executable
        raise SystemExit(launch.main())
