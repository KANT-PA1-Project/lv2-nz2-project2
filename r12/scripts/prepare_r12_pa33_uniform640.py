"""Prepare an isolated PA33 consumer candidate; never launch remote jobs."""
import argparse
import hashlib
import json
import shutil
from pathlib import Path

def prepare(root, output):
    root=Path(root).resolve();output=Path(output).resolve()
    baseline=root/"experiments/pa33-v135"
    if output.exists():raise FileExistsError("Refuse to overwrite an existing snapshot")
    if output==root or root in output.parents and "runs" not in output.parts:
        raise ValueError("Use a runs directory or an output outside the repository")
    shutil.copytree(baseline,output,ignore=shutil.ignore_patterns("__pycache__","*.pyc","runs"))
    for name in ("models","assets"):
        (output/name).symlink_to(root/name,target_is_directory=True)
    for name in ("src/arm_practice/native_rgbd/mixed_camera_schema.py",
                 "configs/sensors/camera_rig.json","configs/mixed_camera_schema.json",
                 "scripts/sensor_runtime/camera_rig.py","configs/r12_tcp_calibration.json"):
        shutil.copy2(root/name,output/name)
    # Keep PA33's optional per-send timing instrumentation.
    ipc=output/"src/arm_practice/native_rgbd/trusted_capture_ipc.py"
    text=ipc.read_text()
    old="if version==WIRE_V2 and isinstance(schema,MixedCameraSchema):return 48*1024*1024"
    if text.count(old)!=1:raise ValueError("PA33 baseline IPC changed; review before preparing")
    text=text.replace("WIRE_V1,WIRE_V2,MixedCameraSchema","WIRE_V1,WIRE_V2,WIRE_V3,MixedCameraSchema")
    text=text.replace(old,"if isinstance(schema,MixedCameraSchema) and version==schema.wire_schema:\n        if version==WIRE_V2:return 48*1024*1024\n        if version==WIRE_V3:return MAX_BYTES")
    ipc.write_text(text)
    # All copied bytes are pinned afresh; old experiment manifests are historical.
    files={str(p.relative_to(output)):hashlib.sha256(p.read_bytes()).hexdigest()
           for p in output.rglob("*") if p.is_file() and not p.is_symlink()
           and p.name not in ("native_entrypoint_manifest.json","uniform640_candidate.json")}
    schema=json.loads((output/"configs/mixed_camera_schema.json").read_text())
    for name,expected in schema["extrinsic_source_sha256"].items():
        actual=hashlib.sha256((output/name).read_bytes()).hexdigest()
        if actual!=expected:raise ValueError("Candidate geometry/schema hash mismatch: "+name)
        files[name]=actual
    manifest={"run_id":"pa33-uniform640-down30-local-candidate","sha256":files,
              "wire_schema":schema["wire_schema"],"calibration_sha256":schema["calibration_sha256"],
              "motion_authorized":False,"remote_started":False}
    (output/"native_entrypoint_manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
    (output/"uniform640_candidate.json").write_text(json.dumps({
        "baseline":"pa33-v135","wire_schema":schema["wire_schema"],
        "payload_bytes":12_288_000,"mount_pitch_down_deg":30,
        "scope":"consumer candidate only; inference/ROS and mission require separate runtime validation",
        "calibration_accepted":False,"motion_authorized":False,"remote_started":False},indent=2)+"\n")
    return output

if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--r12-root",required=True,type=Path)
    p.add_argument("--output",required=True,type=Path);a=p.parse_args()
    print(prepare(a.r12_root,a.output))
