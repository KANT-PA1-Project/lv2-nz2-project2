"""Assemble a v136 adapter bundle without editing shared core or launching children."""
import argparse
import hashlib
import json
from pathlib import Path

EXPERIMENT = Path(__file__).resolve().parent


def assemble(output, *, core_root=None, experiment_root=EXPERIMENT):
    experiment_root = Path(experiment_root).resolve(strict=True)
    core = Path(core_root or EXPERIMENT.parents[1]).resolve(strict=True)
    output = Path(output).expanduser().resolve()
    if output.exists():
        raise FileExistsError(output)
    if output.is_relative_to(core) and not output.is_relative_to(experiment_root / ".runtime"):
        raise ValueError("bundle_must_not_overwrite_source_tree")
    files = {}

    def register(target, source, origin):
        if target in files:
            raise ValueError("bundle_file_collision:" + target)
        files[target] = (source, origin)

    shared = core / "src/arm_practice"
    if not shared.is_dir():
        raise FileNotFoundError(shared)
    for source in sorted(shared.rglob("*.py")):
        if "__pycache__" not in source.parts:
            register(str(Path("src/arm_practice") / source.relative_to(shared)), source, "shared_core")
    for source in sorted((experiment_root / "overlay").rglob("*.py")):
        register(str(Path("src") / source.relative_to(experiment_root / "overlay")), source, "v136_overlay")
    for name in ("native_live_decoder.py", "native_ros_publisher.py", "priority_feedback_consumer.py"):
        register(name, experiment_root / name, "v136_child")
    for name in ("models/r12/r12_arm.urdf", "configs/sensors/camera_rig.json", "scripts/sensor_runtime/camera_rig.py"):
        register(name, core / name, "shared_asset")
    for name in ("mixed_camera_schema.json", "r12_tcp_calibration.json"):
        register("configs/" + name, experiment_root / "reference_configs" / name, "v136_reference")
    manifest = {}
    contents = {}
    for target, (source, origin) in sorted(files.items()):
        data = source.read_bytes()
        if data.startswith(b"version https://git-lfs.github.com/spec/v1"):
            raise ValueError("unresolved_LFS_pointer:" + str(source))
        contents[target] = data
        manifest[target] = {"sha256": hashlib.sha256(data).hexdigest(), "origin": origin}
    output.mkdir(parents=True)
    for target, data in contents.items():
        destination = output / target
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(data)
    content = json.dumps({"schema": "v136-adapter-bundle-v1", "files": manifest,
                          "physics_runtime_included": False, "motion_authorized": False}, sort_keys=True, indent=2)
    (output / "native_entrypoint_manifest.json").write_text(content, encoding="utf-8")
    return {"root": str(output), "source_sha256": hashlib.sha256(content.encode()).hexdigest(),
            "files": len(files), "motion_authorized": False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--core-root", type=Path)
    args = parser.parse_args()
    print(json.dumps(assemble(args.output, core_root=args.core_root), indent=2))


if __name__ == "__main__":
    main()
