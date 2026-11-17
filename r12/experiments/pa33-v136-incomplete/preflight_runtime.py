"""Check an assembled v136 bundle and local assets without launching ROS/Isaac/GPU work."""
import argparse
import ast
import hashlib
import importlib
import json
from pathlib import Path
import sys


def preflight(bundle, config, reference_model_manifest=None):
    bundle = Path(bundle).resolve(strict=True)
    manifest_path = bundle / "native_entrypoint_manifest.json"
    manifest = json.loads(manifest_path.read_text())
    if manifest.get("schema") != "v136-adapter-bundle-v1":
        raise ValueError("invalid_bundle_manifest")
    for name, entry in manifest["files"].items():
        target = (bundle / name).resolve(strict=True)
        if not target.is_relative_to(bundle):
            raise ValueError("bundle_path_escape")
        if hashlib.sha256(target.read_bytes()).hexdigest() != entry["sha256"]:
            raise ValueError("bundle_file_hash_mismatch:" + name)
    sys.path.insert(0, str(bundle / "src"))
    from arm_practice.native_rgbd.runtime_settings import RuntimeSettings
    from arm_practice.native_rgbd.mixed_camera_schema import MixedCameraSchema
    from arm_practice.native_rgbd.capture_fk import CaptureFK
    import numpy as np
    # Ignore inherited settings so this report describes exactly the requested config.
    settings = RuntimeSettings.from_env({"R12_RUNTIME_CONFIG": str(Path(config).resolve(strict=True))})
    paths, pins = settings.decoder()
    settings.verify_models(paths, pins)
    reference_match = None
    if reference_model_manifest is not None:
        reference = RuntimeSettings(dict(settings.values, model_manifest=str(Path(reference_model_manifest).resolve(strict=True))))
        _, reference_pins = reference.decoder()
        if reference_pins != pins:
            raise ValueError("reference_model_pins_mismatch")
        reference_match = True
    domain, ros_python, ros_setup = settings.ros()
    ast.parse((paths["decoder_scripts"] / "train_centerpose.py").read_text())
    schema = MixedCameraSchema.from_file(bundle / "configs/mixed_camera_schema.json")
    CaptureFK(bundle)
    warmup = {}
    with np.load(paths["warmup_npz"], allow_pickle=False) as archive:
        for name, kind in (("arm_camera_left_optical", "rgb"), ("arm_camera_depth", "depth")):
            descriptor = schema.descriptor(name, kind)
            array = archive[name]
            if list(array.shape) != descriptor["shape"] or array.dtype != np.dtype(descriptor["dtype"]):
                raise ValueError("warmup_geometry_mismatch:" + name)
            warmup[name] = {"shape": list(array.shape), "dtype": str(array.dtype)}
    modules = []
    for name, entry in manifest["files"].items():
        if entry["origin"] == "v136_overlay":
            module = name.removeprefix("src/").removesuffix(".py").replace("/", ".")
            loaded = importlib.import_module(module)
            if not Path(loaded.__file__).resolve().is_relative_to(bundle):
                raise ValueError("module_loaded_outside_bundle:" + module)
            modules.append(module)
    return {"bundle": str(bundle), "bundle_manifest_sha256": hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
            "bundle_files_verified": len(manifest["files"]), "overlay_modules_imported": len(modules),
            "camera_schema_sha256": schema.sha256, "model_pins_verified": pins, "repository_model_pins_match": reference_match,
            "decoder_python": str(paths["decoder_python"]), "train_centerpose": str(paths["decoder_scripts"] / "train_centerpose.py"),
            "model_manifest": str(paths["model_manifest"]), "warmup_npz": str(paths["warmup_npz"]),
            "warmup_sha256": hashlib.sha256(paths["warmup_npz"].read_bytes()).hexdigest(), "warmup_arrays": warmup,
            "ros_domain_id": domain, "ros_python": str(ros_python), "ros_setup": str(ros_setup),
            "scope": "CPU_files_schema_imports_only", "children_started": 0, "motion_authorized": False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bundle", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--reference-model-manifest", type=Path)
    args = parser.parse_args()
    result = json.dumps(preflight(args.bundle, args.config, args.reference_model_manifest), indent=2)
    if args.output:
        args.output.write_text(result + "\n", encoding="utf-8")
    print(result)


if __name__ == "__main__":
    sys.dont_write_bytecode = True
    main()
