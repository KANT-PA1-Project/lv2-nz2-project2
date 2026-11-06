"""Explicit host settings for v136 children; importing this starts no process."""
import hashlib
import json
import os
from pathlib import Path


class RuntimeSettings:
    def __init__(self, values):
        if not isinstance(values, dict):
            raise ValueError("runtime_config_must_be_object")
        self.values = dict(values)

    @classmethod
    def from_env(cls, environ=None):
        env = os.environ if environ is None else environ
        if env.get("R12_RUNTIME_VALUES"):
            return cls(json.loads(env["R12_RUNTIME_VALUES"]))
        values = {}
        if env.get("R12_RUNTIME_CONFIG"):
            config = Path(env["R12_RUNTIME_CONFIG"]).expanduser().resolve(strict=True)
            values = json.loads(config.read_text())
            if not isinstance(values, dict):
                raise ValueError("runtime_config_must_be_object")
            for key, value in list(values.items()):
                if key != "ros_domain_id" and isinstance(value, str) and value:
                    # Do not resolve executable symlinks: that loses a venv's identity.
                    path = Path(value).expanduser()
                    values[key] = os.path.abspath(config.parent / path)
        base = env.get("R12_PERCEPTION_BASE")
        if base:
            base = Path(base).expanduser().resolve()
            for key, suffix in {
                "model_root": "models/perception",
                "model_manifest": "perception_model_manifest.json",
                "decoder_deps": "work/perception_deps",
                "decoder_scripts": "work/perception_implementation/scripts",
            }.items():
                values.setdefault(key, str(base / suffix))
        for key in ("decoder_python", "ros_python", "ros_setup", "warmup_npz", "ros_domain_id"):
            name = "R12_" + key.upper()
            if name in env:
                values[key] = env[name]
        values.setdefault("ros_python", "/usr/bin/python3")
        values.setdefault("ros_setup", "/opt/ros/lyrical/setup.bash")
        return cls(values)

    def child_env(self, environ=None):
        env = dict(os.environ if environ is None else environ)
        env["R12_RUNTIME_VALUES"] = json.dumps(self.values, allow_nan=False)
        return env

    def path(self, name, *, directory=False, executable=False):
        value = self.values.get(name)
        if not isinstance(value, str) or not value:
            raise ValueError("missing_runtime_setting:" + name)
        path = Path(os.path.abspath(Path(value).expanduser()))
        if not executable:
            path = path.resolve(strict=True)
        if (directory and not path.is_dir()) or (not directory and not path.is_file()):
            raise ValueError("invalid_runtime_path:" + name)
        if executable and not os.access(path, os.X_OK):
            raise ValueError("runtime_executable_required:" + name)
        return path

    def decoder(self):
        paths = {key: self.path(key, directory=key in ("model_root", "decoder_deps", "decoder_scripts"), executable=key == "decoder_python")
                 for key in ("decoder_python", "model_root", "model_manifest", "decoder_deps", "decoder_scripts", "warmup_npz")}
        manifest = json.loads(paths["model_manifest"].read_text())
        required = {"inference.json", "yolo/weights/best.pt", "centerpose/best.pt"}
        if "model_sha256" in manifest:
            pins = manifest["model_sha256"]
        else:
            # r12/MODEL_MANIFEST.json is also an explicit pin source.
            pins = {}
            for entry in manifest.get("models", []):
                prefix = "models/perception/"
                if entry["path"].startswith(prefix):
                    name = entry["path"][len(prefix):]
                    if name in required:
                        if name in pins:
                            raise ValueError("duplicate_model_pin:" + name)
                        pins[name] = entry["sha256"]
        if not isinstance(pins, dict) or not required.issubset(pins):
            raise ValueError("missing_perception_model_pins")
        for name, digest in pins.items():
            relative = Path(name)
            if relative.is_absolute() or ".." in relative.parts:
                raise ValueError("model_path_escape")
            if not isinstance(digest, str) or len(digest) != 64 or any(c not in "0123456789abcdef" for c in digest):
                raise ValueError("invalid_model_pin:" + name)
            target = (paths["model_root"] / relative).resolve(strict=True)
            if not target.is_relative_to(paths["model_root"]) or not target.is_file():
                raise ValueError("model_path_escape")
        if not (paths["decoder_scripts"] / "train_centerpose.py").is_file():
            raise ValueError("missing_train_centerpose")
        return paths, dict(pins)

    @staticmethod
    def verify_models(paths, pins):
        for name, expected in pins.items():
            actual = hashlib.sha256((paths["model_root"] / name).read_bytes()).hexdigest()
            if actual != expected:
                raise ValueError("perception_model_hash_mismatch:" + name)

    def ros(self):
        value = self.values.get("ros_domain_id")
        if type(value) is bool or not (type(value) is int or isinstance(value, str) and value.isdecimal()):
            raise ValueError("explicit_ROS_domain_required")
        domain = int(value)
        if not 0 <= domain <= 232:
            raise ValueError("invalid_ROS_domain")
        return domain, self.path("ros_python", executable=True), self.path("ros_setup")
