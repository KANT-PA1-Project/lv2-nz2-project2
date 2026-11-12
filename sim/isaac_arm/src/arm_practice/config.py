import json
from pathlib import Path
from .core import joint_targets


def load_robot_config(path):
    path = Path(path).resolve()
    c = json.loads(path.read_text(encoding="utf-8"))
    names = c["joint_names"]
    if not names or len(names) != len(set(names)) or not all(isinstance(n, str) and n for n in names):
        raise ValueError("joint_names must be unique non-empty strings")
    n = len(names)
    joint_targets([0]*n, c["home"], c["action_scale"], c["lower"], c["upper"])
    import math
    for key in ("effort", "velocity", "stiffness", "damping"):
        if len(c[key]) != n or any(not math.isfinite(x) or x <= 0 for x in c[key]):
            raise ValueError(f"Invalid {key}")
    for axis in ("x", "y", "z"):
        a, b = c["target_range"][axis]
        if not math.isfinite(a) or not math.isfinite(b) or a > b:
            raise ValueError("Invalid target range")
    if not math.isfinite(c["success_distance_m"]) or c["success_distance_m"] <= 0 or not c["tcp_link"]:
        raise ValueError("Invalid success threshold / TCP")
    usd = Path(c["usd_path"])
    c["usd_path"] = str(usd if usd.is_absolute() else (path.parent.parent / usd).resolve())
    return c
