"""Configuration and URDF assembly; no Isaac/ROS imports or hardware access."""
import copy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
R12 = HERE.parents[1]


def vector(value, n, name):
    if (not isinstance(value, list) or len(value) != n
            or any(isinstance(x, bool) or not isinstance(x, (int, float))
                   or not math.isfinite(x) for x in value)):
        raise ValueError(f"{name}: finite {n}-element array required (currently {value!r})")
    return value


def load_config(path):
    cfg = json.loads(Path(path).read_text(encoding="utf-8"))
    mount = cfg["arm_mount"]
    vector(mount["xyz_m"], 3, "arm_mount.xyz_m")
    vector(mount["rpy_rad"], 3, "arm_mount.rpy_rad")
    vector(cfg["spawn_xyz_yaw_rad"], 4, "spawn_xyz_yaw_rad [x,y,z,yaw]")
    limits = json.loads((R12 / "configs/r12_robot.json").read_text())
    poses = [cfg["barcode_arm_q_rad"]]
    waypoints = cfg["tower_arm_waypoints_rad"]
    if not isinstance(waypoints, list) or not waypoints:
        raise ValueError("tower_arm_waypoints_rad: at least one 5-joint waypoint required")
    poses += waypoints
    for i, q in enumerate(poses):
        vector(q, 5, f"arm pose {i}")
        if any(not lo <= x <= hi for x, lo, hi in zip(q, limits["lower"], limits["upper"])):
            raise ValueError(f"arm pose {i}: outside configs/r12_robot.json joint limits")
    vector(cfg["barcode"]["center_m"], 3, "barcode.center_m")
    if cfg["barcode"]["code"] not in {f"P2-0{i}" for i in range(1, 5)}:
        raise ValueError("barcode.code must be P2-01..04")
    seq = cfg["sequence"]
    if (not isinstance(seq, str) or len(seq) != 12 or set(seq) - set("RYG")
            or any(not 1 <= seq.count(c) <= 6 for c in "RYG")):
        raise ValueError("sequence: 12 R/Y/G flashes, each color 1..6 times required")
    if cfg["scope"] != "simulation_only_fixed_chassis_arm_observation":
        raise ValueError("Only fixed-chassis arm observation is implemented")
    if cfg.get('trial_mode', 'barcode_then_tower') not in ('barcode_then_tower', 'tower_only'):
        raise ValueError('Unsupported trial_mode')
    if cfg["renderer"] != "PathTracing":
        raise ValueError("This experiment requires PathTracing; other renderer state updates unverified")
    for key in ("dome", "emissive", "max_seconds", "prepare_timeout_s",
                "barcode_timeout_s", "motion_timeout_s"):
        v = cfg[key]
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0:
            raise ValueError(f"{key}: positive finite number required")
    if cfg["barcode_upscale"] not in (1, 2):
        raise ValueError("barcode_upscale must be 1 or 2")
    if cfg.get("camera_fps", 30) not in (20, 30):
        raise ValueError("camera_fps must be 20 or 30 (120 Hz physics divisor)")
    if cfg.get('physics_hz', 120) not in (120, 240, 480, 960, 1920):
        raise ValueError('Unsupported physics_hz')
    root = Path(cfg["perception_root"]).expanduser()
    if not root.is_absolute():  # 상대 경로는 설정 파일 위치 기준 (예: "../barcode_signal")
        root = Path(path).resolve().parent / root
    cfg["perception_root"] = str(root.resolve())
    cfg["r20d_archive"] = str(Path(cfg["r20d_archive"]).expanduser().resolve())
    if cfg.get('floor_visual_usd'):
        cfg['floor_visual_usd'] = str(Path(cfg['floor_visual_usd']).expanduser().resolve())
        from appearance import asset_paths
        for file in asset_paths(cfg):
            if not file.is_file():
                raise FileNotFoundError(file)
    for file in (Path(cfg["perception_root"]) / "scripts/live_mission_optimized.py",
                 Path(cfg["r20d_archive"]), R12 / "models/r12/r12_mobile_gripper.urdf",
                 R12 / "models/navigation/mission_scene.usda"):
        if not file.is_file():
            raise FileNotFoundError(file)
    return cfg, limits


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def assemble_urdf(cfg, r20d_dir, destination):
    """Keep R20D chassis; copy only the R12 arm, moving fingers and TCP."""
    chassis = Path(r20d_dir) / "urdf/r20d.urdf"
    arm = R12 / "models/r12/r12_mobile_gripper.urdf"
    tree = ET.parse(chassis)
    robot = tree.getroot()
    robot.set("name", "r20d_perception")
    arm_root = ET.parse(arm).getroot()
    link_names = {f"arm_link{i}" for i in range(1, 6)} | {
        "gripper_left", "gripper_right", "grasp_tcp"}
    joint_names = {f"arm_joint{i}" for i in range(1, 6)} | {
        "gripper_left_joint", "gripper_right_joint", "grasp_tcp_fixed"}

    def resolve_meshes(element, source):
        for mesh in element.iter("mesh"):
            name = mesh.get("filename")
            if "://" in name:
                raise ValueError(f"Unexpected mesh URI: {name}")
            path = (source.parent / name).resolve()
            if not path.is_file():
                raise FileNotFoundError(path)
            mesh.set("filename", str(path))

    resolve_meshes(robot, chassis)
    selected_links, selected_joints = set(), set()
    for element in arm_root:
        name = element.get("name")
        selected = ((element.tag == "link" and name in link_names)
                    or (element.tag == "joint" and name in joint_names))
        if not selected:
            continue
        item = copy.deepcopy(element)
        resolve_meshes(item, arm)
        if element.tag == "link":
            selected_links.add(name)
        else:
            selected_joints.add(name)
        if name == "arm_joint1":
            item.find("parent").set("link", "base_link")
            origin = item.find("origin")
            origin.set("xyz", " ".join(map(str, cfg["arm_mount"]["xyz_m"])))
            origin.set("rpy", " ".join(map(str, cfg["arm_mount"]["rpy_rad"])))
        robot.append(item)
    if selected_links != link_names or selected_joints != joint_names:
        raise ValueError("R12 arm/gripper source structure changed")
    # Fixing the chassis is explicit. This is an arm/camera integration trial,
    # not a navigation benchmark or reuse of the R12 drive policy.
    for j in robot.findall("joint"):
        if j.get("name", "").endswith(("_steer_joint", "_wheel_joint")):
            j.set("type", "fixed")
            for tag in ("axis", "limit", "dynamics"):
                for item in j.findall(tag):
                    j.remove(item)
    ET.indent(tree)
    tree.write(destination, encoding="utf-8", xml_declaration=True)
    return {"r20d_urdf_sha256": sha256(chassis), "r12_arm_urdf_sha256": sha256(arm),
            "composite_urdf_sha256": sha256(destination),
            "arm_mount": cfg["arm_mount"], "base_fixed": True,
            "scope": cfg["scope"], "driving_policy_loaded": False}
