"""Prepare an immutable, local Isaac validation plan. Never starts Isaac or children."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import textwrap
import uuid

from assemble_bundle import assemble, EXPERIMENT

PARENT_SHA = "5bf73b7a50355c4ef709c20999148de3295d3b5c107155666c806307b28baccf"
ASSET_FOLDERS = ("models/r12", "models/navigation", "models/swerve",
                 "assets/vendor/R12_NO_USB_URDF_USD", "assets/vendor/maze",
                 "scripts/sensor_runtime")


def digest(path):
    value = hashlib.sha256()
    with Path(path).open("rb") as stream:
        if stream.read(128).startswith(b"version https://git-lfs.github.com/spec/v1"):
            raise ValueError("unresolved_LFS_pointer:" + str(path))
        stream.seek(0)
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def replace_once(source, before, after):
    if source.count(before) != 1:
        raise ValueError("parent_patch_anchor_changed:" + before[:80])
    return source.replace(before, after, 1)


def parent_parts(path):
    """Pin the shared source and retain its exact actuator block, with no execution."""
    if digest(path) != PARENT_SHA:
        raise ValueError("shared_parent_changed_review_required")
    original = Path(path).read_text()
    lines = original.splitlines(keepends=True)
    tree = ast.parse(original)
    frame_loop = next(n for n in tree.body if isinstance(n, ast.For)
                      and ast.unparse(n.target) == "frame")
    sub_loop = next(n for n in frame_loop.body if isinstance(n, ast.For)
                    and ast.unparse(n.target) == "sub")
    if len(sub_loop.body) != 3 or ast.unparse(sub_loop.body[1]) != "world.step(render=False)":
        raise ValueError("parent_controller_structure_changed")
    first, last = sub_loop.body[0], sub_loop.body[1]
    controller = textwrap.dedent("".join(lines[first.lineno - 1:last.end_lineno]))
    control_file = ("# Exact original actuator and world.step block; wrapper only.\n"
                    "def apply_controls_and_step(sub):\n"
                    " global sub_previous, sub_encoder, wheel_demand\n"
                    + textwrap.indent(controller, " "))
    prefix = original.split("# The worker receives only rendered camera frames;", 1)[0]
    if len(prefix) == len(original):
        raise ValueError("parent_initialization_anchor_changed")
    parser_line = next(line for line in lines if line.startswith("p=argparse.ArgumentParser();"))
    prefix = replace_once(prefix, parser_line, "# args is provided by the validation owner.\n")
    prefix = replace_once(prefix, "'anti_aliasing':0",
        "'anti_aliasing':0,'fast_shutdown':False,'active_gpu':0,'physics_gpu':0,'multi_gpu':False,'disable_viewport_updates':True,"
        "'extra_args':['--/app/hydraEngine/waitIdle=true','--/app/updateOrder/checkForHydraRenderComplete=1000']")
    startup_check = """
import carb
from omni.syntheticdata import SyntheticData
startup_settings=carb.settings.get_settings()
cached_order=SyntheticData.post_process_graph_tick_order()
observer_order=SyntheticData.Get()._post_process_graph_tick_sub.order
if startup_settings.get('/app/hydraEngine/waitIdle') is not True or startup_settings.get('/app/updateOrder/checkForHydraRenderComplete')!=1000 or cached_order!=1001 or observer_order!=1001:
 raise ValueError('v136_renderer_startup_order_contract_failed')
atomic_json(out/'renderer_startup_order.json',dict(wait_idle=True,render_complete_order=1000,cached_order=cached_order,observer_order=observer_order,phase='before_camera_creation',motion_authorized=False))
"""
    prefix = replace_once(prefix, "if small_viewport:\n import carb", startup_check + "\nif small_viewport:\n import carb")
    prefix = replace_once(prefix, "rendering_dt=1/20", "rendering_dt=1/physics_hz")
    prefix = replace_once(prefix, "capture_run_id=uuid.uuid4().hex",
                          "capture_run_id=validation_run_id")
    root_line = next(line for line in lines if line.startswith("root=Path(__file__)"))
    prefix = replace_once(prefix, root_line,
                          "root=asset_root;out=run_output\n")
    attach = """
from arm_practice.native_rgbd.mixed_camera_schema import MixedCameraSchema
native_schema=MixedCameraSchema.from_file(bundle_root/'configs/mixed_camera_schema.json')
native_rgb_refs={};native_depth_refs={};native_optics={};native_render_products=[]
def native_attach(frame,camera,rp,is_left):
 native_render_products.append(rp)
 ref=rep.AnnotatorRegistry.get_annotator('ReferenceTime');ref.attach(rp);native_rgb_refs[frame]=ref
 if is_left:
  ref=rep.AnnotatorRegistry.get_annotator('ReferenceTime');ref.attach(rp);native_depth_refs[frame.replace('_left_optical','_depth')]=ref
 native_optics[frame]={'resolution_wh':[int(x) for x in stage.GetPrimAtPath(rp.path).GetAttribute('resolution').Get()], 'clipping_range_m':[float(x) for x in camera.GetClippingRangeAttr().Get()], 'focal_length_mm':float(camera.GetFocalLengthAttr().Get()), 'horizontal_aperture_mm':float(camera.GetHorizontalApertureAttr().Get()), 'vertical_aperture_mm':float(camera.GetVerticalApertureAttr().Get())}
"""
    prefix = replace_once(prefix, "anns=[];segments=[];extrinsics={}", attach + "\nanns=[];segments=[];extrinsics={}")
    prefix = replace_once(prefix,
        "rp=rep.create.render_product(str(camera.GetPath()),(640,480))",
        "rp=rep.create.render_product(str(camera.GetPath()),tuple(native_schema.config['cameras'][frame]['resolution_wh']))")
    prefix = replace_once(prefix,
        ";seg=rep.AnnotatorRegistry.get_annotator('instance_id_segmentation_fast',device=annotator_device);seg.attach(rp);segments.append(seg)",
        "\n native_attach(frame,camera,rp,side=='left')")
    prefix = replace_once(prefix,
        "rp=rep.create.render_product(str(cam.GetPath()),(640,480))",
        "rp=rep.create.render_product(str(cam.GetPath()),tuple(native_schema.config['cameras'][optical]['resolution_wh']))")
    prefix = replace_once(prefix,
        "  seg=rep.AnnotatorRegistry.get_annotator('instance_id_segmentation_fast',device=annotator_device);seg.attach(rp);extra_segments[optical]=seg",
        "  native_attach(optical,cam,rp,side=='left')")
    maps = """
native_rgb={**{'r07_camera1_'+side+'_optical':anns[i] for i,side in enumerate(('left','right'))},**extra_cameras}
native_depth={'r07_camera1_depth':depth,**{name+'_depth':ann for name,ann in extra_depth.items()}}
native_schema.check_actual_optics(native_optics)
atomic_json(out/'native_actual_optics.json',dict(verified=True,readback=native_optics,camera_schema_sha256=native_schema.sha256,segmentation_annotators=0,raw_unmasked_only=True))
"""
    prefix = replace_once(prefix, "lidar_names=['front_right','rear_left','low_front_left','low_rear_right']",
                           maps + "\nlidar_names=['front_right','rear_left','low_front_left','low_rear_right']")
    ast.parse(prefix)
    ast.parse(control_file)
    return prefix, control_file, dict(
        original_parent_sha256=PARENT_SHA,
        actuator_block_sha256=hashlib.sha256(controller.encode()).hexdigest(),
        original_controller_lines=[first.lineno, last.end_lineno],
        changes=["separate asset/output/bundle roots", "schema render sizes",
                 "ReferenceTime and native mappings", "omit unused segmentation",
                 "omit legacy worker/socket/fixture sequence/main loop", "normal extension shutdown",
                 "render timestep follows native 960Hz physics; no timestamp rewriting"],
        scene_setup="original shared parent, no added support fixture or ground-truth controller",
    )


def prepare(output, *, isaac_python, settings, core=None):
    core = Path(core or EXPERIMENT.parents[1]).resolve(strict=True)
    output = Path(output).expanduser().resolve()
    if not output.is_relative_to(EXPERIMENT / ".runtime"):
        raise ValueError("validation_must_stay_inside_v136_runtime")
    if output.exists():
        raise FileExistsError(output)
    isaac_python = Path(isaac_python).expanduser().absolute()
    if not isaac_python.is_file():
        raise FileNotFoundError(isaac_python)
    settings = Path(settings).resolve(strict=True)
    prefix, controller, provenance = parent_parts(core / "scripts/r12_sensor_sim.py")
    assets = {}
    paths = [core / "configs/r12_gripper.json", core / "configs/sensors/camera_rig.json",
             core / "scripts/r12_sensor_sim.py", core / "models/swerve/policy.pt",
             core / "models/navigation/mission_scene.usda", core / "models/r12/r12_mobile_gripper.usda",
             core / "assets/vendor/R12_NO_USB_URDF_USD/usd/r12_no_usb.usdc",
             core / "assets/vendor/maze/maze.usda"]
    for folder in ASSET_FOLDERS:
        directory = core / folder
        if not directory.is_dir():
            raise FileNotFoundError(directory)
        paths.extend(p for p in directory.rglob("*") if p.is_file()
                     and "__pycache__" not in p.parts and p.suffix != ".pyc")
    for path in sorted(set(paths)):
        if not path.resolve().is_relative_to(core):
            raise ValueError("asset_symlink_outside_core:" + str(path))
        assets[str(path.relative_to(core))] = digest(path)
    # All input checks precede creation. A failed later preflight remains inspectable.
    output.mkdir(parents=True)
    bundle = assemble(output / "bundle")
    (output / "parent_initialization.py").write_text(prefix)
    (output / "parent_control.py").write_text(controller)
    (output / "runtime.json").write_bytes(settings.read_bytes())
    for name in ("isaac_validation_runtime.py", "run_isaac_validation.py",
                 "prepare_isaac_validation.py", "preflight_runtime.py", "assemble_bundle.py",
                 "verify_isaac_validation.py"):
        (output / name).write_bytes((EXPERIMENT / name).read_bytes())
    config = json.loads((output / "runtime.json").read_text())
    if config.get("ros_domain_id") != 33:
        raise ValueError("pa33_validation_requires_domain33")
    # A staged config must be independent of the source config's directory.
    for name, value in config.items():
        if name != "ros_domain_id" and value and not Path(value).expanduser().is_absolute():
            raise ValueError("validation_config_requires_absolute_paths:" + name)
    manifest = {p.name: digest(p) for p in output.iterdir() if p.is_file()}
    plan = dict(schema="v136-isaac-validation-plan-v1", run_id="pa33_v136_" + uuid.uuid4().hex[:12],
        prepared_only=True, native_verified=False, motion_authorized=False, coverage_required=True,
        asset_root=str(core), bundle=bundle, isaac_python=str(isaac_python),
        seed=0, physics_hz=960, initial_record_steps=960, sensor_warmup_renders=2, interval_steps=48,
        warmup_epochs=2, measurement_epochs=8, attempts=10,
        wall_budget_s=300, free_gpu_reserve_mib=1024, minimum_free_disk_mib=2048,
        output=str(output / "run"), inputs=manifest, assets=assets, provenance=provenance,
        limits=["stationary observation bench without v135 support fixture",
                "XYZ/XYZRGB/lidar are archived diagnostics, not additional ROS receipt acceptance",
                "native fault-injection and full mission remain separate validation",
                "ACK comparison is not performance acceptance; scene and first-packet IMU count differ"])
    (output / "plan.json").write_text(json.dumps(plan, indent=2) + "\n")
    return plan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=EXPERIMENT / "runtime.local.json")
    parser.add_argument("--isaac-python", type=Path,
        default=Path.home() / "pa-course/.venv-isaacsim/bin/python")
    args = parser.parse_args()
    plan = prepare(args.output, isaac_python=args.isaac_python, settings=args.config)
    print(json.dumps(dict(plan=str(args.output.resolve() / "plan.json"),
                         bundle=plan["bundle"], assets_pinned=len(plan["assets"]),
                         native_started=False), indent=2))


if __name__ == "__main__":
    main()
