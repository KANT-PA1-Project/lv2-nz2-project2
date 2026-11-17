"""Native worker for a prepared plan; run through run_isaac_validation.py --execute."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import sys
import time
from types import SimpleNamespace


def atomic(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def archive_diagnostics(namespace, sample, output, frame):
    """Read actual lidar and project captured RGBD using each camera's own K."""
    import numpy as np
    from pxr import UsdGeom
    schema = namespace["native_schema"]
    body = sample["certificate"]["body"]
    clouds = {}
    summaries = {}
    for name, spec in schema.config["depth"].items():
        camera = spec["rgb_camera"]
        depth = sample["depths"][name]
        rgb = sample["pixels"][camera]
        fx, fy, cx, cy = schema.config["cameras"][camera]["intrinsics"]
        v, u = np.mgrid[0:depth.shape[0]:16, 0:depth.shape[1]:16]
        z = depth[::16, ::16]
        good = np.isfinite(z) & (z > .05) & (z < 5.)
        # Optical camera coordinates; no fixture or simulator target truth.
        with np.errstate(invalid="ignore"):
            xyz = np.stack(((u - cx) * z / fx, (v - cy) * z / fy, z), axis=-1)[good].astype("<f4")
        colors = rgb[::16, ::16][good].astype("uint8")
        clouds[name + "_xyz"] = xyz
        clouds[name + "_rgb"] = colors
        summaries[name] = dict(points=len(xyz), frame=camera, stride=16,
                               capture_physics_step=body["capture_physics_step"])
    np.savez(output / ("clouds_%02d.npz" % frame), **clouds)
    stage = namespace["stage"]
    base = np.array(UsdGeom.Xformable(stage.GetPrimAtPath(
        "/World/Spawn/R12/base_link")).ComputeLocalToWorldTransform(0)).T
    lidar_read_start = time.monotonic_ns()
    scans = {}
    for name, local in namespace["lidar_T"].items():
        transform = base @ local
        distances = []
        for angle in np.linspace(-np.pi / 2, np.pi / 2, 90):
            direction = transform[:3, :3] @ np.array((np.cos(angle), np.sin(angle), 0))
            hits = []
            def hit(reading):
                if "/R12/" not in str(reading.rigid_body):
                    hits.append(float(reading.distance))
                return True
            namespace["query"].raycast_all(tuple(transform[:3, 3]), tuple(direction), 6., hit)
            distances.append(min(hits) if hits else 6.)
        scans[name] = distances
    lidar_read_end = time.monotonic_ns()
    lidar_step = int(namespace["world"].current_time_step_index)
    atomic(output / ("sensor_coverage_%02d.json" % frame), dict(
        captured_clouds=summaries, lidar=scans,
        lidar_physics_step=int(namespace["world"].current_time_step_index),
        capture_physics_step=body["capture_physics_step"],
        scope="archived actual RGBD projection and current raycast, not ROS topic receipt",
        full_sensor_ROS_verified=False, motion_authorized=False))
    return dict(lidar=scans, lidar_step=lidar_step,
                read_start_ns=lidar_read_start, read_end_ns=lidar_read_end)


def check_runtime_rejections(connection, output):
    """Exercise non-advancing owner guards on the actual initialized connection."""
    import threading
    epoch = connection.read_epoch()
    rows = []
    def rejected(name, action, reason):
        try:
            action()
        except ValueError as error:
            if reason not in str(error):
                raise
            rows.append(dict(case=name, rejected=True, reason=str(error)))
        else:
            raise RuntimeError("expected_runtime_rejection_missing:" + name)
        if connection.read_epoch() != epoch:
            raise RuntimeError("rejection_advanced_physics:" + name)
    rejected("invalid_frame", lambda: connection.observe(-1), "frame_order")
    rejected("unowned_capture", lambda: connection.refresh_packet({}), "owned_pending")
    original = connection.session.priority.source
    try:
        connection.session.priority.source = "0" * 64
        rejected("source_mismatch", lambda: connection.step(0), "source_identity_changed")
    finally:
        connection.session.priority.source = original
    thread_errors = []
    def foreign_thread():
        try:
            connection.step(0)
        except BaseException as error:
            thread_errors.append(error)
    worker = threading.Thread(target=foreign_thread)
    worker.start()
    worker.join(timeout=2)
    if worker.is_alive() or len(thread_errors) != 1 or "owner_thread_required" not in str(thread_errors[0]):
        raise RuntimeError("cross_thread_rejection_missing")
    rows.append(dict(case="cross_thread", rejected=True, reason=str(thread_errors[0])))
    if connection.read_epoch() != epoch or connection.session.owner.state != "idle":
        raise RuntimeError("rejection_probe_changed_runtime")
    atomic(output / "runtime_rejections.json", dict(
        cases=rows, epoch_unchanged=True,
        scope="actual connection owner guards only; no malformed ROS/IPC packet injection",
        native_wire_fault_injection_verified=False))


def run(plan_path):
    from run_isaac_validation import verify_plan
    plan = verify_plan(plan_path)
    stage_dir = Path(plan_path).resolve().parent
    if os.environ.get("R12_V136_SUPERVISED") != plan["run_id"]:
        raise ValueError("use_supervised_launcher")
    bundle = Path(plan["bundle"]["root"])
    output = Path(plan["output"])
    output.mkdir(exist_ok=False)
    # No inherited experiment options: the supervisor supplies a fixed environment.
    sys.path.insert(0, str(bundle / "src"))
    import numpy as np
    np.random.seed(plan["seed"])
    from arm_practice.native_rgbd.isaac_runtime_adapter import connect_sensor_runtime
    from arm_practice.native_rgbd.runtime_settings import RuntimeSettings
    from arm_practice.native_rgbd.mission_capture import plain
    namespace = dict(__name__="__v136_owned_parent__",
        __file__=str(stage_dir / "parent_initialization.py"),
        args=SimpleNamespace(frames=plan["attempts"], port=0),
        asset_root=Path(plan["asset_root"]), bundle_root=bundle,
        run_output=output, validation_run_id=plan["run_id"])
    connection = None
    records = []
    errors = []
    children = {}
    app_closed = False
    original_acquire = None
    latest = [None]
    def interrupted(signum, frame):
        raise KeyboardInterrupt("signal:" + str(signum))
    signal.signal(signal.SIGTERM, interrupted)
    try:
        exec(compile((stage_dir / "parent_initialization.py").read_text(),
                     namespace["__file__"], "exec"), namespace)
        # This is the original stationary branch: hold the measured startup q.
        namespace["q"] = namespace["stationary_reference"].copy()
        namespace["vel"][:] = 0.
        namespace["integral_torque"] = np.zeros(4)
        namespace["drive_substeps"] = []
        exec(compile((stage_dir / "parent_control.py").read_text(),
                     str(stage_dir / "parent_control.py"), "exec"), namespace)
        reference_reads = []
        class ReferenceAudit:
            def __init__(self, name, actual):
                self.name, self.actual = name, actual
            def get_data(self):
                raw = self.actual.get_data()
                reference_reads.append(dict(name=self.name, data=plain(raw),
                    step=int(namespace["world"].current_time_step_index)))
                return raw
            def detach(self):
                return self.actual.detach()
        for group in ("native_rgb_refs", "native_depth_refs"):
            namespace[group] = {name:ReferenceAudit(name, actual)
                                for name,actual in namespace[group].items()}
        namespace["v136_encoder_prime_steps"] = plan["initial_record_steps"]
        namespace["v136_sensor_warmup_renders"] = plan["sensor_warmup_renders"]
        namespace["perception_enabled"] = False
        namespace["native_producer_enabled"] = False
        settings = RuntimeSettings.from_env({"R12_RUNTIME_CONFIG": str(stage_dir / "runtime.json")})
        connection = connect_sensor_runtime(namespace, root=bundle, out=output / "observer",
            parent_source=stage_dir / "parent_initialization.py",
            clock_epoch=plan["run_id"] + "-physics-epoch1", settings=settings)
        for name in ("decoder", "ros", "priority"):
            resource = getattr(connection.session, name)
            children[name] = resource.child
        atomic(output / "owned_processes.json", dict(
            parent_pid=os.getpid(), children={name:child.pid for name,child in children.items()}))
        latest = [None]
        original_acquire = connection.session.capture.acquire
        def retain_capture():
            result = original_acquire()
            latest[0] = result
            return result
        connection.session.capture.acquire = retain_capture
        check_runtime_rejections(connection, output)
        coverage_sequence = 0
        for frame in range(plan["attempts"]):
            if (output / "STOP").exists():
                raise RuntimeError("stopped_by_file")
            namespace["contact_frame"][0] = frame
            namespace["drive_substeps"].clear()
            latest[0] = None
            reference_reads.clear()
            import gc
            def physical_state():
                return (connection.read_epoch(),
                    namespace["robot"].get_joint_positions().tolist(),
                    namespace["robot"].get_joint_velocities().tolist())
            before_gc = physical_state()
            gc_start = time.monotonic_ns()
            collected = gc.collect()
            gc_end = time.monotonic_ns()
            if physical_state() != before_gc:
                raise RuntimeError("stationary_gc_changed_native_state")
            atomic(output / ("gc_barrier_%02d.json" % frame),
                   dict(collected=collected, start_ns=gc_start, end_ns=gc_end,
                        actual_state_unchanged=True, before_next_physics_and_capture=True))
            for sub in range(plan["interval_steps"]):
                connection.step(sub)
            result = connection.observe(frame)
            atomic(output / ("reference_readback_%02d.json" % frame), reference_reads)
            record = dict(frame=frame,
                phase="warmup" if frame < plan["warmup_epochs"] else "measurement",
                completed_monotonic_ns=time.monotonic_ns(), result=plain(result))
            # These writes run after ACK/decoder consumption and never refresh capture time.
            sample = latest[0]
            if sample is not None and sample.get("captured"):
                atomic(output / ("certificate_%02d.json" % frame), plain(sample["certificate"]))
                np.savez(output / ("raw_%02d.npz" % frame), **sample["pixels"], **sample["depths"])
                from arm_practice.native_rgbd.coverage_transport import make_payload, seal
                current_lidar = archive_diagnostics(namespace, sample, output, frame)
                coverage_sequence += 1
                payload = make_payload(sample, namespace["native_schema"],
                                       sequence=coverage_sequence, **current_lidar)
                record["coverage_ROS"] = connection.session.ros.coverage_exchange(payload)
                atomic(output / ("coverage_payload_%02d.json" % frame), payload)
                if not (output/"coverage_native_rejections.json").exists():
                    import copy, secrets
                    probes = []
                    bad = copy.deepcopy(payload)
                    bad["nonce"] = secrets.token_hex(16)
                    bad["sequence"] = coverage_sequence+1
                    bad["source_sha256"] = "0"*64
                    probes.append(("source_mismatch",seal(bad),"source_or_run_mismatch"))
                    bad = copy.deepcopy(payload)
                    bad["nonce"],bad["sequence"] = secrets.token_hex(16),coverage_sequence+1
                    bad["payload_sha256"] = "0"*64
                    probes.append(("hash_mismatch",bad,"payload_hash_mismatch"))
                    probes.append(("nonce_replay",payload,"replayed_nonce"))
                    bad = copy.deepcopy(payload)
                    bad["nonce"],bad["sequence"] = secrets.token_hex(16),coverage_sequence+1
                    bad["lidar_read_start_ns"] -= 200_000_000
                    bad["lidar_read_end_ns"] -= 200_000_000
                    probes.append(("stale_lidar",seal(bad),"stale_or_invalid_time"))
                    rejected = []
                    for label, invalid, expected in probes:
                        rejection = connection.session.ros.coverage_exchange(invalid,expect_rejection=True)
                        if expected not in rejection.get("reason",""):
                            raise RuntimeError("coverage_wrong_rejection:"+str(rejection))
                        rejected.append(dict(case=label,**rejection))
                    atomic(output/"coverage_native_rejections.json",rejected)
            records.append(record)
            atomic(output / "observations.json", records)
            print("V136_OBSERVATION", frame, result["captured"], flush=True)
    except BaseException as error:
        import traceback
        errors.append(type(error).__name__ + ":" + str(error))
        if latest[0] is not None and latest[0].get("captured"):
            atomic(output / "failed_certificate.json", plain(latest[0]["certificate"]))
            np.savez(output / "failed_raw.npz", **latest[0]["pixels"], **latest[0]["depths"])
        atomic(output / "failure.json", dict(errors=errors, traceback=traceback.format_exc()))
        traceback.print_exc()
    finally:
        if connection is not None:
            try:
                connection.close()
            except BaseException as error:
                errors.append("close:" + str(error))
        for name, child in children.items():
            child.poll()
        try:
            annotations = {}
            for group in ("native_rgb", "native_depth", "native_rgb_refs", "native_depth_refs"):
                for annotation in namespace.get(group, {}).values():
                    annotations[id(annotation)] = annotation
            for annotation in annotations.values():
                annotation.detach()
            for product in namespace.get("native_render_products", []):
                product.destroy()
        except BaseException as error:
            errors.append("sensor_detach:" + str(error))
        try:
            if namespace.get("world") is not None:
                namespace["world"].stop()
                namespace["world"].clear()
                type(namespace["world"]).clear_instance()
        except BaseException as error:
            errors.append("world_stop:" + str(error))
        atomic(output / "shutdown_pre_app.json", dict(errors=errors,
            child_exit_codes={name:child.returncode for name,child in children.items()},
            app_closed=False, clean_teardown=False))
        try:
            if namespace.get("app") is not None:
                import gc
                app = namespace.pop("app")
                namespace.clear()
                connection = None
                original_acquire = None
                latest[0] = None
                gc.collect()
                app.close()
                app_closed = True
                del app
                gc.collect()
        except BaseException as error:
            errors.append("app_close:" + str(error))
        child_codes = {name: child.returncode for name,child in children.items()}
        all_captured = len(records) == plan["attempts"] and all(
            row["result"].get("captured") is True for row in records)
        all_ros = all_captured and all(row["result"]["ROS"].get("all_received") is True
            and row["result"]["ROS"].get("raw_bytes_hash_verified") == 12 for row in records)
        clean = not errors and app_closed and len(child_codes) == 3 and all(
            code == 0 for code in child_codes.values())
        atomic(output / "shutdown.json", dict(
            run_id=plan["run_id"], errors=errors, child_exit_codes=child_codes,
            app_closed=app_closed, clean_teardown=clean,
            all_attempts_captured=all_captured, all_required_ROS_received=all_ros,
            observation_bench_pass=clean and all_ros,
            full_pa33_acceptance=False, native_fault_injection_verified=False,
            mission_success=False, performance_accepted=False, motion_authorized=False))
    if not clean:
        raise RuntimeError("native_validation_failed:" + ";".join(errors))
    if not all_ros:
        raise RuntimeError("required_native_observations_not_completed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    args = parser.parse_args()
    run(args.plan)


if __name__ == "__main__":
    main()
