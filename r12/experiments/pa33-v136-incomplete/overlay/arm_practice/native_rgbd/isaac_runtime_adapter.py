"""Bind v136 to an existing Isaac world on its owning thread.

Importing this module neither creates a world nor starts a child. Constructing
IsaacRuntimeAdapter explicitly starts the observation children. The supplied
physics_stepper retains the caller's controller and must advance exactly one
960 Hz step. No scene, fixture, target pose or actuator command is created here.
"""
from collections.abc import Mapping
import copy
import hashlib
import json
import math
from pathlib import Path
import threading
import time

import numpy as np

from .mixed_camera_schema import MixedCameraSchema
from .packet_timeline import NativePacketTimeline, validate_sample
from .postcapture_feedback import PostCaptureFeedback
from .runtime_owner import RuntimeCallbacks, build_observation_owner

PHYSICS_HZ = 960
CONTROL_STRIDE = 8
MAX_CURRENT_AGE_NS = 100_000_000
ARM_JOINTS = tuple("arm_joint" + str(i) for i in range(1, 6))


def _atomic_json(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False), encoding="utf-8")
    temporary.replace(path)


class IsaacRuntimeAdapter:
    """Connect initialized world/robot/annotators/IMUs; never instantiate Isaac.

    joint_names is the complete ordered articulation DOF list (the parent's
    'names'), not a guessed arm slice. imus must be the actual native sensors,
    ordered front then rear. rgb/depth and their ReferenceTime mappings must
    contain every entry in the bundle camera schema.

    All physics after construction must pass through step(sub), including
    postcapture refresh. The wrapped stepper must NOT also write issuer history
    or publish current feedback. observe() requires a 120 Hz sample boundary.
    """

    def __init__(self, *, root, out, run_id, clock_epoch, parent_source,
                 world, robot, joint_names, rig, rgb, depth,
                 rgb_reference, depth_reference, imus, physics_stepper,
                 settings=None, encoder_prime_steps=0, sensor_warmup_renders=0):
        self.thread_id = threading.get_ident()
        self.closed = False
        self.failed = False
        self.session = None
        self._refresh_sample = None
        self._latest_imus = None
        self._previous_encoder = None
        self._last_frame = -1
        self.world, self.robot, self.rig = world, robot, rig
        self.physics_stepper = physics_stepper
        self.imus = tuple(imus)
        if not callable(physics_stepper) or not callable(getattr(rig, "transforms", None)):
            raise ValueError("actual_stepper_and_camera_fk_required")
        for obj, methods in ((world, ("render",)), (robot, ("get_joint_positions", "get_joint_velocities"))):
            if any(not callable(getattr(obj, name, None)) for name in methods):
                raise ValueError("initialized_isaac_world_and_robot_required")
        if len(self.imus) != 2 or self.imus[0] is self.imus[1]:
            raise ValueError("distinct_front_rear_native_imus_required")
        for imu in self.imus:
            if not callable(getattr(getattr(imu, "_imu_sensor_interface", None), "get_sensor_reading", None)):
                raise ValueError("native_imu_interface_required")
            if not getattr(imu, "prim_path", None):
                raise ValueError("native_imu_prim_path_required")
        if self.imus[0].prim_path == self.imus[1].prim_path:
            raise ValueError("distinct_native_imu_paths_required")
        self.joint_names = tuple(joint_names)
        if len(set(self.joint_names)) != len(self.joint_names):
            raise ValueError("duplicate_articulation_joint_names")
        if any(self.joint_names.count(name) != 1 for name in ARM_JOINTS):
            raise ValueError("actual_arm_joint_names_required")
        self.arm = tuple(self.joint_names.index(name) for name in ARM_JOINTS)
        self.root = Path(root).expanduser().resolve(strict=True)
        schema = MixedCameraSchema.from_file(self.root / "configs/mixed_camera_schema.json")
        for mapping, expected, name in (
            (rgb, schema.config["cameras"], "rgb"),
            (depth, schema.config["depth"], "depth"),
            (rgb_reference, schema.config["cameras"], "rgb_reference"),
            (depth_reference, schema.config["depth"], "depth_reference"),
        ):
            if not isinstance(mapping, Mapping) or set(mapping) != set(expected):
                raise ValueError("isaac_schema_annotator_mismatch:" + name)
        self.parent_source = Path(parent_source).expanduser().resolve(strict=True)
        source_bytes = self.parent_source.read_bytes()
        self.parent_sha = hashlib.sha256(source_bytes).hexdigest()
        self._primed_history = self._prime_encoder_history(encoder_prime_steps)
        self.initial_epoch = self.read_epoch()
        if type(sensor_warmup_renders) is not int or sensor_warmup_renders not in (0, 2):
            raise ValueError("sensor_warmup_renders_must_be_zero_or_two")
        for _ in range(sensor_warmup_renders):
            self.world.render()
            if self.read_epoch() != self.initial_epoch:
                raise ValueError("physics_advanced_during_sensor_warmup")
        self._last_step = self.initial_epoch[0]
        self.timeline = NativePacketTimeline(run_id, self._last_step)
        self._sample_control_encoder()
        callbacks = RuntimeCallbacks(
            rgb=rgb, depth=depth, rgb_reference=rgb_reference,
            depth_reference=depth_reference, read_epoch=self.read_epoch,
            read_joint_positions=robot.get_joint_positions,
            arm_indices=self.arm, render=world.render, fk=rig.transforms,
            refresh_packet=self.refresh_packet,
        )
        # No existing output/logs are overwritten by attaching another session.
        self.out = Path(out).expanduser().resolve()
        self.out.mkdir(parents=True, exist_ok=False)
        (self.out / "parent_runtime_source.py").write_bytes(source_bytes)
        if self._primed_history:
            _atomic_json(self.out / "pre_session_encoder_history.json", dict(
                source="actual_parent_stepper_and_native_joint_getters",
                packet_interval_starts_after_history=True, samples=self._primed_history))
        try:
            self.session = build_observation_owner(
                root=self.root, out=self.out, run_id=run_id,
                clock_epoch=clock_epoch, callbacks=callbacks, settings=settings,
            )
            for row in self._primed_history:
                self.session.issuer.record_encoder_epoch(**row)
            if self.read_epoch() != self.initial_epoch:
                raise ValueError("isaac_epoch_advanced_during_attach")
            _atomic_json(self.out / "isaac_connection.json", {
                "run_id": run_id, "clock_epoch": clock_epoch,
                "source_sha256": self.session.source_sha,
                "parent_source": str(self.parent_source),
                "parent_source_sha256": self.parent_sha,
                "initial_physics_step": self.initial_epoch[0],
                "pre_session_encoder_steps": len(self._primed_history),
                "pre_session_sensor_warmup_renders": sensor_warmup_renders,
                "all_packet_IMUs_retained_since_initial_physics_step": True,
                "physics_hz": PHYSICS_HZ,
                "control_sample_hz": 120,
                "arm_indices": list(self.arm),
                "imu_prim_paths": [imu.prim_path for imu in self.imus],
                "rgb": sorted(rgb), "depth": sorted(depth),
                "camera_schema_sha256": schema.sha256,
                "scope": "caller_owned_isaac_callback_connection",
                "native_validation_completed": False,
                "motion_authorized": False,
            })
        except Exception as error:
            self.failed = True
            if self.session is not None:
                try:
                    self.session.close()
                except Exception as close_error:
                    error.add_note("isaac_attach_cleanup:" + str(close_error))
            raise

    def _prime_encoder_history(self, count):
        """Optional actual pre-session settling; no packet interval exists yet."""
        if type(count) is not int or count not in (0, 960):
            raise ValueError("encoder_prime_steps_must_be_zero_or_960")
        history = []
        for sub in range(count):
            before = self.read_epoch()
            q0 = self._arm_values(self.robot.get_joint_positions)
            qd0 = self._arm_values(self.robot.get_joint_velocities)
            self.physics_stepper(sub)
            after = self.read_epoch()
            start = time.monotonic_ns()
            raw = np.asarray(self.robot.get_joint_positions()).copy()
            end = time.monotonic_ns()
            if raw.shape != (len(self.joint_names),) or not np.isfinite(raw).all():
                raise ValueError("invalid_actual_articulation_values")
            q1 = raw[list(self.arm)].astype(float)
            qd1 = self._arm_values(self.robot.get_joint_velocities)
            if self.read_epoch() != after or after[0] != before[0] + 1 or after[1] <= before[1]:
                raise ValueError("invalid_pre_session_physics_step")
            measured = np.rint(raw[list(self.arm)] * 4096 / (2 * np.pi)) * 2 * np.pi / 4096
            history.append(dict(physics_step=after[0], sim_time_s=after[1],
                read_start_ns=start, read_end_ns=end, measured_encoder=measured.tolist(),
                native_qd=np.maximum(np.abs(qd0), np.abs(qd1)).tolist(),
                same_step_fd=((q1 - q0) / (after[1] - before[1])).tolist()))
        return history

    def _require_owner(self):
        if threading.get_ident() != self.thread_id:
            raise ValueError("isaac_owner_thread_required")
        if self.closed:
            raise ValueError("isaac_connection_closed")
        if self.failed:
            raise ValueError("isaac_connection_failed")
        if self.session is not None:
            self.session._require_owner()
            if self.session.owner.state == "failed":
                raise ValueError("isaac_observation_owner_failed")

    def read_epoch(self):
        step = self.world.current_time_step_index
        if isinstance(step, np.integer):
            step = int(step)
        sim_time = float(self.world.current_time)
        if type(step) is not int or step < 0 or not math.isfinite(sim_time):
            raise ValueError("invalid_isaac_epoch")
        if abs(sim_time - step / PHYSICS_HZ) > 1 / (4 * PHYSICS_HZ):
            raise ValueError("isaac_runtime_requires_960hz")
        return step, sim_time

    def _arm_values(self, method):
        values = np.asarray(method(), dtype=float)
        if values.shape != (len(self.joint_names),) or not np.isfinite(values).all():
            raise ValueError("invalid_actual_articulation_values")
        return values[list(self.arm)].copy()

    def _sample_control_encoder(self):
        step, sim_time = self.read_epoch()
        start = time.monotonic_ns()
        raw = np.asarray(self.robot.get_joint_positions()).copy()
        if raw.shape != (len(self.joint_names),) or not np.isfinite(raw).all():
            raise ValueError("invalid_actual_articulation_values")
        # Match NativeMissionCapture exactly: quantize in the native getter dtype.
        q = np.rint(raw[list(self.arm)] * 4096 / (2 * np.pi)) * 2 * np.pi / 4096
        end = time.monotonic_ns()
        if self.read_epoch() != (step, sim_time):
            raise ValueError("physics_advanced_during_control_sample")
        old = self._previous_encoder
        if old is not None and old["step"] == step:
            # Reusing the same sample must never refresh its wall-clock age.
            if old["q"] != q.tolist():
                raise ValueError("encoder_changed_within_same_epoch")
            return copy.deepcopy(old)
        row = dict(q=q.tolist(), step=step, sim_time_s=sim_time,
                   read_start_ns=start, read_end_ns=end,
                   velocity=None, velocity_valid=False)
        if old is not None:
            dt = sim_time - old["sim_time_s"]
            if step - old["step"] != CONTROL_STRIDE or abs(dt - 1 / 120) > 1e-8:
                raise ValueError("missing_actual_120hz_encoder_sample")
            delta = (q - np.asarray(old["q"]) + np.pi) % (2 * np.pi) - np.pi
            row.update(velocity=(delta / dt).tolist(), velocity_valid=True)
        self._previous_encoder = row
        return copy.deepcopy(row)

    def _sample_native_imus(self):
        epoch = self.read_epoch()
        samples = []
        for imu in self.imus:
            native = imu._imu_sensor_interface.get_sensor_reading(
                imu.prim_path, use_latest_data=True, read_gravity=True,
            )
            sample = dict(
                time=float(native.time), valid=bool(native.is_valid),
                gyro=[float(native.ang_vel_x), float(native.ang_vel_y), float(native.ang_vel_z)],
                accel=[float(native.lin_acc_x), float(native.lin_acc_y), float(native.lin_acc_z)],
                physics_step=epoch[0], physics_time_s=epoch[1],
            )
            # Preserve actual sensor time, including its float32 phase.
            validate_sample(sample)
            samples.append(sample)
        if self.read_epoch() != epoch:
            raise ValueError("physics_advanced_during_native_imu_read")
        self.timeline.add([[sample] for sample in samples])
        self._latest_imus = samples
        return copy.deepcopy(samples)

    def _current_feedback(self):
        position = self.session.capture.read_current_feedback()
        now = time.monotonic_ns()
        velocity = copy.deepcopy(self._previous_encoder)
        if velocity is None or velocity["velocity_valid"] is not True:
            raise ValueError("actual_control_encoder_velocity_required")
        if velocity["step"] != position["physics_step"] or velocity["q"] != position["q"]:
            _atomic_json(self.out / "current_feedback_mismatch.json", dict(position=position, velocity=velocity))
            raise ValueError("current_position_velocity_epoch_mismatch")
        position_age = now - position["read_start_ns"]
        velocity_age = now - velocity["read_start_ns"]
        if not 0 <= position_age <= MAX_CURRENT_AGE_NS or not 0 <= velocity_age <= MAX_CURRENT_AGE_NS:
            raise ValueError("actual_control_feedback_stale")
        velocity.update(age_ns=velocity_age, valid=True, reason="fresh")
        return dict(
            position=position, position_age_ns=position_age, position_valid=True,
            velocity=velocity, control_admitted=False,
            reason="observation_transport_only_no_actuator_admission",
            historical_q_used_as_current=False,
        )

    def step(self, sub):
        """Run the parent's real controller/stepper once and record that epoch."""
        self._require_owner()
        try:
            before = self.read_epoch()
            if before[0] != self._last_step:
                raise ValueError("physics_advanced_outside_v136_step")
            q0 = self._arm_values(self.robot.get_joint_positions)
            qd0 = self._arm_values(self.robot.get_joint_velocities)
            self.physics_stepper(sub)
            after = self.read_epoch()
            q1 = self._arm_values(self.robot.get_joint_positions)
            qd1 = self._arm_values(self.robot.get_joint_velocities)
            if after[0] != before[0] + 1 or after[1] <= before[1]:
                raise ValueError("isaac_stepper_must_advance_exactly_one_step")
            dt = after[1] - before[1]
            native_qd = np.maximum(np.abs(qd0), np.abs(qd1)).tolist()
            same_step_fd = ((q1 - q0) / dt).tolist()
            if self._refresh_sample is None:
                self.session.record_physics_step(native_qd=native_qd, same_step_fd=same_step_fd)
            else:
                self.session.record_postcapture_step(
                    self._refresh_sample, native_qd=native_qd, same_step_fd=same_step_fd,
                )
            self._last_step = after[0]
            if (after[0] - self.initial_epoch[0]) % CONTROL_STRIDE == 0:
                self._sample_control_encoder()
                self._sample_native_imus()
        except Exception:
            self.failed = True
            raise

    def refresh_packet(self, sample):
        """Refresh actual sensors; RuntimePublicationPrepare alone sends feedback."""
        self._require_owner()
        if self._refresh_sample is not None:
            raise ValueError("nested_isaac_refresh")
        owner = self.session.owner
        if owner.state != "authenticating" or owner.pending is not sample or self.session.prepare.pending is not sample:
            raise ValueError("refresh_requires_owned_pending_capture")
        body = sample["certificate"]["body"]
        before_bytes = json.dumps(body, sort_keys=True, allow_nan=False).encode()
        if body["run_id"] != self.session.issuer.run_id or body["capture_physics_step"] != self._last_step:
            raise ValueError("isaac_capture_current_epoch_mismatch")
        self._refresh_sample = sample
        try:
            sequence = PostCaptureFeedback(
                capture_step=body["capture_physics_step"],
                capture_lower_bound_ns=body["capture_monotonic_lower_bound_ns"],
                capture_completed_ns=body["issued_ns"], physics_hz=PHYSICS_HZ,
            )

            def read_imus():
                current = self.read_epoch()[0]
                if self._latest_imus is None or any(s["physics_step"] != current for s in self._latest_imus):
                    raise ValueError("postcapture_native_imu_epoch_missing")
                return copy.deepcopy(self._latest_imus)

            result = sequence.run(
                stepper=self.step, read_step=lambda: self.read_epoch()[0],
                sample_encoder=self._sample_control_encoder, read_imus=read_imus,
            )
            if result["imu_distinct_120hz_valid"] is not True:
                raise ValueError("postcapture_native_imu_cadence_invalid")
            if json.dumps(body, sort_keys=True, allow_nan=False).encode() != before_bytes:
                raise ValueError("historical_capture_changed_during_refresh")
            current = self._current_feedback()
            timing = self.timeline.packet(current_step=self.read_epoch()[0], capture_body=body)
            return dict(run_id=body["run_id"], native_packet_timeline=timing,
                        current_feedback=current, all_IMU_samples=timing["imu"])
        except Exception:
            self.failed = True
            raise
        finally:
            self._refresh_sample = None

    def observe(self, frame):
        self._require_owner()
        if type(frame) is not int or frame < 0 or frame <= self._last_frame:
            raise ValueError("isaac_observation_frame_order")
        epoch = self.read_epoch()
        if epoch[0] != self._last_step:
            raise ValueError("physics_advanced_outside_v136_step")
        if self._previous_encoder["step"] != epoch[0] or not self._previous_encoder["velocity_valid"]:
            raise ValueError("observe_requires_actual_120hz_sample_boundary")
        if any(len(stream) + 2 > self.timeline.max_pending_samples for stream in self.timeline.pending):
            raise ValueError("native_imu_budget_requires_shorter_packet_interval")
        if hashlib.sha256(self.parent_source.read_bytes()).hexdigest() != self.parent_sha:
            raise ValueError("parent_runtime_source_changed")
        try:
            self._last_frame = frame
            return self.session.observe(frame)
        except Exception:
            self.failed = True
            raise

    def close(self):
        if threading.get_ident() != self.thread_id:
            raise ValueError("isaac_owner_thread_required")
        if self.closed:
            return
        try:
            self.session.close()
        finally:
            self.closed = True

    def __enter__(self):
        self._require_owner()
        return self

    def __exit__(self, kind, error, traceback):
        try:
            self.close()
        except Exception as close_error:
            if error is None:
                raise
            error.add_note("isaac_runtime_close:" + str(close_error))
        return False


def connect_sensor_runtime(namespace, *, root, out, parent_source, clock_epoch, settings=None):
    """Install v136 hooks into an initialized sensor-runtime namespace.

    The key names are those of the inspected v135 sensor entrypoint. This
    function does not execute that entrypoint or copy its fixture/controller.
    Call it after the parent's actual stepper and annotators have been built,
    before the sampling loop, with the legacy producer/worker disabled.
    """
    required = (
        "world", "robot", "names", "prepared_camera_rig", "native_rgb",
        "native_depth", "native_rgb_refs", "native_depth_refs", "imus",
        "apply_controls_and_step", "capture_run_id",
    )
    if not isinstance(namespace, dict):
        raise ValueError("sensor_runtime_namespace_required")
    missing = [name for name in required if name not in namespace]
    if missing:
        raise ValueError("sensor_runtime_binding_missing:" + ",".join(missing))
    if namespace.get("native_producer") is not None or namespace.get("worker") is not None:
        raise ValueError("legacy_observer_or_worker_already_started")
    if any(namespace.get(name, False) for name in ("native_producer_enabled", "perception_enabled")):
        raise ValueError("legacy_observer_paths_must_be_disabled")
    if "v136_runtime" in namespace:
        raise ValueError("v136_runtime_already_attached")
    parent_stepper = namespace["apply_controls_and_step"]
    adapter = IsaacRuntimeAdapter(
        root=root, out=out, run_id=namespace["capture_run_id"],
        clock_epoch=clock_epoch, parent_source=parent_source,
        world=namespace["world"], robot=namespace["robot"],
        joint_names=namespace["names"], rig=namespace["prepared_camera_rig"],
        rgb=namespace["native_rgb"], depth=namespace["native_depth"],
        rgb_reference=namespace["native_rgb_refs"],
        depth_reference=namespace["native_depth_refs"], imus=namespace["imus"],
        physics_stepper=parent_stepper, settings=settings,
        encoder_prime_steps=namespace.get("v136_encoder_prime_steps", 0),
        sensor_warmup_renders=namespace.get("v136_sensor_warmup_renders", 0),
    )
    namespace["v136_runtime"] = adapter
    namespace["apply_controls_and_step"] = adapter.step
    return adapter
