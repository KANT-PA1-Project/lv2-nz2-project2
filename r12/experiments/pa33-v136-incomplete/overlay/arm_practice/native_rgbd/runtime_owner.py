"""Assemble a caller-owned observation runtime; no fixture/world creation or autostart."""
from collections.abc import Mapping
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import threading
import time

from .live_decode_bridge import LiveDecodeBridge
from .live_publication_bridge import LivePublicationBridge
from .mission_capture import NativeMissionCapture
from .mixed_camera_schema import MixedCameraSchema
from .observation_owner import ObservationOwner
from .priority_feedback_bridge import PriorityFeedbackBridge
from .renderer_epoch_issuer import RendererEpochIssuer, canonical
from .runtime_prepare import RuntimePublicationPrepare
from .runtime_settings import RuntimeSettings


@dataclass(frozen=True)
class RuntimeCallbacks:
    rgb: Mapping
    depth: Mapping
    rgb_reference: Mapping
    depth_reference: Mapping
    read_epoch: object
    read_joint_positions: object
    arm_indices: tuple
    render: object
    fk: object
    refresh_packet: object
    clock: object = time.monotonic_ns

    def __post_init__(self):
        for name in ("rgb", "depth", "rgb_reference", "depth_reference"):
            if not isinstance(getattr(self, name), Mapping):
                raise ValueError("runtime_annotator_mapping_required:" + name)
            if any(not callable(getattr(value, "get_data", None)) for value in getattr(self, name).values()):
                raise ValueError("runtime_annotator_get_data_required:" + name)
        for name in ("read_epoch", "read_joint_positions", "render", "fk", "refresh_packet", "clock"):
            if not callable(getattr(self, name)):
                raise ValueError("runtime_callback_required:" + name)


def bundle_source_sha(root):
    root = Path(root).resolve(strict=True)
    # The source identity must describe the bundle this adapter is running from.
    if Path(__file__).resolve() != root / "src/arm_practice/native_rgbd/runtime_owner.py":
        raise ValueError("runtime_loaded_outside_bundle")
    raw = (root / "native_entrypoint_manifest.json").read_bytes()
    manifest = json.loads(raw)
    if manifest.get("schema") != "v136-adapter-bundle-v1":
        raise ValueError("invalid_bundle_manifest")
    for name, entry in manifest["files"].items():
        target = (root / name).resolve(strict=True)
        if not target.is_relative_to(root):
            raise ValueError("bundle_path_escape")
        if hashlib.sha256(target.read_bytes()).hexdigest() != entry["sha256"]:
            raise ValueError("bundle_file_hash_mismatch:" + name)
    return hashlib.sha256(raw).hexdigest()


class ObservationSession:
    def __init__(self, *, root, source_sha, issuer, capture, prepare, decoder, ros, priority):
        self.root = Path(root)
        self.source_sha = source_sha
        self.issuer = issuer
        self.capture = capture
        self.prepare = prepare
        self.decoder = decoder
        self.ros = ros
        self.priority = priority
        self.owner = ObservationOwner(capture=capture, prepare_publication=prepare, decoder=decoder, ros=ros)
        self.thread_id = threading.get_ident()
        self.closed = False

    def _require_owner(self, *, verify_manifest=False):
        if threading.get_ident() != self.thread_id:
            raise ValueError("runtime_owner_thread_required")
        if self.closed:
            raise ValueError("runtime_session_closed")
        if any(value != self.source_sha for value in (self.issuer.source, self.ros.source_sha, self.priority.source)):
            raise ValueError("runtime_source_identity_changed")
        if verify_manifest and hashlib.sha256((self.root / "native_entrypoint_manifest.json").read_bytes()).hexdigest() != self.source_sha:
            raise ValueError("runtime_manifest_changed")

    def record_physics_step(self, *, native_qd, same_step_fd):
        self._require_owner()
        if self.owner.state != "idle":
            raise ValueError("owner_not_idle")
        return self.capture.record_physics_step(native_qd=native_qd, same_step_fd=same_step_fd)

    def record_postcapture_step(self, sample, *, native_qd, same_step_fd):
        """Record only the refresh steps of this owner's pending capture."""
        self._require_owner()
        if sample is None or self.owner.state != "authenticating":
            raise ValueError("owned_postcapture_refresh_required")
        if self.owner.pending is not sample or self.prepare.pending is not sample:
            raise ValueError("postcapture_sample_identity_mismatch")
        body = sample["certificate"]["body"]
        step, _ = self.capture.epoch()
        if body["run_id"] != self.issuer.run_id or not body["capture_physics_step"] < step <= body["capture_physics_step"] + 16:
            raise ValueError("postcapture_step_outside_owned_window")
        return self.capture.record_physics_step(native_qd=native_qd, same_step_fd=same_step_fd)

    def observe(self, frame):
        self._require_owner(verify_manifest=True)
        return self.owner.observe(frame)

    def close(self):
        if threading.get_ident() != self.thread_id:
            raise ValueError("runtime_owner_thread_required")
        if self.closed:
            return
        if self.owner.state not in ("idle", "failed"):
            raise ValueError("runtime_owner_busy")
        self.closed = True
        errors = []
        for name in ("decoder", "ros", "priority"):
            resource = getattr(self, name)
            try:
                resource.close()
                child = getattr(resource, "child", None)
                if child is not None and child.returncode != 0:
                    errors.append(name + ":child_exit=" + str(child.returncode))
            except Exception as error:
                errors.append(name + ":" + str(error))
        # Keep owner.pending and failed state intact for external diagnosis.
        if errors:
            raise RuntimeError("runtime_close_failed:" + ";".join(errors))

    def __enter__(self):
        self._require_owner(verify_manifest=True)
        return self

    def __exit__(self, kind, error, traceback):
        try:
            self.close()
        except Exception as close_error:
            if error is None:
                raise
            error.add_note(str(close_error))
        return False


def build_observation_owner(*, root, out, run_id, clock_epoch, callbacks, settings=None):
    if not isinstance(callbacks, RuntimeCallbacks):
        raise ValueError("explicit_runtime_callbacks_required")
    source = bundle_source_sha(root)
    root = Path(root).resolve(strict=True)
    settings = settings or RuntimeSettings.from_env()
    paths, pins = settings.decoder()
    settings.verify_models(paths, pins)
    settings.ros()
    schema = MixedCameraSchema.from_file(root / "configs/mixed_camera_schema.json")
    initial_epoch = callbacks.read_epoch()
    step, sim_time = initial_epoch
    if type(step) is not int or step < 0 or not math.isfinite(sim_time) or abs(sim_time - step / 960) > 1 / (4 * 960):
        raise ValueError("invalid_runtime_initial_epoch")
    joints = ["arm_joint" + str(index) for index in range(1, 6)]
    issuer = RendererEpochIssuer(run_id=run_id, clock_epoch=clock_epoch, cameras=list(schema.config["cameras"]),
        joint_names=joints, physics_hz=960, model_sha256=hashlib.sha256(canonical(pins)).hexdigest(), source_sha256=source,
        fk=callbacks.fk, depth_camera_map={name: spec["rgb_camera"] for name, spec in schema.config["depth"].items()}, camera_schema=schema)
    capture = NativeMissionCapture(issuer=issuer, rgb=callbacks.rgb, depth=callbacks.depth,
        rgb_reference=callbacks.rgb_reference, depth_reference=callbacks.depth_reference, read_epoch=callbacks.read_epoch,
        read_joint_positions=callbacks.read_joint_positions, arm_indices=callbacks.arm_indices, joint_names=joints,
        render=callbacks.render, clock=callbacks.clock)
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    resources = []
    try:
        priority = PriorityFeedbackBridge(root, out, run_id, source, settings=settings)
        resources.append(priority)
        ros = LivePublicationBridge(root, out, run_id, schema, source_sha=source, settings=settings)
        resources.append(ros)
        decoder = LiveDecodeBridge(root, out, run_id, schema, issuer, capture.read_current_feedback, settings=settings)
        resources.append(decoder)
        if callbacks.read_epoch() != initial_epoch:
            raise ValueError("runtime_epoch_advanced_during_setup")
        prepare = RuntimePublicationPrepare(issuer=issuer, initial_step=step, refresh_packet=callbacks.refresh_packet,
            ros=ros, priority=priority, clock=callbacks.clock)
        return ObservationSession(root=root, source_sha=source, issuer=issuer, capture=capture,
            prepare=prepare, decoder=decoder, ros=ros, priority=priority)
    except Exception as error:
        for resource in reversed(resources):
            try:
                resource.close()
            except Exception as close_error:
                error.add_note("runtime_setup_cleanup:" + str(close_error))
        raise
