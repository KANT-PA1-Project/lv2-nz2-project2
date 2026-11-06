"""CPU checks only: child launches are mocked; the -O test rejects before GPU imports."""
import ast
import contextlib
import hashlib
import importlib
import json
import os
import shutil
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

sys.dont_write_bytecode = True
EXPERIMENT = Path(__file__).resolve().parents[1]
CORE = EXPERIMENT.parents[1]
sys.path.insert(0, str(EXPERIMENT))
from assemble_bundle import assemble


class V136Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        (EXPERIMENT / ".runtime").mkdir(exist_ok=True)
        cls.temporary = tempfile.TemporaryDirectory(dir=EXPERIMENT / ".runtime")
        cls.area = Path(cls.temporary.name)
        cls.bundle = cls.area / "bundle"
        cls.summary = assemble(cls.bundle)
        sys.path.insert(0, str(cls.bundle / "src"))
        cls.settings_type = importlib.import_module("arm_practice.native_rgbd.runtime_settings").RuntimeSettings

    @classmethod
    def tearDownClass(cls):
        sys.path.remove(str(cls.bundle / "src"))
        cls.temporary.cleanup()

    def setUp(self):
        self.scratch = tempfile.TemporaryDirectory(dir=self.area)
        self.addCleanup(self.scratch.cleanup)
        self.root = Path(self.scratch.name)
        models = self.root / "models"
        pins = {}
        for name in ("inference.json", "yolo/weights/best.pt", "centerpose/best.pt"):
            path = models / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(name.encode())
            pins[name] = hashlib.sha256(path.read_bytes()).hexdigest()
        manifest = self.root / "pins.json"
        manifest.write_text(json.dumps({"model_sha256": pins}))
        deps = self.root / "deps"
        scripts = self.root / "scripts"
        deps.mkdir(); scripts.mkdir()
        (scripts / "train_centerpose.py").write_text("")
        warmup = self.root / "warmup.npz"
        warmup.write_bytes(b"preflight only")
        python = self.root / "venv/bin/python"
        python.parent.mkdir(parents=True)
        python.symlink_to(sys.executable)
        setup = self.root / "ROS setup.bash"
        setup.write_text("")
        self.settings = self.settings_type({
            "decoder_python": str(python), "model_root": str(models),
            "model_manifest": str(manifest), "decoder_deps": str(deps),
            "decoder_scripts": str(scripts), "warmup_npz": str(warmup),
            "ros_python": str(python), "ros_setup": str(setup), "ros_domain_id": 87})

    def test_bundle_preserves_core_bytes_and_records_all_hashes(self):
        manifest = json.loads((self.bundle / "native_entrypoint_manifest.json").read_text())
        for target, entry in manifest["files"].items():
            digest = hashlib.sha256((self.bundle / target).read_bytes()).hexdigest()
            self.assertEqual(digest, entry["sha256"], target)
            if target.endswith(".py"):
                ast.parse((self.bundle / target).read_text())
            if entry["origin"] in ("shared_core", "shared_asset"):
                self.assertEqual(digest, hashlib.sha256((CORE / target).read_bytes()).hexdigest(), target)
        self.assertEqual(self.summary["source_sha256"], hashlib.sha256((self.bundle / "native_entrypoint_manifest.json").read_bytes()).hexdigest())
        self.assertFalse(manifest["physics_runtime_included"])

    def test_bundle_collision_fails_before_writing(self):
        core = self.root / "core"
        shared = core / "src/arm_practice"
        overlay = self.root / "experiment/overlay/arm_practice"
        shared.mkdir(parents=True); overlay.mkdir(parents=True)
        (shared / "same.py").write_text("core = True")
        (overlay / "same.py").write_text("overlay = True")
        output = self.root / "result"
        with self.assertRaisesRegex(ValueError, "bundle_file_collision"):
            assemble(output, core_root=core, experiment_root=overlay.parents[1])
        self.assertFalse(output.exists())
        self.assertEqual((shared / "same.py").read_text(), "core = True")

    def test_existing_bundle_is_not_overwritten(self):
        with self.assertRaises(FileExistsError):
            assemble(self.bundle)

    def test_lfs_pointer_fails_before_writing(self):
        experiment = self.root / "experiment"
        (experiment / "overlay").mkdir(parents=True)
        shutil.copytree(EXPERIMENT / "reference_configs", experiment / "reference_configs")
        for name in ("native_ros_publisher.py", "priority_feedback_consumer.py"):
            shutil.copyfile(EXPERIMENT / name, experiment / name)
        (experiment / "native_live_decoder.py").write_text("version https://git-lfs.github.com/spec/v1\n")
        output = experiment / ".runtime/result"
        with self.assertRaisesRegex(ValueError, "unresolved_LFS_pointer"):
            assemble(output, experiment_root=experiment)
        self.assertFalse(output.exists())

    def test_bundle_modules_and_camera_schema(self):
        prefix = "arm_practice.native_rgbd."
        for module in sorted((EXPERIMENT / "overlay/arm_practice/native_rgbd").glob("*.py")):
            importlib.import_module(prefix + module.stem)
        schema_type = importlib.import_module(prefix + "mixed_camera_schema").MixedCameraSchema
        schema = schema_type.from_file(self.bundle / "configs/mixed_camera_schema.json")
        self.assertEqual(set(schema.config["cameras"]), set(json.loads((CORE / "configs/mixed_camera_schema.json").read_text())["cameras"]))
        fk_type = importlib.import_module(prefix + "capture_fk").CaptureFK
        fk_type(self.bundle)

    def test_relative_venv_interpreter_is_not_resolved_to_system_python(self):
        config = self.root / "runtime.json"
        config.write_text(json.dumps({"decoder_python": "venv/bin/python"}))
        settings = self.settings_type.from_env({"R12_RUNTIME_CONFIG": str(config)})
        self.assertEqual(settings.path("decoder_python", executable=True), self.root / "venv/bin/python")
        self.assertNotEqual(settings.path("decoder_python", executable=True), Path(sys.executable).resolve())

    def test_child_receives_same_settings_without_parent_environment(self):
        child = self.settings_type.from_env(self.settings.child_env({}))
        self.assertEqual(child.values, self.settings.values)
        self.assertEqual(child.ros()[0], 87)

    def test_ros_domain_must_be_explicit_and_valid(self):
        for value in (None, True, -1, 233, "abc"):
            with self.subTest(value=value):
                values = dict(self.settings.values, ros_domain_id=value)
                with self.assertRaises(ValueError):
                    self.settings_type(values).ros()

    def test_repository_manifest_pin_format(self):
        _, expected = self.settings.decoder()
        manifest = {"models": [{"path": "models/perception/" + k, "sha256": v} for k, v in expected.items()]}
        Path(self.settings.values["model_manifest"]).write_text(json.dumps(manifest))
        paths, actual = self.settings.decoder()
        self.assertEqual(actual, expected)
        self.settings.verify_models(paths, actual)

    def test_model_path_escape_is_rejected(self):
        manifest = Path(self.settings.values["model_manifest"])
        data = json.loads(manifest.read_text())
        data["model_sha256"]["../outside"] = "a" * 64
        manifest.write_text(json.dumps(data))
        with self.assertRaisesRegex(ValueError, "model_path_escape"):
            self.settings.decoder()

    def test_optimized_decoder_still_rejects_changed_model_before_importing_gpu(self):
        (Path(self.settings.values["model_root"]) / "inference.json").write_text("changed")
        env = self.settings.child_env()
        env["PYTHONDONTWRITEBYTECODE"] = "1"
        result = subprocess.run([sys.executable, "-O", str(self.bundle / "native_live_decoder.py")], env=env, capture_output=True, text=True, timeout=15)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("perception_model_hash_mismatch:inference.json", result.stderr)
        self.assertNotIn("ModuleNotFoundError", result.stderr)

    def start_mock_bridge(self, module_name, class_name, hello, *args, **kwargs):
        module = importlib.import_module("arm_practice.native_rgbd." + module_name)
        stack = self.enterContext(contextlib.ExitStack())
        listener = MagicMock()
        connection = MagicMock()
        listener.accept.return_value = (connection, None)
        child = MagicMock(pid=1234, returncode=0)
        child.poll.return_value = 0
        stack.enter_context(patch.object(module.socket, "socket", return_value=listener))
        stack.enter_context(patch.object(module.os, "chmod"))
        popen = stack.enter_context(patch.object(module.subprocess, "Popen", return_value=child))
        stack.enter_context(patch.object(module.atexit, "register"))
        stack.enter_context(patch.object(module, "peer_credentials", return_value=(1234, os.getuid(), 0)))
        stack.enter_context(patch.object(module, "require_peer"))
        stack.enter_context(patch.object(module, "receive_packet", return_value=(hello, [])))
        stack.enter_context(patch.object(module, "send_packet"))
        if hasattr(module, "secrets"):
            stack.enter_context(patch.object(module.secrets, "token_hex", return_value="stream"))
        bridge = getattr(module, class_name)(self.bundle, self.root, "test-run", *args, settings=self.settings, **kwargs)
        self.addCleanup(bridge.close)
        return bridge, popen

    def test_decoder_bridge_uses_configured_python(self):
        _, popen = self.start_mock_bridge("live_decode_bridge", "LiveDecodeBridge", {"kind": "ready", "run_id": "test-run", "gpu_free_MiB": 2048}, None, None, lambda: None)
        self.assertEqual(popen.call_args.args[0][0], self.settings.values["decoder_python"])
        self.assertEqual(json.loads(popen.call_args.kwargs["env"]["R12_RUNTIME_VALUES"]), self.settings.values)

    def test_priority_bridge_uses_configured_python(self):
        _, popen = self.start_mock_bridge("priority_feedback_bridge", "PriorityFeedbackBridge", {"kind": "priority_ready", "run_id": "test-run", "stream_id": "stream"}, "a" * 64)
        self.assertEqual(popen.call_args.args[0][0], self.settings.values["ros_python"])

    def test_ros_bridge_passes_parent_source_and_quoted_setup(self):
        bundle_sha = self.summary["source_sha256"]
        hello = {"kind": "ready", "run_id": "test-run", "domain": 87, "source_sha256": "a" * 64, "bundle_manifest_sha256": bundle_sha}
        bridge, popen = self.start_mock_bridge("live_publication_bridge", "LivePublicationBridge", hello, None, source_sha="a" * 64)
        command = popen.call_args.args[0]
        self.assertIn('source "$1" && shift && exec "$@"', command)
        self.assertEqual(command[command.index("--source-sha") + 1], "a" * 64)
        self.assertIn(self.settings.values["ros_python"], command)
        self.assertIn(self.settings.values["ros_setup"], command)
        self.assertEqual(popen.call_args.kwargs["env"]["ROS_AUTOMATIC_DISCOVERY_RANGE"], "LOCALHOST")
        with self.assertRaisesRegex(ValueError, "ROS_feedback_source_changed"):
            bridge.publish_current({}, "b" * 64, 1)

    def test_missing_decoder_setting_fails_before_socket_or_child(self):
        module = importlib.import_module("arm_practice.native_rgbd.live_decode_bridge")
        with patch.object(module.socket, "socket") as socket, patch.object(module.subprocess, "Popen") as popen:
            with self.assertRaisesRegex(ValueError, "missing_runtime_setting"):
                module.LiveDecodeBridge(self.bundle, self.root, "run", None, None, None, settings=self.settings_type({}))
            socket.assert_not_called()
            popen.assert_not_called()


    def owner_components(self, *, advance_epoch=False):
        from arm_practice.native_rgbd.ros_current_contract import CurrentROSGuard
        import numpy as np
        module = importlib.import_module("arm_practice.native_rgbd.runtime_owner")
        schema = importlib.import_module("arm_practice.native_rgbd.mixed_camera_schema").MixedCameraSchema.from_file(self.bundle / "configs/mixed_camera_schema.json")
        epoch = {"step": 0}
        events = []
        resources = []
        annotator = lambda: SimpleNamespace(get_data=lambda *args, **kwargs: None)
        callbacks = module.RuntimeCallbacks(
            rgb={name: annotator() for name in schema.config["cameras"]},
            depth={name: annotator() for name in schema.config["depth"]},
            rgb_reference={name: annotator() for name in schema.config["cameras"]},
            depth_reference={name: annotator() for name in schema.config["depth"]},
            read_epoch=lambda: (epoch["step"], epoch["step"] / 960), read_joint_positions=lambda: np.zeros(5),
            arm_indices=(0, 1, 2, 3, 4), render=lambda: events.append("render"),
            fk=lambda joints: {name: np.eye(4) for name in schema.config["cameras"]},
            refresh_packet=lambda sample: events.append("refresh") or {"current_feedback": self.feedback()})

        class Endpoint:
            def __init__(self):
                self.child = SimpleNamespace(returncode=0)
                self.closed = False
                resources.append(self)
            def close(self):
                self.closed = True

        class Priority(Endpoint):
            def __init__(self, root, out, run, source, *, settings):
                super().__init__(); self.source = source
            def send(self, feedback):
                events.append("priority")

        class ROS(Endpoint):
            def __init__(self, root, out, run, schema, *, source_sha, settings):
                super().__init__(); self.source_sha = source_sha; self.run_id = run
                self.guard = CurrentROSGuard(run, source_sha)
                self.current = []
            def publish_current(self, feedback, source_sha, sequence):
                self.guard.validate({"kind": "current_feedback", "identity": {"run_id": self.run_id,
                    "source_sha256": source_sha, "stream_id": "ros-current-" + self.run_id},
                    "sequence": sequence, "feedback": feedback}, time.monotonic_ns())
                self.current.append(source_sha); events.append("current")
            def publish(self, *args, **kwargs):
                events.append("publish")
                return {"all_received": True}

        class Decoder(Endpoint):
            def __init__(self, root, out, run, schema, issuer, read_feedback, *, settings):
                super().__init__(); self.issuer = issuer; self.read_feedback = read_feedback
                if advance_epoch:
                    epoch["step"] += 1
            def send(self, *args):
                events.append("send")
            def verify_before_publication(self):
                events.append("verify")
            def collect(self, frame):
                events.append("collect")
                return {"frame": frame, "live_shadow": {"rejected": "CPU_test_rejection", "adapter_ready": False, "motion_authorized": False}}

        stack = self.enterContext(contextlib.ExitStack())
        for name, value in (("PriorityFeedbackBridge", Priority), ("LivePublicationBridge", ROS), ("LiveDecodeBridge", Decoder)):
            stack.enter_context(patch.object(module, name, value))
        self.settings.values["ros_domain_id"] = 33
        return module, callbacks, resources, events

    @staticmethod
    def feedback():
        now = time.monotonic_ns()
        return {"position": {"physics_step": 16, "q": [0.] * 5, "read_start_ns": now, "read_end_ns": now},
                "velocity": {"step": 16, "q": [0.] * 5, "velocity": [0.] * 5, "velocity_valid": True,
                             "read_start_ns": now, "read_end_ns": now}}

    def make_owner(self):
        module, callbacks, resources, events = self.owner_components()
        session = module.build_observation_owner(root=self.bundle, out=self.root / "owner", run_id="owner_test",
            clock_epoch="cpu_clock", callbacks=callbacks, settings=self.settings)
        self.addCleanup(session.close)
        return session, callbacks, resources, events

    def mock_capture_plan(self, session):
        # Only source plumbing/owner transitions are under test here. Authentication
        # and native packet semantics are not claimed by these synthetic inputs.
        sample = {"captured": True, "certificate": {"body": {"cpu_test": True}}, "pixels": {}, "depths": {}}
        self.enterContext(patch.object(session.capture, "acquire", return_value=sample))
        self.enterContext(patch.object(session.issuer, "verify", return_value={"rgbd_capture_association_verified": True}))
        def plan(**kwargs):
            session.prepare.adapter.verify_capture(kwargs["certificate"])
            return {"CPU_test_plan": True}
        self.enterContext(patch.object(session.prepare.adapter, "prepare", side_effect=plan))
        return sample

    def test_pa33_example_and_local_domains(self):
        self.assertEqual(json.loads((EXPERIMENT / "runtime.example.json").read_text())["ros_domain_id"], 33)
        local = EXPERIMENT / "runtime.local.json"
        if local.exists():
            self.assertEqual(json.loads(local.read_text())["ros_domain_id"], 33)

    def test_owner_uses_one_bundle_hash_for_all_source_identities(self):
        session, _, resources, events = self.make_owner()
        self.assertEqual(session.source_sha, self.summary["source_sha256"])
        self.assertEqual(session.issuer.source, session.source_sha)
        self.assertEqual(session.ros.source_sha, session.source_sha)
        self.assertEqual(session.priority.source, session.source_sha)
        self.assertEqual(session.prepare.source_sha, session.source_sha)
        self.assertIs(session.owner.capture, session.capture)
        self.assertIs(session.owner.prepare, session.prepare)
        self.assertEqual(events, [])
        self.assertEqual(len(resources), 3)

    def test_first_feedback_source_plumbing_with_mock_capture_plan(self):
        session, _, _, events = self.make_owner()
        self.mock_capture_plan(session)
        result = session.observe(0)
        self.assertEqual(session.ros.current, [self.summary["source_sha256"]])
        self.assertEqual(events, ["refresh", "priority", "current", "send", "verify", "publish", "collect"])
        self.assertFalse(result["motion_authorized"])
        self.assertEqual(result["decoded"]["live_shadow"]["rejected"], "CPU_test_rejection")
        self.assertEqual(session.owner.state, "idle")
        self.assertIsNone(session.owner.pending)

    def test_prepare_refuses_source_mismatch_before_refresh(self):
        module = importlib.import_module("arm_practice.native_rgbd.runtime_prepare")
        with self.assertRaisesRegex(ValueError, "runtime_source_identity_mismatch"):
            module.RuntimePublicationPrepare(issuer=SimpleNamespace(source="a" * 64), initial_step=0,
                refresh_packet=lambda sample: self.fail("refresh must not run"),
                ros=SimpleNamespace(source_sha="b" * 64), priority=SimpleNamespace(source="a" * 64))

    def test_prepare_refuses_changed_source_before_refresh(self):
        session, _, _, events = self.make_owner()
        session.priority.source = "b" * 64
        with self.assertRaisesRegex(ValueError, "runtime_source_identity_changed"):
            session.prepare({})
        self.assertEqual(events, [])
        self.assertIsNone(session.prepare.pending)

    def test_owner_transport_failure_retains_sample_and_blocks_recapture(self):
        session, _, _, _ = self.make_owner()
        sample = self.mock_capture_plan(session)
        with patch.object(session.ros, "publish", side_effect=RuntimeError("CPU_transport_failure")):
            with self.assertRaisesRegex(RuntimeError, "CPU_transport_failure"):
                session.observe(0)
        self.assertEqual(session.owner.state, "failed")
        self.assertIs(session.owner.pending, sample)
        with self.assertRaisesRegex(ValueError, "owner_not_idle"):
            session.observe(1)
        session.close()
        self.assertIs(session.owner.pending, sample)

    def test_owner_setup_epoch_change_closes_created_bridges(self):
        module, callbacks, resources, _ = self.owner_components(advance_epoch=True)
        with self.assertRaisesRegex(ValueError, "runtime_epoch_advanced_during_setup"):
            module.build_observation_owner(root=self.bundle, out=self.root / "owner", run_id="owner_test",
                clock_epoch="cpu_clock", callbacks=callbacks, settings=self.settings)
        self.assertEqual(len(resources), 3)
        self.assertTrue(all(resource.closed for resource in resources))

    def test_owner_refuses_another_bundle_root(self):
        module, callbacks, resources, _ = self.owner_components()
        with self.assertRaisesRegex(ValueError, "runtime_loaded_outside_bundle"):
            module.build_observation_owner(root=self.root, out=self.root / "owner", run_id="owner_test",
                clock_epoch="cpu_clock", callbacks=callbacks, settings=self.settings)
        self.assertEqual(resources, [])

    def test_owner_requires_callers_callbacks(self):
        module, _, resources, _ = self.owner_components()
        with self.assertRaisesRegex(ValueError, "explicit_runtime_callbacks_required"):
            module.build_observation_owner(root=self.bundle, out=self.root / "owner", run_id="owner_test",
                clock_epoch="cpu_clock", callbacks=None, settings=self.settings)
        self.assertEqual(resources, [])

    def test_owner_refuses_calls_from_another_thread(self):
        session, _, _, events = self.make_owner()
        errors = []
        def wrong_thread():
            try:
                session.observe(0)
            except Exception as error:
                errors.append(str(error))
        thread = threading.Thread(target=wrong_thread)
        thread.start(); thread.join(timeout=2)
        self.assertFalse(thread.is_alive())
        self.assertEqual(errors, ["runtime_owner_thread_required"])
        self.assertEqual(events, [])

    def test_owner_close_reports_nonzero_child_exit(self):
        session, _, resources, _ = self.make_owner()
        session.decoder.child.returncode = 7
        with self.assertRaisesRegex(RuntimeError, "decoder:child_exit=7"):
            session.close()
        self.assertTrue(all(resource.closed for resource in resources))

    def test_owner_forwards_same_step_velocities_to_core_capture(self):
        session, _, _, events = self.make_owner()
        session.record_physics_step(native_qd=[0.] * 5, same_step_fd=[0.] * 5)
        self.assertEqual(session.issuer.history[0]["physics_step"], 0)
        self.assertEqual(session.issuer.history[0]["encoder_q"], [0.] * 5)
        self.assertEqual(events, [])


    def test_owner_refuses_changed_manifest_before_capture(self):
        session, _, _, events = self.make_owner()
        path = self.bundle / "native_entrypoint_manifest.json"
        original = path.read_bytes()
        try:
            path.write_bytes(original + b" ")
            with self.assertRaisesRegex(ValueError, "runtime_manifest_changed"):
                session.observe(0)
        finally:
            path.write_bytes(original)
        self.assertEqual(events, [])


    def test_ros_bridge_persists_exact_ack_timestamps_before_return(self):
        import time
        schema = SimpleNamespace(wire_schema="CPU_mock_wire")
        hello = {"kind": "ready", "run_id": "test-run", "domain": 87, "source_sha256": "a" * 64,
                 "bundle_manifest_sha256": self.summary["source_sha256"]}
        bridge, _ = self.start_mock_bridge("live_publication_bridge", "LivePublicationBridge", hello, schema, source_sha="a" * 64)
        module = importlib.import_module("arm_practice.native_rgbd.live_publication_bridge")
        metrics = importlib.import_module("arm_practice.native_rgbd.ack_metrics")
        start = time.monotonic_ns() - 20_000_000
        certificate = {"body": {"run_id": "test-run", "source_sha256": "a" * 64, "clock_epoch": "CPU_clock",
            "capture_physics_step": 9600, "physics_hz": 960, "capture_monotonic_lower_bound_ns": start}}
        response = {"kind": "publication_result", "all_received": True, "expected": 31, "received": 31,
                    "errors": [], "domain": 87, "capture_stamp_ns": 10_000_000_000}
        with patch.object(module, "receive_packet", return_value=(response, [])):
            result = bridge.publish(certificate, {}, {}, {}, {"rgbd_capture_association_verified": True})
        journal = self.root / "producer_ack.jsonl"
        record = json.loads(journal.read_text())
        self.assertEqual(record["capture_monotonic_ns"], start)
        self.assertEqual(record["producer_transport_timing"]["ack_receive_end_ns"], result["producer_transport_timing"]["ack_receive_end_ns"])
        report = metrics.analyze(journal)
        self.assertTrue(report["valid"], report["errors"])
        self.assertEqual(report["summary"]["p50"], (result["producer_transport_timing"]["ack_receive_end_ns"] - start) / 1e6)
        self.assertEqual(record["clock_domain"], "same-host-monotonic-ns")

    def test_ros_bridge_refuses_ack_from_different_capture(self):
        schema = SimpleNamespace(wire_schema="CPU_mock_wire")
        hello = {"kind": "ready", "run_id": "test-run", "domain": 87, "source_sha256": "a" * 64,
                 "bundle_manifest_sha256": self.summary["source_sha256"]}
        bridge, _ = self.start_mock_bridge("live_publication_bridge", "LivePublicationBridge", hello, schema, source_sha="a" * 64)
        module = importlib.import_module("arm_practice.native_rgbd.live_publication_bridge")
        certificate = {"body": {"run_id": "test-run", "source_sha256": "a" * 64, "clock_epoch": "CPU_clock",
            "capture_physics_step": 9600, "physics_hz": 960, "capture_monotonic_lower_bound_ns": time.monotonic_ns()}}
        response = {"kind": "publication_result", "all_received": True, "expected": 31, "received": 31,
                    "errors": [], "domain": 87, "capture_stamp_ns": 10_000_000_001}
        with patch.object(module, "receive_packet", return_value=(response, [])):
            with self.assertRaisesRegex(ValueError, "ROS_ACK_capture_or_domain_mismatch"):
                bridge.publish(certificate, {}, {}, {}, {"rgbd_capture_association_verified": True})
        self.assertFalse((self.root / "producer_ack.jsonl").exists())


    def test_owner_postcapture_record_requires_identity_and_bounded_window(self):
        session, _, _, _ = self.make_owner()
        sample = {"certificate": {"body": {"run_id": "owner_test", "capture_physics_step": 8}}}
        with self.assertRaisesRegex(ValueError, "owned_postcapture_refresh_required"):
            session.record_postcapture_step(sample, native_qd=[0.] * 5, same_step_fd=[0.] * 5)
        session.owner.state = "authenticating"
        session.owner.pending = session.prepare.pending = sample
        try:
            with patch.object(session.capture, "epoch", return_value=(24, 24 / 960)), \
                 patch.object(session.capture, "record_physics_step") as record:
                session.record_postcapture_step(sample, native_qd=[0.] * 5, same_step_fd=[0.] * 5)
                record.assert_called_once()
                with self.assertRaisesRegex(ValueError, "sample_identity_mismatch"):
                    session.record_postcapture_step(dict(sample), native_qd=[0.] * 5, same_step_fd=[0.] * 5)
            for step in (8, 25):
                with patch.object(session.capture, "epoch", return_value=(step, step / 960)):
                    with self.assertRaisesRegex(ValueError, "outside_owned_window"):
                        session.record_postcapture_step(sample, native_qd=[0.] * 5, same_step_fd=[0.] * 5)
        finally:
            session.owner.state = "idle"
            session.owner.pending = session.prepare.pending = None

    def test_postcapture_accepts_exact_float32_imu_phase_at_longer_uptime(self):
        module = importlib.import_module("arm_practice.native_rgbd.postcapture_feedback")
        timeline = importlib.import_module("arm_practice.native_rgbd.packet_timeline")
        state = {"step": 7504}
        now = time.monotonic_ns()
        def stepper(sub):
            state["step"] += 1
        def encoder():
            return dict(step=state["step"], read_start_ns=time.monotonic_ns(),
                        velocity_valid=True, velocity=[0.] * 5)
        def imus():
            sim = state["step"] * timeline.f32(1 / 960)
            sample = dict(physics_step=state["step"], physics_time_s=sim,
                          time=timeline.f32(sim - 1 / 960), valid=True,
                          gyro=[0.] * 3, accel=[0., 0., 9.81])
            return [dict(sample), dict(sample)]
        result = module.PostCaptureFeedback(capture_step=7504,
            capture_lower_bound_ns=now - 1, capture_completed_ns=now, physics_hz=960).run(
                stepper=stepper, read_step=lambda: state["step"],
                sample_encoder=encoder, read_imus=imus)
        self.assertTrue(result["imu_distinct_120hz_valid"])
        self.assertGreater(abs(result["imu_deltas_s"][0] - 1 / 120), 1e-7)

    def test_postcapture_still_rejects_imu_from_another_physics_step(self):
        module = importlib.import_module("arm_practice.native_rgbd.postcapture_feedback")
        timeline = importlib.import_module("arm_practice.native_rgbd.packet_timeline")
        state = {"step": 7504}
        now = time.monotonic_ns()
        def stepper(sub):
            state["step"] += 1
        def encoder():
            return dict(step=state["step"], read_start_ns=time.monotonic_ns(),
                        velocity_valid=True, velocity=[0.] * 5)
        def imus():
            step = state["step"] - 8
            sim = step * timeline.f32(1 / 960)
            return [dict(physics_step=step, physics_time_s=sim,
                time=timeline.f32(sim - 1 / 960), valid=True,
                gyro=[0.] * 3, accel=[0., 0., 9.81]) for _ in range(2)]
        with self.assertRaisesRegex(ValueError, "imu_encoder_epoch_mismatch"):
            module.PostCaptureFeedback(capture_step=7504, capture_lower_bound_ns=now - 1,
                capture_completed_ns=now, physics_hz=960).run(
                    stepper=stepper, read_step=lambda: state["step"],
                    sample_encoder=encoder, read_imus=imus)

    def make_isaac_adapter(self, encoder_prime_steps=0):
        """CPU doubles only: patch the factory before constructing the adapter."""
        import numpy as np
        module = importlib.import_module("arm_practice.native_rgbd.isaac_runtime_adapter")
        timeline = importlib.import_module("arm_practice.native_rgbd.packet_timeline")
        world = SimpleNamespace(current_time_step_index=0, current_time=0., render=MagicMock())
        robot = SimpleNamespace(get_joint_positions=lambda: np.zeros(6),
                                get_joint_velocities=lambda: np.zeros(6))

        def stepper(sub):
            world.current_time_step_index += 1
            world.current_time = world.current_time_step_index * timeline.f32(1 / 960)

        def reading(*args, **kwargs):
            return SimpleNamespace(time=timeline.f32(world.current_time - 1 / 960), is_valid=True,
                ang_vel_x=0., ang_vel_y=0., ang_vel_z=0., lin_acc_x=0., lin_acc_y=0., lin_acc_z=9.81)

        imus = [SimpleNamespace(prim_path="/imu" + str(i),
                 _imu_sensor_interface=SimpleNamespace(get_sensor_reading=reading)) for i in range(2)]
        schema = json.loads((self.bundle / "configs/mixed_camera_schema.json").read_text())
        annotators = lambda names: {name: SimpleNamespace(get_data=MagicMock()) for name in names}
        source = self.root / "parent.py"
        source.write_text("# CPU parent double\n")
        session = MagicMock(source_sha="a" * 64)
        session.owner.state = "idle"
        session.issuer.run_id = "cpu-isaac"
        factory = self.enterContext(patch.object(module, "build_observation_owner", return_value=session))
        adapter = module.IsaacRuntimeAdapter(root=self.bundle, out=self.root / "attachment",
            run_id="cpu-isaac", clock_epoch="cpu-clock", parent_source=source,
            world=world, robot=robot, joint_names=["wheel"] + list(module.ARM_JOINTS),
            rig=SimpleNamespace(transforms=MagicMock()), rgb=annotators(schema["cameras"]),
            depth=annotators(schema["depth"]), rgb_reference=annotators(schema["cameras"]),
            depth_reference=annotators(schema["depth"]), imus=imus, physics_stepper=stepper,
            encoder_prime_steps=encoder_prime_steps)
        self.addCleanup(adapter.close)
        return module, adapter, session, factory

    def coverage_payload(self):
        import numpy as np
        module = importlib.import_module("arm_practice.native_rgbd.coverage_transport")
        schema = importlib.import_module("arm_practice.native_rgbd.mixed_camera_schema").MixedCameraSchema.from_file(
            self.bundle/"configs/mixed_camera_schema.json")
        sample = dict(certificate=dict(body=dict(run_id="cpu-coverage",source_sha256="a"*64,
            capture_physics_step=960)), pixels={}, depths={})
        for name in schema.config["cameras"]:
            sample["pixels"][name] = np.zeros(schema.descriptor(name,"rgb")["shape"], dtype=np.uint8)
        for name in schema.config["depth"]:
            sample["depths"][name] = np.full(schema.descriptor(name,"depth")["shape"], .5, dtype=np.float32)
        payload = module.make_payload(sample,schema,lidar={name:[1.]*90 for name in module.LIDARS},
            lidar_step=976,read_start_ns=4_000_000_000,read_end_ns=4_000_010_000,sequence=1)
        return module,schema,payload

    def test_coverage_projection_and_control_identity(self):
        module,schema,payload = self.coverage_payload()
        module.validate(payload,run_id="cpu-coverage",source="a"*64,schema=schema,
                        sequence=0,seen=set(),now_ns=4_001_000_000)
        self.assertEqual(set(payload["clouds"]),set(schema.config["depth"]))
        self.assertTrue(all(len(cloud["xyz"]) == len(cloud["rgb"]) for cloud in payload["clouds"].values()))
        self.assertFalse(payload["motion_authorized"])

    def test_coverage_rejects_source_hash_replay_and_stale_lidar(self):
        import copy
        module,schema,payload = self.coverage_payload()
        def validate(row,seen=()):
            module.validate(row,run_id="cpu-coverage",source="a"*64,schema=schema,
                            sequence=0,seen=set(seen),now_ns=4_001_000_000)
        bad = copy.deepcopy(payload);bad["source_sha256"]="b"*64
        with self.assertRaisesRegex(ValueError,"source_or_run"):
            validate(bad)
        bad = copy.deepcopy(payload);bad["payload_sha256"]="0"*64
        with self.assertRaisesRegex(ValueError,"payload_hash"):
            validate(bad)
        with self.assertRaisesRegex(ValueError,"replayed_nonce"):
            validate(payload,[payload["nonce"]])
        bad = copy.deepcopy(payload);bad["lidar_read_start_ns"]-=200_000_000
        bad["lidar_read_end_ns"]-=200_000_000;bad=module.seal(bad)
        with self.assertRaisesRegex(ValueError,"stale_or_invalid"):
            validate(bad)

    def test_wire_view_preserves_exact_bytes_and_retains_sealed_owner(self):
        import numpy as np
        module = importlib.import_module("arm_practice.native_rgbd.immutable_blobs")
        source = np.arange(30, dtype=np.float32).reshape(5, 6).copy()
        expected = source.tobytes()
        source.flags.writeable = False
        view = module.immutable_blob(source)
        self.assertTrue(view.readonly)
        self.assertIs(view.obj, source)
        self.assertEqual(len(view), source.nbytes)
        self.assertEqual(bytes(view), expected)
        with self.assertRaises(TypeError):
            view[0] = 0

    def test_wire_view_rejects_mutable_or_borrowed_arrays(self):
        import numpy as np
        module = importlib.import_module("arm_practice.native_rgbd.immutable_blobs")
        source = np.arange(30, dtype=np.uint8)
        with self.assertRaisesRegex(ValueError, "sealed_readonly"):
            module.immutable_blob(source)
        source.flags.writeable = False
        with self.assertRaisesRegex(ValueError, "owned_contiguous"):
            module.immutable_blob(source[::2])

    def test_isaac_actual_priming_history_precedes_packet_interval(self):
        _, adapter, session, _ = self.make_isaac_adapter(encoder_prime_steps=960)
        self.assertEqual(adapter.initial_epoch[0], 960)
        self.assertEqual(session.issuer.record_encoder_epoch.call_count, 960)
        rows = [call.kwargs for call in session.issuer.record_encoder_epoch.call_args_list]
        self.assertEqual([row["physics_step"] for row in rows], list(range(1, 961)))
        self.assertTrue(all(row["read_start_ns"] <= row["read_end_ns"] for row in rows))
        self.assertEqual(adapter.timeline.initial_step, 960)
        self.assertEqual(adapter.timeline.pending, [[], []])
        for sub in range(8):
            adapter.step(sub)
        self.assertEqual([s["physics_step"] for s in adapter.timeline.pending[0]], [968])
        saved = json.loads((adapter.out / "pre_session_encoder_history.json").read_text())
        self.assertEqual(len(saved["samples"]), 960)
        self.assertTrue(saved["packet_interval_starts_after_history"])

    def test_isaac_adapter_records_real_step_and_120hz_boundaries(self):
        _, adapter, session, factory = self.make_isaac_adapter()
        for sub in range(16):
            adapter.step(sub)
        self.assertEqual(session.record_physics_step.call_count, 16)
        self.assertEqual(adapter.arm, (1, 2, 3, 4, 5))
        self.assertEqual(adapter._previous_encoder["step"], 16)
        self.assertTrue(adapter._previous_encoder["velocity_valid"])
        self.assertEqual([s["physics_step"] for s in adapter.timeline.pending[0]], [8, 16])
        self.assertEqual(factory.call_count, 1)
        recorded = session.record_physics_step.call_args.kwargs
        self.assertEqual(recorded, dict(native_qd=[0.] * 5, same_step_fd=[0.] * 5))

    def test_isaac_control_quantization_matches_core_native_float32(self):
        import numpy as np
        _, adapter, _, _ = self.make_isaac_adapter()
        native = np.asarray([.11, .22, .33, .44, .55, .66], dtype=np.float32)
        adapter.robot.get_joint_positions = lambda: native.copy()
        for sub in range(8):
            adapter.step(sub)
        expected = np.rint(native[list(adapter.arm)] * 4096 / (2 * np.pi)) * 2 * np.pi / 4096
        self.assertEqual(adapter._previous_encoder["q"], expected.tolist())

    def test_isaac_adapter_same_epoch_does_not_refresh_encoder_timestamp(self):
        _, adapter, _, _ = self.make_isaac_adapter()
        first = adapter._sample_control_encoder()
        self.assertEqual(adapter._sample_control_encoder(), first)

    def test_isaac_adapter_external_step_is_terminal(self):
        _, adapter, session, _ = self.make_isaac_adapter()
        adapter.physics_stepper(0)
        with self.assertRaisesRegex(ValueError, "physics_advanced_outside"):
            adapter.step(0)
        with self.assertRaisesRegex(ValueError, "isaac_connection_failed"):
            adapter.step(0)
        session.record_physics_step.assert_not_called()

    def test_isaac_adapter_refresh_retains_precapture_imu_history(self):
        _, adapter, session, _ = self.make_isaac_adapter()
        for sub in range(16):
            adapter.step(sub)
        now = time.monotonic_ns()
        body = dict(run_id="cpu-isaac", physics_hz=960, capture_physics_step=16,
                    capture_monotonic_lower_bound_ns=now - 1, issued_ns=now)
        sample = {"certificate": {"body": body}}
        session.owner.state = "authenticating"
        session.owner.pending = session.prepare.pending = sample
        def current():
            start = time.monotonic_ns()
            return dict(q=[0.] * 5, physics_step=adapter.world.current_time_step_index,
                        read_start_ns=start, read_end_ns=time.monotonic_ns())
        session.capture.read_current_feedback.side_effect = current
        result = adapter.refresh_packet(sample)
        self.assertEqual(session.record_postcapture_step.call_count, 16)
        self.assertEqual(result["native_packet_timeline"]["current_physics_step"], 32)
        for stream in result["all_IMU_samples"]:
            self.assertEqual([s["physics_step"] for s in stream["native_samples"]], [8, 16, 24, 32])
        self.assertEqual(sample["certificate"]["body"], body)
        self.assertIsNone(adapter._refresh_sample)
        session.ros.publish_current.assert_not_called()
        session.priority.publish.assert_not_called()

    def test_isaac_adapter_refuses_unowned_refresh_and_wrong_boundary(self):
        _, adapter, session, _ = self.make_isaac_adapter()
        with self.assertRaisesRegex(ValueError, "refresh_requires_owned"):
            adapter.refresh_packet({})
        adapter.step(0)
        with self.assertRaisesRegex(ValueError, "120hz_sample_boundary"):
            adapter.observe(0)
        session.observe.assert_not_called()

    def test_isaac_adapter_observe_pins_parent_source_and_frame_order(self):
        _, adapter, session, _ = self.make_isaac_adapter()
        for sub in range(8):
            adapter.step(sub)
        adapter.observe(0)
        session.observe.assert_called_once_with(0)
        with self.assertRaisesRegex(ValueError, "frame_order"):
            adapter.observe(0)
        adapter.parent_source.write_text("# changed\n")
        with self.assertRaisesRegex(ValueError, "parent_runtime_source_changed"):
            adapter.observe(1)

    def test_isaac_adapter_close_once_and_reject_reuse(self):
        _, adapter, session, _ = self.make_isaac_adapter()
        adapter.close()
        adapter.close()
        session.close.assert_called_once()
        with self.assertRaisesRegex(ValueError, "isaac_connection_closed"):
            adapter.step(0)

    def test_isaac_connector_rejects_missing_bindings_and_legacy_before_attach(self):
        module = importlib.import_module("arm_practice.native_rgbd.isaac_runtime_adapter")
        arguments = dict(root=self.bundle, out=self.root / "unused", parent_source=__file__, clock_epoch="cpu")
        with patch.object(module, "IsaacRuntimeAdapter") as constructor:
            with self.assertRaisesRegex(ValueError, "binding_missing"):
                module.connect_sensor_runtime({}, **arguments)
            keys = ("world robot names prepared_camera_rig native_rgb native_depth native_rgb_refs "
                    "native_depth_refs imus apply_controls_and_step capture_run_id").split()
            namespace = dict.fromkeys(keys)
            namespace["perception_enabled"] = True
            with self.assertRaisesRegex(ValueError, "legacy_observer_paths"):
                module.connect_sensor_runtime(namespace, **arguments)
            constructor.assert_not_called()


if __name__ == "__main__":
    unittest.main()
