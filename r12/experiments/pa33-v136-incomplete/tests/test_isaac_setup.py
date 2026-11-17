"""CPU-only preparation checks: never imports Isaac or starts native children."""
import ast
import hashlib
import importlib
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

EXPERIMENT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(EXPERIMENT))
from prepare_isaac_validation import parent_parts, digest
from run_isaac_validation import fixed_environment, verify_plan


class IsaacSetupTests(unittest.TestCase):
    def test_parent_is_pinned_and_only_initialization_is_staged(self):
        prefix, control, provenance = parent_parts(EXPERIMENT.parents[1] / "scripts/r12_sensor_sim.py")
        ast.parse(prefix)
        ast.parse(control)
        self.assertNotIn("socket.create_connection(", prefix)
        self.assertNotIn("fixture=json.loads(", prefix)
        self.assertNotIn("subprocess.Popen(", prefix)
        self.assertNotIn("instance_id_segmentation_fast", prefix)
        self.assertIn("native_schema.check_actual_optics", prefix)
        self.assertIn("native_depth_refs", prefix)
        self.assertEqual(provenance["original_controller_lines"], [338, 349])
        original = (EXPERIMENT.parents[1] / "scripts/r12_sensor_sim.py").read_text().splitlines()
        extracted = "\n".join(line[2:] for line in original[337:349]) + "\n"
        self.assertEqual(hashlib.sha256(extracted.encode()).hexdigest(),
                         provenance["actuator_block_sha256"])

    def test_parent_changes_are_rejected(self):
        with tempfile.TemporaryDirectory(dir=EXPERIMENT / ".runtime") as scratch:
            parent = Path(scratch) / "parent.py"
            parent.write_text("# changed")
            with self.assertRaisesRegex(ValueError, "shared_parent_changed"):
                parent_parts(parent)

    def test_staged_original_stationary_step_calls_world_once(self):
        _, control, _ = parent_parts(EXPERIMENT.parents[1] / "scripts/r12_sensor_sim.py")
        robot = MagicMock()
        world = MagicMock()
        q, velocity = np.array([.1, .2]), np.zeros(2)
        namespace = dict(robot=robot, world=world, latest_mode=False, q=q, vel=velocity,
                         ArticulationAction=lambda **kwargs: kwargs)
        exec(compile(control, "<CPU original controller>", "exec"), namespace)
        namespace["apply_controls_and_step"](0)
        world.step.assert_called_once_with(render=False)
        action = robot.apply_action.call_args.args[0]
        self.assertIs(action["joint_positions"], q)
        self.assertIs(action["joint_velocities"], velocity)

    def test_environment_removes_inherited_motion_and_experiment_options(self):
        with patch.dict("os.environ", {"R12_MISSION": "1", "R12_SIM_COMMAND_DELAY_NS": "12",
                                     "FLEET_RUN_ID": "old", "CUDA_VISIBLE_DEVICES": "1"}):
            env = fixed_environment({"run_id": "cpu"}, Path("/tmp/cpu"))
        self.assertEqual(env["R12_MISSION"], "0")
        self.assertEqual(env["R12_MISSION_PERCEPTION"], "0")
        self.assertEqual(env["CUDA_VISIBLE_DEVICES"], "0")
        self.assertNotIn("R12_SIM_COMMAND_DELAY_NS", env)
        self.assertNotIn("FLEET_RUN_ID", env)

    def make_plan(self, area):
        bundle = area / "bundle"
        bundle.mkdir()
        manifest = bundle / "native_entrypoint_manifest.json"
        manifest.write_text(json.dumps({"files": {}}))
        source = area / "input.py"
        source.write_text("# pinned")
        plan = dict(schema="v136-isaac-validation-plan-v1",
            output=str(area / "run"), bundle=dict(root=str(bundle), source_sha256=digest(manifest)),
            inputs={"input.py": digest(source)}, asset_root=str(area), assets={},
            physics_hz=960, initial_record_steps=960, interval_steps=48,
            warmup_epochs=2, measurement_epochs=8, attempts=10, wall_budget_s=300,
            free_gpu_reserve_mib=1024, minimum_free_disk_mib=2048,
            motion_authorized=False, seed=0)
        path = area / "plan.json"
        path.write_text(json.dumps(plan))
        return path, plan

    def test_read_only_plan_verification_never_starts_processes(self):
        with tempfile.TemporaryDirectory(dir=EXPERIMENT / ".runtime") as scratch:
            path, _ = self.make_plan(Path(scratch))
            with patch("subprocess.Popen", side_effect=AssertionError("native forbidden")):
                self.assertEqual(verify_plan(path)["physics_hz"], 960)

    def test_changed_stage_and_protocol_are_rejected(self):
        with tempfile.TemporaryDirectory(dir=EXPERIMENT / ".runtime") as scratch:
            area = Path(scratch)
            path, plan = self.make_plan(area)
            (area / "input.py").write_text("# changed")
            with self.assertRaisesRegex(ValueError, "staged_input_changed"):
                verify_plan(path)
            (area / "input.py").write_text("# pinned")
            plan["measurement_epochs"] = 100
            path.write_text(json.dumps(plan))
            with self.assertRaisesRegex(ValueError, "protocol_changed"):
                verify_plan(path)

    def test_supervisor_cleans_owned_group_after_parent_exit(self):
        import run_isaac_validation as launcher
        process = MagicMock(pid=123456)
        process.poll.return_value = 1
        with patch.object(launcher.os, "killpg") as kill:
            launcher.terminate_owned(process)
        kill.assert_called_once_with(123456, launcher.signal.SIGTERM)

    def test_offline_report_keeps_fixed_measurement_phase(self):
        import verify_isaac_validation as verifier
        with tempfile.TemporaryDirectory(dir=EXPERIMENT / ".runtime") as scratch:
            area = Path(scratch)
            run = area / "run"
            run.mkdir()
            (run / "shutdown.json").write_text(json.dumps({"observation_bench_pass": True,
                "child_exit_codes": {"decoder": 0, "ros": 0, "priority": 0}}))
            (area / "supervisor_exit.json").write_text(json.dumps(dict(
                returncode=0, failure=None, sources_unchanged=True, plan_unchanged=True)))
            cases = ["invalid_frame", "unowned_capture", "source_mismatch", "cross_thread"]
            (run / "runtime_rejections.json").write_text(json.dumps(dict(
                epoch_unchanged=True, cases=[dict(case=n, rejected=True) for n in cases])))
            observations = [dict(frame=i, phase="warmup" if i < 2 else "measurement",
                result=dict(captured=True, ROS=dict(all_received=True, errors=[],
                    raw_bytes_hash_verified=12, capture_stamp_ns=i, expected=31, received=31),
                    decoded=dict(live_source_authenticated=True, raw_hashes_verified=12,
                                 camera_fresh_at_completion=True))) for i in range(10)]
            (run / "observations.json").write_text(json.dumps(observations))
            metric = SimpleNamespace(analyze=lambda path: dict(valid=True, errors=[],
                samples=[dict(producer_ACK_ms=i, capture_stamp_ns=i) for i in range(10)]),
                stats=lambda values: dict(samples=len(values)))
            plan = dict(output=str(run), run_id="CPU", asset_root=str(area))
            with patch("run_isaac_validation.verify_plan", return_value=plan), patch.dict(
                    sys.modules, {"arm_practice.native_rgbd.ack_metrics": metric}):
                result = verifier.report(area / "plan.json")
            self.assertTrue(result["observation_bench_pass"], result["errors"])
            self.assertEqual(result["ACK_measurement_only"]["samples"], 8)
            self.assertFalse(result["full_pa33_acceptance"])
            self.assertFalse(result["native_fault_injection_verified"])

    def test_ros_queue_holds_maximum_pending_samples_per_sensor(self):
        source = (EXPERIMENT / "native_ros_publisher.py").read_text()
        self.assertIn("QoSProfile(depth=256,", source)
        self.assertGreaterEqual(256, 240)

    def test_runtime_guard_probes_do_not_call_physics(self):
        module = importlib.import_module("isaac_validation_runtime")
        connection = SimpleNamespace()
        connection.read_epoch = lambda: (0, 0.)
        connection.session = SimpleNamespace(priority=SimpleNamespace(source="a" * 64),
                                             owner=SimpleNamespace(state="idle"))
        def step(sub):
            import threading
            if threading.current_thread() is not threading.main_thread():
                raise ValueError("isaac_owner_thread_required")
            if connection.session.priority.source != "a" * 64:
                raise ValueError("runtime_source_identity_changed")
            raise AssertionError("physics forbidden")
        def observe(frame):
            raise ValueError("isaac_observation_frame_order")
        def refresh(sample):
            raise ValueError("refresh_requires_owned_pending_capture")
        connection.step = step
        connection.observe = observe
        connection.refresh_packet = refresh
        with tempfile.TemporaryDirectory(dir=EXPERIMENT / ".runtime") as scratch:
            module.check_runtime_rejections(connection, Path(scratch))
            result = json.loads((Path(scratch) / "runtime_rejections.json").read_text())
            self.assertEqual(len(result["cases"]), 4)
            self.assertTrue(result["epoch_unchanged"])
        self.assertEqual(connection.session.priority.source, "a" * 64)


if __name__ == "__main__":
    unittest.main()
