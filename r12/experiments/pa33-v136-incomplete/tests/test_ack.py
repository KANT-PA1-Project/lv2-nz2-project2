"""Offline ACK parsing/regression tests; no native or ROS processes."""
import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

EXPERIMENT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(EXPERIMENT / "overlay/arm_practice/native_rgbd"))
from ack_metrics import analyze, capture_identity, legacy_record, make_ack_record, normalize


class AckTests(unittest.TestCase):
    def setUp(self):
        (EXPERIMENT / ".runtime").mkdir(exist_ok=True)
        self.temporary = tempfile.TemporaryDirectory(dir=EXPERIMENT / ".runtime")
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.rows = json.loads((EXPERIMENT / "tests/data/v135_producer_frames_timing.json").read_text())

    def report(self, rows):
        path = self.root / "frames.json"
        path.write_text(json.dumps(rows))
        return analyze(path)

    def assert_invalid(self, rows, reason):
        report = self.report(rows)
        self.assertFalse(report["valid"])
        self.assertIsNone(report["summary"])
        self.assertTrue(any(reason in item["reason"] for item in report["errors"]), report["errors"])
        return report

    def test_v135_ten_sample_regression(self):
        expected = json.loads((EXPERIMENT / "tests/data/v135_expected.json").read_text())
        report = self.report(self.rows)
        self.assertTrue(report["valid"], report["errors"])
        self.assertEqual(report["summary"]["samples"], 10)
        for key in ("p50", "p95", "max"):
            self.assertAlmostEqual(report["summary"][key], expected[key], places=9)

    def test_ack_receive_endpoint_is_independent_of_validation_endpoint(self):
        row = self.rows[0]
        original = self.report([row])["summary"]
        row["ROS_publication"]["producer_transport_timing"]["ack_validation_end_ns"] += 1_000_000
        self.assertEqual(self.report([row])["summary"], original)

    def test_missing_monotonic_capture_never_falls_back_to_sim_stamp(self):
        self.rows[0].pop("capture_monotonic_ns")
        self.assert_invalid(self.rows, "capture_monotonic_ns")

    def test_missing_receive_endpoint_never_falls_back_to_validation(self):
        self.rows[0]["ROS_publication"]["producer_transport_timing"].pop("ack_receive_end_ns")
        self.assert_invalid(self.rows, "ack_receive_end_ns")

    def test_duplicate_capture_is_not_silently_counted(self):
        self.rows.append(copy.deepcopy(self.rows[0]))
        report = self.assert_invalid(self.rows, "duplicate_ACK_capture")
        self.assertEqual(report["valid_subset_summary"]["samples"], 10)

    def test_reordered_capture_is_rejected(self):
        self.assert_invalid([self.rows[1], self.rows[0]], "reordered_ACK_capture")

    def test_mixed_runs_are_not_aggregated(self):
        self.rows[1]["native_verification"]["run_id"] = "other_run"
        self.assert_invalid(self.rows, "mixed_ACK_sessions")

    def test_uncaptured_rows_are_explicitly_skipped(self):
        report = self.report([{"captured": False}] + self.rows)
        self.assertTrue(report["valid"])
        self.assertEqual(report["skipped_uncaptured"], 1)
        self.assertEqual(report["summary"]["samples"], 10)

    def test_empty_or_uncaptured_only_has_no_statistics(self):
        self.assert_invalid([], "no_valid_ACK_samples")
        self.assert_invalid([{"captured": False}], "no_valid_ACK_samples")

    def test_partial_jsonl_is_an_error_and_preserves_diagnostic_subset(self):
        path = self.root / "producer_ack.jsonl"
        record = legacy_record(self.rows[0])
        record["source_sha256"] = "a" * 64
        path.write_text(json.dumps(record) + '\n{"broken":')
        report = analyze(path)
        self.assertFalse(report["valid"])
        self.assertIsNone(report["summary"])
        self.assertEqual(report["valid_subset_summary"]["samples"], 1)
        self.assertEqual(report["errors"][0]["input_record"], 2)

    def test_v136_missing_fine_stages_stay_unavailable(self):
        record = legacy_record(self.rows[0])
        record["source_sha256"] = "a" * 64
        for key in ("serialization_end_ns", "unix_send_start_ns", "header_bytes"):
            record["producer_transport_timing"].pop(key)
        report = self.report([record])
        self.assertTrue(report["valid"])
        self.assertIsNone(report["stage_summary"]["serialization_ms"])
        self.assertIsNone(report["stage_summary"]["unix_send_ms"])
        self.assertEqual(report["stage_missing_samples"]["unix_send_ms"], 1)
        self.assertIsNotNone(report["stage_summary"]["serialization_and_send_ms"])

    def test_wrong_clock_domain_is_rejected(self):
        record = legacy_record(self.rows[0])
        record.update(source_sha256="a" * 64, clock_domain="native_physics_ns")
        self.assert_invalid([record], "unsupported_ACK_schema_or_clock")

    def test_boolean_or_reversed_time_is_rejected(self):
        self.rows[0]["capture_monotonic_ns"] = True
        self.assert_invalid(self.rows, "invalid_integer")
        self.rows[0]["capture_monotonic_ns"] = 10**20
        self.assert_invalid(self.rows, "ACK_clock_order")

    def test_incomplete_ros_receipt_is_not_an_ack_sample(self):
        self.rows[0]["ROS_publication"]["received"] -= 1
        self.assert_invalid(self.rows, "ACK_publication_incomplete")

    def test_capture_stamp_is_bound_to_same_physics_epoch(self):
        self.rows[0]["ROS_publication"]["capture_stamp_ns"] += 1
        self.assert_invalid(self.rows, "ACK_capture_step_stamp_mismatch")

    def test_writer_binds_run_source_and_exact_capture_time(self):
        original = self.rows[0]
        verification = original["native_verification"]
        body = {"run_id": verification["run_id"], "clock_epoch": verification["clock_epoch"], "source_sha256": "a" * 64,
                "capture_physics_step": original["physics_step"], "physics_hz": 960,
                "capture_monotonic_lower_bound_ns": original["capture_monotonic_ns"]}
        identity = capture_identity(body["run_id"], "a" * 64, body)
        ros = original["ROS_publication"]
        record = make_ack_record(identity, ros, ros["producer_transport_timing"])
        self.assertEqual(record["capture_monotonic_ns"], original["capture_monotonic_ns"])
        self.assertEqual(normalize(record)["producer_ACK_ms"], normalize(legacy_record(original), legacy=True)["producer_ACK_ms"])
        with self.assertRaisesRegex(ValueError, "ACK_capture_source_or_run_mismatch"):
            capture_identity(body["run_id"], "b" * 64, body)
        ros["capture_stamp_ns"] += 1
        with self.assertRaisesRegex(ValueError, "ACK_capture_stamp_mismatch"):
            make_ack_record(identity, ros, ros["producer_transport_timing"])

    def test_cli_comparison_same_input_has_zero_delta(self):
        fixture = EXPERIMENT / "tests/data/v135_producer_frames_timing.json"
        output = self.root / "analysis.json"
        env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
        process = subprocess.run([sys.executable, str(EXPERIMENT / "analyze_ack.py"), "--input", str(fixture),
            "--baseline", str(fixture), "--output", str(output)], env=env, capture_output=True, text=True, timeout=30)
        self.assertEqual(process.returncode, 0, process.stderr)
        report = json.loads(output.read_text())
        self.assertTrue(report["valid"])
        self.assertEqual(report["candidate_minus_baseline_ms"], {"p50": 0, "p95": 0, "max": 0})

    def test_cli_invalid_records_return_nonzero_and_write_errors(self):
        path = self.root / "invalid.json"
        path.write_text('[{"captured": true}]')
        output = self.root / "analysis.json"
        process = subprocess.run([sys.executable, str(EXPERIMENT / "analyze_ack.py"), "--input", str(path), "--output", str(output)],
            env=dict(os.environ, PYTHONDONTWRITEBYTECODE="1"), capture_output=True, text=True, timeout=30)
        self.assertEqual(process.returncode, 2)
        report = json.loads(output.read_text())
        self.assertFalse(report["valid"])
        self.assertIsNone(report["summary"])
        self.assertTrue(report["errors"])


    def test_cli_never_overwrites_input(self):
        path = self.root / "frames.json"
        original = json.dumps(self.rows)
        path.write_text(original)
        process = subprocess.run([sys.executable, str(EXPERIMENT / "analyze_ack.py"), "--input", str(path), "--output", str(path)],
            env=dict(os.environ, PYTHONDONTWRITEBYTECODE="1"), capture_output=True, text=True, timeout=30)
        self.assertEqual(process.returncode, 2)
        self.assertIn("analysis_output_must_not_replace_input", process.stderr)
        self.assertEqual(path.read_text(), original)


    def test_source_or_domain_change_in_one_run_is_rejected(self):
        rows = [legacy_record(row) for row in self.rows[:2]]
        for row in rows:
            row["source_sha256"] = "a" * 64
        rows[1]["source_sha256"] = "b" * 64
        self.assert_invalid(rows, "ACK_session_identity_changed")
        rows[1]["source_sha256"] = "a" * 64
        rows[1]["ros_domain_id"] = 34
        self.assert_invalid(rows, "ACK_session_identity_changed")


if __name__ == "__main__":
    unittest.main()
