"""Regression checks for torque retention using a simulated arm connection."""
import contextlib
import io
import json
from pathlib import Path
import signal
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import arm_j2_tune_test as trial


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        self.now += 0.05
        return self.now

    def time(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class FakeArm:
    def __init__(self, failure=None, torque=False):
        self.hello = {"diag_rev": 2}
        self.commands = []
        self.torque = torque
        self.state = 0
        self.position = [0.0, 20.0, 10.0, 0.0, 0.0, -100.0]
        self.gain = 800
        self.failure = failure
        self.closed = False

    def identity(self):
        return "NZ2_ARM fw=2.0 limits_rev=8 ok=6/6 fail=-"

    def snapshot(self):
        return dict(state=self.state, p_state=self.state, torque=self.torque,
                    fault_code=0, hold_fail_mask=0, offline_mask=0,
                    hardware_errors=[0]*6, current_ma=[0.0]*6,
                    position_deg=list(self.position))

    def query(self, command, prefix, timeout=None):
        self.commands.append(command)
        if command == "T 1":
            self.torque = True
            if self.failure == "enable_ack":
                raise OSError("Lost enable ACK")
        elif command == "T 0":
            self.torque = False
        elif command == "S":
            if self.failure == "stop":
                raise OSError("USB unavailable")
            self.state = 2
        elif command == "R":
            self.state = 0
        elif command.startswith("J "):
            self.gain = int(command.split()[1])
        return prefix

    def send(self, command):
        self.commands.append(command)
        if command.startswith("A "):
            if self.failure == "motion":
                raise RuntimeError("Tracking failure")
            if self.failure == "interrupt":
                raise KeyboardInterrupt()
            if self.failure == "sigterm":
                signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)
            self.position = [float(value) for value in command.split()[1:]]

    def close(self):
        self.closed = True


class TorqueExitTests(unittest.TestCase):
    def run_trial(self, arm, *options):
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory)/"trial.jsonl"
            argv = ["trial", "--gain", "800", "--stage", "home", "--log", str(log), *options]
            settings = lambda rows: {
                "complete": True,
                "raw": {"operating_mode": 3, "goal_pwm": 885,
                        "position_p_gain": arm.gain, "torque_enable": int(arm.torque)}}
            with patch.object(sys, "argv", argv), \
                 patch.object(trial.ArmSerial, "discover", return_value=arm), \
                 patch.object(trial, "time", FakeClock()), \
                 patch.object(trial, "read_diagnostics", side_effect=lambda obj: (obj.snapshot(), [])), \
                 patch.object(trial, "summarize", side_effect=settings), \
                 patch.object(trial, "load_sample", return_value={"current_ma": 0.0}), \
                 contextlib.redirect_stdout(io.StringIO()):
                result = trial.main()
            rows = [json.loads(line) for line in log.read_text().splitlines()]
        self.assertTrue(arm.closed)
        return result, rows

    def test_success_retains_torque_and_holds(self):
        arm = FakeArm()
        result, _ = self.run_trial(arm)
        self.assertEqual(result, 0)
        self.assertTrue(arm.torque)
        self.assertIn("S", arm.commands)
        self.assertNotIn("T 0", arm.commands)
        self.assertEqual(arm.position[:5], [0.0]*5)

    def test_failures_retain_torque(self):
        for failure in ("motion", "interrupt", "sigterm", "enable_ack", "stop"):
            with self.subTest(failure=failure):
                arm = FakeArm(failure)
                result, _ = self.run_trial(arm)
                self.assertEqual(result, 1)
                self.assertTrue(arm.torque)
                self.assertNotIn("T 0", arm.commands)
                self.assertIn("K", arm.commands)

    def test_return_to_start_keeps_torque(self):
        arm = FakeArm()
        start = list(arm.position)
        result, _ = self.run_trial(arm, "--return-to-start")
        self.assertEqual(result, 0)
        self.assertEqual(arm.position, start)
        self.assertTrue(arm.torque)
        self.assertNotIn("T 0", arm.commands)

    def test_gain_restoration_is_deferred_while_enabled(self):
        arm = FakeArm()
        result, rows = self.run_trial(arm, "--gain", "1000")
        self.assertEqual(result, 0)
        self.assertEqual(arm.gain, 1000)
        self.assertTrue(any(row["event"] == "gain_restore_pending" for row in rows))
        self.assertNotIn("T 0", arm.commands)

    def test_explicit_supported_off_restores_gain(self):
        arm = FakeArm(torque=True)
        arm.gain = 1000
        result, _ = self.run_trial(arm, "--stage", "supported-off", "--support-confirmed")
        self.assertEqual(result, 0)
        self.assertFalse(arm.torque)
        self.assertEqual(arm.gain, 800)
        self.assertEqual(arm.commands[:3], ["S", "T 0", "J 800"])
        self.assertFalse(any(command.startswith("A ") for command in arm.commands))

    def test_supported_off_requires_physical_confirmation(self):
        argv = ["trial", "--gain", "800", "--stage", "supported-off", "--log", "/tmp/unused"]
        with patch.object(sys, "argv", argv), \
             patch.object(trial.ArmSerial, "discover") as discover, \
             contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as error:
                trial.main()
        self.assertEqual(error.exception.code, 2)
        discover.assert_not_called()


if __name__ == "__main__":
    unittest.main()
