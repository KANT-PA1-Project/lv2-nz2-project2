import json
import tempfile
import unittest
from pathlib import Path
import numpy as np
from arm_practice.core import joint_targets, reach_score
from arm_practice.urdf import inspect_urdf
from arm_practice.config import load_robot_config

ROOT = Path(__file__).resolve().parents[1]


class NumericalContract(unittest.TestCase):
    def test_action_order_and_clipping(self):
        out = joint_targets([4, -4], [.2, -.1], [.5, .5], [-.3, -.4], [.6, .7])
        np.testing.assert_allclose(out, [.6, -.4])

    def test_nonfinite_and_shape_rejected(self):
        for a in ([float("nan")], [float("inf")], [[0]], [0, 1]):
            with self.assertRaises(ValueError):
                joint_targets(a, [0], [1], [-1], [1])

    def test_reward_distance_and_action_change(self):
        near = reach_score([.01, 0, 0], [0, 0, 0], [0], [0])
        far = reach_score([.2, 0, 0], [0, 0, 0], [0], [0])
        jerk = reach_score([.01, 0, 0], [0, 0, 0], [1], [0])
        self.assertTrue(near["success"])
        self.assertFalse(far["success"])
        self.assertGreater(near["reward"], far["reward"])
        self.assertGreater(near["reward"], jerk["reward"])

    def test_config_joint_order_matches_asset(self):
        cfg = load_robot_config(ROOT / "configs/demo_robot.json")
        report = inspect_urdf(ROOT / "assets/demo_arm.urdf")
        self.assertEqual(cfg["joint_names"], [j["name"] for j in report["movable_joints"]])
        self.assertIn(cfg["tcp_link"], report["links"])
        self.assertFalse(report["warnings"])

    def test_bad_config_rejected(self):
        cfg = json.loads((ROOT / "configs/demo_robot.json").read_text())
        cfg["joint_names"][1] = cfg["joint_names"][0]
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "bad.json"
            p.write_text(json.dumps(cfg))
            with self.assertRaises(ValueError):
                load_robot_config(p)

    def test_disconnected_urdf_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "bad.urdf"
            p.write_text('<robot name="bad"><link name="a"/><link name="b"/></robot>')
            with self.assertRaises(ValueError):
                inspect_urdf(p)

    def test_duplicate_joint_rejected(self):
        s = (ROOT / "assets/demo_arm.urdf").read_text().replace('name="joint2"', 'name="joint1"')
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "bad.urdf"
            p.write_text(s)
            with self.assertRaises(ValueError):
                inspect_urdf(p)


if __name__ == "__main__":
    unittest.main()
