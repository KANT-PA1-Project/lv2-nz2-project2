"""CPU boundary tests with synthetic state; these never authorize movement."""
import copy
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np
ROOT = Path(__file__).resolve().parent / "source"
sys.path[:0] = [str(ROOT / "src"), str(ROOT / "scripts/sensor_runtime")]
from arm_practice.native_rgbd.live_view import LiveViewPlanner
from arm_practice.native_rgbd.mixed_camera_schema import MixedCameraSchema
from arm_practice.native_rgbd.renderer_epoch_issuer import RendererEpochIssuer


class StubIssuer:
    run_id = "synthetic-unit-test"
    def verify(self, *args, **kwargs):
        return {"matched": True}


class LifecycleTests(unittest.TestCase):
    def setUp(self):
        self.now = 1_000_000_000
        self.step = 500
        self.q = np.array([0., .25, -.33, -.33, 0.])
        schema = MixedCameraSchema.from_file(ROOT / "configs/mixed_camera_schema.json")
        self.planner = LiveViewPlanner(
            ROOT, StubIssuer(), schema, json.loads((ROOT / "cad_geometry.json").read_text()),
            lambda: (self.step, 0.), lambda: self.q, lambda: self.q, lambda: self.now)
        self.body = dict(run_id=self.planner.issuer.run_id,capture_monotonic_lower_bound_ns=self.now,
                         capture_physics_step=self.step, current_physics_step=self.step,
                         encoder_history={"encoder_q": self.q.tolist()})
        self.capture = dict(captured=True, certificate={"body": self.body}, pixels={}, depths={})
        self.planner._observe = lambda *a: {"id": 0}
        self.planner._fit = lambda *a: (self.q.copy(), [319.5, 239.5])

    def assertRejected(self, reason):
        r = self.planner.propose(self.capture)
        self.assertFalse(r["accepted"])
        self.assertFalse(r["motion_authorized"])
        self.assertEqual(r["reason"], reason)

    def test_fresh_proposal_still_vetoes_movement(self):
        r = self.planner.propose(self.capture)
        self.assertTrue(r["accepted"])
        self.assertFalse(r["motion_authorized"])
        self.assertEqual(r["movement_veto"], "continuous_swept_clearance_not_verified")

    def test_stale_at_entry(self):
        self.now += 500_000_001
        self.assertRejected("capture_stale_before_proposal_return")

    def test_future_timestamp(self):
        self.now -= 1
        self.assertRejected("capture_stale_before_proposal_return")

    def test_stale_after_fitting(self):
        def fit(*a):
            self.now += 500_000_001
            return self.q.copy(), []
        self.planner._fit = fit
        self.assertRejected("capture_stale_before_proposal_return")

    def test_epoch_advanced_after_fitting(self):
        def fit(*a):
            self.step += 1
            return self.q.copy(), []
        self.planner._fit = fit
        self.assertRejected("physics_advanced_during_proposal")

    def test_feedback_changed_after_fitting(self):
        def fit(*a):
            self.q[1] += .001
            return self.q.copy(), []
        self.planner._fit = fit
        self.assertRejected("feedback_changed_during_proposal")

    def test_capture_and_current_epoch_mismatch(self):
        self.body["current_physics_step"] += 1
        self.assertRejected("capture_not_current_epoch")

    def test_duplicate_consumption(self):
        self.assertTrue(self.planner.propose(self.capture)["accepted"])
        self.assertRejected("duplicate_live_proposal")

    def test_unverified_capture(self):
        self.planner.issuer.verify = lambda *a, **k: {"matched": False, "reason": "bad_signature"}
        self.assertRejected("bad_signature")

    def test_capture_failure(self):
        self.capture["captured"] = False
        self.assertRejected("capture_failed")

    def test_nonfinite_feedback(self):
        self.q[0] = np.nan
        self.assertRejected("feedback_changed_during_proposal")

    def test_missing_observation_fields(self):
        del self.body["encoder_history"]
        self.assertRejected("'encoder_history'")

    def test_raw_and_encoder_are_compared_in_their_own_domains(self):
        encoder = self.q.copy()
        self.q[1] += .0003
        self.planner.read_encoder = lambda: encoder
        self.assertTrue(self.planner.propose(self.capture)["accepted"])

    def test_sub_encoder_bin_native_drift_rejected(self):
        encoder = self.q.copy()
        self.planner.read_encoder = lambda: encoder
        def fit(*a):
            self.q[1] += .00001
            return self.q.copy(), []
        self.planner._fit = fit
        self.assertRejected("native_feedback_changed_during_proposal")


class SensorTests(unittest.TestCase):
    def setUp(self):
        self.schema = MixedCameraSchema.from_file(ROOT / "configs/mixed_camera_schema.json")
        self.geometry = json.loads((ROOT / "cad_geometry.json").read_text())
        self.planner = LiveViewPlanner(ROOT, StubIssuer(), self.schema, self.geometry,
                                       lambda: (0, 0.), lambda: np.zeros(5), lambda: np.zeros(5))
        self.rgb = np.zeros((480, 640, 3), dtype=np.uint8)
        self.depth = np.ones((480, 640), dtype=np.float32)*.3
        self.capture = {"pixels": {self.planner.CAMERA: self.rgb},
                        "depths": {self.planner.DEPTH: self.depth}}
        self.body = {"encoder_history": {"camera_fk": {self.planner.CAMERA: np.eye(4).tolist()}}}
        self.tag = dict(id=0, mission_marker={"role": "puck"}, agreeing_scales=[1,2,3,4],
                        corners=[[300.,220.],[340.,220.],[340.,260.],[300.,260.]])

    def test_no_tag_rejected(self):
        with self.assertRaisesRegex(ValueError, "absent_or_unregistered"):
            self.planner._observe(self.capture, self.body)

    def test_ambiguous_tags_rejected(self):
        with patch("arm_practice.native_rgbd.live_view.detect_tag_evidence", return_value={"tags":[self.tag,self.tag],"ambiguous":False}):
            with self.assertRaisesRegex(ValueError, "multiple_visible_markers"):
                self.planner._observe(self.capture, self.body)

    def test_invalid_depth_rejected(self):
        self.depth[:] = np.inf
        with patch("arm_practice.native_rgbd.live_view.detect_tag_evidence", return_value={"tags":[self.tag],"ambiguous":False}):
            with self.assertRaisesRegex(ValueError, "insufficient_native_depth"):
                self.planner._observe(self.capture, self.body)

    def test_surface_only_reconstruction(self):
        with patch("arm_practice.native_rgbd.live_view.detect_tag_evidence", return_value={"tags":[self.tag],"ambiguous":False}):
            r = self.planner._observe(self.capture, self.body)
        self.assertAlmostEqual(r["surface_point_optical_m"][2], .3, places=6)
        self.assertFalse(r["object_center_or_orientation_estimated"])

    def test_forged_certificate_rejected_by_real_issuer(self):
        issuer = RendererEpochIssuer(run_id="unit-test", clock_epoch="unit-test",
            cameras=list(self.schema.config["cameras"]),
            joint_names=[f"arm_joint{i}" for i in range(1,6)], physics_hz=960,
            model_sha256="0"*64, source_sha256="1"*64, fk=lambda q: {},
            depth_camera_map={k:v["rgb_camera"] for k,v in self.schema.config["depth"].items()},
            camera_schema=self.schema)
        verdict = issuer.verify({"body": {}, "signature": "0"*64},
                                pixels={}, depths={}, now_ns=1,
                                expected_current_run="unit-test")
        self.assertFalse(verdict["matched"])
        self.assertEqual(verdict["reason"], "untrusted_or_modified_issuer_certificate")

    def test_conflicting_group_rejected_even_with_one_agreed_tag(self):
        with patch("arm_practice.native_rgbd.live_view.detect_tag_evidence", return_value={"tags":[self.tag],"ambiguous":True}):
            with self.assertRaisesRegex(ValueError,"ambiguous_or_unregistered_tag_evidence"):
                self.planner._observe(self.capture,self.body)

    def test_real_destination_marker_cannot_be_used_as_object_target(self):
        import cv2
        self.rgb[:] = 255
        marker = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),12,100)
        self.rgb[190:290,270:370] = marker[:,:,None]
        with self.assertRaisesRegex(ValueError,"destination_marker_not_object_target"):
            self.planner._observe(self.capture,self.body)


class RigParityTests(unittest.TestCase):
    def test_prepared_rig_matches_current_joint_fk(self):
        from camera_rig import PreparedCameraRig, transforms
        rig = PreparedCameraRig(ROOT)
        geom = json.loads((ROOT / "cad_geometry.json").read_text())
        rng = np.random.default_rng(2427)
        for _ in range(12):
            values = {f"arm_joint{i+1}": float(x) for i,x in enumerate(rng.uniform(-.3,.3,5))}
            values.update(geom["gripper_open_positions"])
            expected = transforms(ROOT, values)
            actual = rig.transforms(values)
            self.assertEqual(set(actual),set(expected))
            for key in actual:
                np.testing.assert_allclose(actual[key],expected[key],atol=1e-12,rtol=0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
