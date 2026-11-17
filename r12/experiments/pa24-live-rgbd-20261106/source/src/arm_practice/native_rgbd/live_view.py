"""In-process RGB-D view proposal. No actuator or calibration authority.

Accept only a current issuer verification; never load archived observations.
The caller must still prove continuous swept safety before any movement.
"""
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares

from arm_practice.tag_detection import detect_tags, detect_tag_evidence
from camera_rig import PreparedCameraRig
from .identity_reobservation import IdentityReobservation


_CANDIDATE_MINTER = object()


class LiveViewCandidate:
    """Opaque process-local dispatch value; JSON/dict replay is unsupported."""
    __slots__ = ('_data',)
    def __init__(self,data,*,_mint):
        if _mint is not _CANDIDATE_MINTER:raise ValueError('untrusted_candidate_minter')
        self._data=dict(data)
        for key in ('q','q_start'):
            value=np.asarray(self._data[key],float).copy();value.flags.writeable=False
            self._data[key]=value
    def __getitem__(self,key):return self._data[key]


class LiveViewPlanner:
    CAMERA = "arm_camera_left_optical"
    DEPTH = "arm_camera_depth"
    MAX_AGE_NS = 500_000_000

    def __init__(self, source, issuer, schema, geometry, read_epoch, read_q,
                 read_encoder, clock=time.monotonic_ns):
        self.source = Path(source)
        self.rig = PreparedCameraRig(self.source)
        self.issuer = issuer
        self.schema = schema
        self.geometry = geometry
        self.read_epoch = read_epoch
        self.read_q = read_q
        self.read_encoder = read_encoder
        self.clock = clock
        cv2.setNumThreads(1)
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11))
        urdf = ET.parse(self.source / "models/r12/r12_arm.urdf").getroot()
        limits = {j.get("name"): (float(j.find("limit").get("lower")),
                                  float(j.find("limit").get("upper")))
                  for j in urdf.findall("joint") if j.find("limit") is not None}
        self.lower = np.array([limits[f"arm_joint{i}"][0] + .02 for i in range(1, 6)])
        self.upper = np.array([limits[f"arm_joint{i}"][1] - .02 for i in range(1, 6)])
        self.last_step = -1
        self.identity_tracker=IdentityReobservation(issuer.run_id)
        self.identity_id = None
        self.identity_streak = 0
        self.identity_evidence = None
        self._live_candidate = None
        if schema.config["depth"][self.DEPTH]["rgb_camera"] != self.CAMERA:
            raise ValueError("unpaired_rgb_depth")
        # Initialize detector kernels before the timed live capture. This blank
        # image has no target or control meaning and creates no certificate.
        detect_tags(np.zeros((480, 640, 3), dtype=np.uint8), self.detector)
        least_squares(lambda x: x, np.zeros(3), max_nfev=2)

    def _current(self, body, raw_reference=None):
        now = self.clock()
        if not 0 <= now - body["capture_monotonic_lower_bound_ns"] <= self.MAX_AGE_NS:
            raise ValueError("capture_stale_before_proposal_return")
        if self.read_epoch()[0] != body["capture_physics_step"]:
            raise ValueError("physics_advanced_during_proposal")
        if body["capture_physics_step"] != body["current_physics_step"]:
            raise ValueError("capture_not_current_epoch")
        q = np.asarray(self.read_q(), dtype=float)
        encoder = np.asarray(self.read_encoder(), dtype=float)
        if q.shape != (5,) or not np.isfinite(q).all() or encoder.shape != (5,) or not np.array_equal(
                encoder, np.asarray(body["encoder_history"]["encoder_q"], dtype=float)):
            raise ValueError("feedback_changed_during_proposal")
        if raw_reference is not None and not np.array_equal(q, raw_reference):
            raise ValueError("native_feedback_changed_during_proposal")
        return now, q.copy()

    def propose(self, result):
        start = self.clock()
        self.identity_evidence = None
        self._live_candidate = None
        report = dict(accepted=False, motion_authorized=False,
                      calibration_accepted=False, fixture_target_coordinates_read=False,
                      scope="same-process current RGB-D view proposal only")
        try:
            if result.get("captured") is not True:
                raise ValueError("capture_failed")
            body = result["certificate"]["body"]
            _, raw_reference = self._current(body)
            if body["capture_physics_step"] <= self.last_step:
                raise ValueError("duplicate_live_proposal")
            verify_start = self.clock()
            verification = self.issuer.verify(
                result["certificate"], pixels=result["pixels"], depths=result["depths"],
                now_ns=self.clock(), expected_current_run=self.issuer.run_id)
            report["verification_elapsed_ms"] = (self.clock()-verify_start)/1e6
            report["verification"] = verification
            if verification.get("matched") is not True:
                raise ValueError(verification.get("reason", "unverified_capture"))
            self.last_step = body["capture_physics_step"]
            observe_start = self.clock()
            observation = self._observe(result, body)
            report["observation_elapsed_ms"] = (self.clock()-observe_start)/1e6
            fit_start = self.clock()
            q, pixel = self._fit(observation, body)
            report["fit_elapsed_ms"] = (self.clock()-fit_start)/1e6
            now, _ = self._current(body, raw_reference)
            identity = observation['id']
            identity_state=self.identity_tracker.confirm(identity,physics_step=body['capture_physics_step'],run_id=body['run_id'])
            self.identity_streak = identity_state['identity_streak']
            self.identity_id = identity
            if self.identity_streak>=2:
                self._live_candidate=dict(body=body,raw_reference=raw_reference.copy(),q=q.copy())
            report.update(identity_confirmed=self.identity_streak>=2,
                          identity_streak=self.identity_streak,
                          reobserve_required=self.identity_streak<2)
            report.update(accepted=True, reason="fresh_view_proposal_no_motion_authority",
                          observation=observation, candidate_q_rad=q.tolist(),
                          candidate_surface_pixel=pixel,
                          capture_physics_step=body["capture_physics_step"],
                          capture_age_ms=(now-body["capture_monotonic_lower_bound_ns"])/1e6,
                          movement_veto="continuous_swept_clearance_not_verified")
        except (ValueError, KeyError, TypeError, OverflowError, np.linalg.LinAlgError) as exc:
            self.identity_tracker.reject()
            self.identity_id = None
            self.identity_streak = 0
            report.update(reason=str(exc), movement_veto="live_observation_rejected",
                          identity_confirmed=False, identity_streak=0, reobserve_required=True)
        report["identity_evidence"] = self.identity_evidence
        report["proposal_elapsed_ms"] = (self.clock()-start)/1e6
        return report

    def consume_live_candidate(self):
        """Single-use in-memory candidate; there is no archived-input path."""
        candidate=self._live_candidate
        self._live_candidate=None
        if candidate is None:raise ValueError('no_live_confirmed_candidate')
        body=candidate['body']
        self._current(body,candidate['raw_reference'])
        if self.issuer.last_verified!=body['capture_physics_step']:
            raise ValueError('candidate_issuer_state_changed')
        settling=body['encoder_history'].get('capture_settling',{})
        if settling.get('settled_interval') is not True or settling.get('continuous_steps',0)<480:
            raise ValueError('candidate_capture_not_settled')
        return LiveViewCandidate(dict(q=candidate['q'].copy(),q_start=candidate['raw_reference'].copy(),
                    capture_step=body['capture_physics_step'],
                    capture_lower_ns=body['capture_monotonic_lower_bound_ns'],
                    run_id=body['run_id'],issuer_id=body['issuer_id']),_mint=_CANDIDATE_MINTER)

    def _observe(self, result, body):
        rgb = result["pixels"][self.CAMERA]
        depth = result["depths"][self.DEPTH]
        fx, fy, cx, cy = self.schema.config["cameras"][self.CAMERA]["intrinsics"]
        evidence = detect_tag_evidence(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), self.detector)
        self.identity_evidence = evidence
        tags = evidence['tags']
        if evidence['ambiguous']:
            raise ValueError("ambiguous_or_unregistered_tag_evidence")
        if len(tags)>1:
            raise ValueError("multiple_visible_markers")
        if len(tags)!=1 or tags[0]['mission_marker'] is None:
            raise ValueError("absent_or_unregistered_tag")
        if tags[0]['mission_marker'].get('role')!='puck':
            raise ValueError("destination_marker_not_object_target")
        tag = tags[0]
        corners = np.asarray(tag["corners"], dtype=float)
        center = corners.mean(axis=0)
        mask = np.zeros(depth.shape, np.uint8)
        cv2.fillConvexPoly(mask, np.rint(center+.5*(corners-center)).astype(np.int32), 1)
        yy, xx = np.nonzero(mask)
        if len(xx) < 25:
            raise ValueError("insufficient_native_depth")
        valid = np.isfinite(depth[yy, xx]) & (depth[yy, xx] > .01) & (depth[yy, xx] < 30)
        if valid.mean() < .95 or valid.sum() < 25:
            raise ValueError("insufficient_native_depth")
        yy, xx = yy[valid], xx[valid]
        z = depth[yy, xx].astype(float)
        points = np.c_[(xx-cx)*z/fx, (yy-cy)*z/fy, z]
        median = np.median(points, axis=0)
        _, _, vt = np.linalg.svd(points-median, full_matrices=False)
        inliers = abs((points-median)@vt[-1]) < .002
        if inliers.mean() < .95:
            raise ValueError("surface_plane_inconsistent")
        centroid = points[inliers].mean(axis=0)
        _, _, vt = np.linalg.svd(points[inliers]-centroid, full_matrices=False)
        normal = vt[-1]
        ray = np.array([(center[0]-cx)/fx, (center[1]-cy)/fy, 1.])
        if abs(normal@ray) < .2:
            raise ValueError("grazing_observation")
        point = ray * (normal@centroid/(normal@ray))
        if not np.isfinite(point).all() or point[2] <= .01:
            raise ValueError("invalid_surface_point")
        world = (np.asarray(body["encoder_history"]["camera_fk"][self.CAMERA]) @ np.r_[point, 1])[:3]
        return dict(id=int(tag["id"]), agreeing_scales=tag["agreeing_scales"],
                    center_pixel=center.tolist(), surface_point_world_m=world.tolist(),
                    surface_point_optical_m=point.tolist(), native_depth_samples=len(z),
                    plane_inlier_fraction=float(inliers.mean()),
                    object_center_or_orientation_estimated=False)

    def _fit(self, observation, body):
        initial = np.asarray(body["encoder_history"]["encoder_q"], dtype=float)
        if np.any(initial < self.lower) or np.any(initial > self.upper):
            raise ValueError("initial_joint_margin")
        target = np.r_[observation["surface_point_world_m"], 1.]
        def camera(q):
            joints = {f"arm_joint{i+1}": float(v) for i, v in enumerate(q)}
            joints.update(self.geometry["gripper_open_positions"])
            return self.rig.transforms(joints)[self.CAMERA]
        recorded = np.asarray(body["encoder_history"]["camera_fk"][self.CAMERA])
        base = recorded @ np.linalg.inv(camera(initial))
        def project(q):
            return np.linalg.inv(base @ camera(q)) @ target
        def residual(x):
            q = initial.copy(); q[1:4] = x
            p = project(q)
            return np.r_[p[1]*12, max(0., .25-p[2])*12,
                         max(0., p[2]-.65)*12, (x-initial[1:4])*.01]
        fit = least_squares(residual, initial[1:4],
                            bounds=(self.lower[1:4], self.upper[1:4]), max_nfev=300)
        q = initial.copy(); q[1:4] = fit.x
        p = project(q)
        if not fit.success or not np.isfinite(q).all() or p[2] <= 0 or abs(p[1]/p[2]) > .4:
            raise ValueError("view_proposal_outside_central_view")
        if np.any(q < self.lower) or np.any(q > self.upper):
            raise ValueError("candidate_joint_margin")
        fx, fy, cx, cy = self.schema.config["cameras"][self.CAMERA]["intrinsics"]
        return q, [float(fx*p[0]/p[2]+cx), float(fy*p[1]/p[2]+cy)]
