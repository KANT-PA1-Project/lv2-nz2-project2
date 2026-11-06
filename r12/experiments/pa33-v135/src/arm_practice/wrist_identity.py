"""Repeated wrist observations confirm identity; they never authorize grasping."""
import copy
import numpy as np


class WristIdentity:
    def __init__(self):
        self.pending = {}
        self.reason = 'await_wrist_view'
        self.invalidated_ids = set()

    def observe(self, candidate, *, camera, camera_frame, run_id, stamp_ns,
                fresh, view_ready, stationary, arm_velocity, unique_in_frame=True):
        self.reason = 'await_wrist_view'
        self.invalidated_ids.clear()
        if camera != 'arm_camera' or camera_frame != 'arm_camera_left_optical':
            return None
        if not (fresh and view_ready and stationary):
            self.pending.clear()
            return None
        velocity = np.asarray(arm_velocity, float)
        if velocity.shape != (5,) or not np.isfinite(velocity).all() or np.max(np.abs(velocity)) >= .02:
            self.pending.clear()
            self.reason = 'arm_not_settled'
            return None
        try:
            pid = candidate['id']
            center = np.asarray(candidate['center_map_m'], float)
            pose = candidate['pose_camera']
            camera_center = np.asarray(pose['center_camera_m'], float)
            confidence = float(pose['confidence'])
            depth = float(candidate['depth_surface_m'])
            good = (type(pid) is int and 0 <= pid < 12 and unique_in_frame and
                    type(run_id) is str and bool(run_id) and type(stamp_ns) is int and stamp_ns >= 0 and
                    candidate['color'] in ('R', 'G') and candidate['shape'] in ('cylinder', 'cuboid') and
                    center.shape == (3,) and np.isfinite(center).all() and
                    camera_center.shape == (3,) and np.isfinite(camera_center).all() and camera_center[2] > 0 and
                    pose.get('translation_valid') is True and np.isfinite(confidence) and confidence >= .25 and
                    candidate.get('depth_consistent') is True and np.isfinite(depth) and depth > 0)
        except (KeyError, TypeError, ValueError):
            good = False
        if not good:
            if isinstance(candidate, dict) and type(candidate.get('id')) is int:
                self.invalidated_ids.add(candidate['id'])
            self.pending.clear()
            self.reason = 'invalid_wrist_evidence'
            return None
        previous = self.pending.get(pid)
        if previous is not None and (previous['color'] != candidate['color'] or
                                     previous['shape'] != candidate['shape'] or
                                     np.linalg.norm(center - previous['anchor']) > .02):
            self.invalidated_ids.add(pid)
        consistent = (previous is not None and previous['run_id'] == run_id and
                      0 < stamp_ns - previous['stamp_ns'] <= 200_000_000 and
                      previous['color'] == candidate['color'] and previous['shape'] == candidate['shape'] and
                      np.linalg.norm(center - previous['anchor']) <= .02)
        # Duplicate frames cannot advance the evidence counter.
        if previous is not None and previous['run_id'] == run_id and stamp_ns <= previous['stamp_ns']:
            self.reason = 'non_increasing_frame'
            return None
        count = previous['count'] + 1 if consistent else 1
        anchor = previous['anchor'] if consistent else center.copy()
        self.pending[pid] = dict(run_id=run_id, stamp_ns=stamp_ns, count=count,
                                anchor=anchor, color=candidate['color'], shape=candidate['shape'])
        self.reason = 'confirmed' if count >= 3 else 'await_repeated_wrist_evidence'
        if count < 3:
            return None
        result = copy.deepcopy(candidate)
        result.update(camera=camera, camera_frame=camera_frame, run_id=run_id,
                      stamp_ns=stamp_ns, wrist_consistent_frames=count,
                      evidence_role='wrist_confirmed_identity_with_pose_estimate',
                      grasp_pose_accepted=False)
        return result
