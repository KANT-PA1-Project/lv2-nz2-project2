"""tower_signal CPU 테스트.

실행 (둘 중 하나)
  - 단독: tutor 를 수정하지 않고 읽기만 함. R12_SRC 기본값 ~/git/lv2-nz2-project2/r12/src
      cd "$HOME/git/lv2-nz2-project2/r12/experiments/barcode_signal" && python3 -m pytest tests/test_tower_signal.py -q
  - tutor 에 통합한 뒤: cd r12 && python3 -m pytest tests/test_tower_signal.py -q
"""
import importlib.util
import os
import sys
import unittest
from pathlib import Path
import numpy as np

_R12_SRC = Path(os.environ.get('R12_SRC', Path.home() / 'git/lv2-nz2-project2/r12/src')).expanduser()
if (_R12_SRC / 'arm_practice/signal_observation.py').is_file():
    sys.path.insert(0, str(_R12_SRC))          # signal_observation 등 tutor 모듈은 읽기만 함
try:
    from arm_practice.tower_signal import TOWER, PARAMS, tier_boxes, tower_signal_color, signal_frame_fresh
except ModuleNotFoundError:                    # tutor 에 아직 없음 → 이 폴더의 사본을 파일 경로로 불러옴
    _p = Path(__file__).resolve().parents[1] / 'src/arm_practice/tower_signal.py'
    _spec = importlib.util.spec_from_file_location('tower_signal', _p)
    _m = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(_m)
    TOWER, PARAMS, tier_boxes, tower_signal_color, signal_frame_fresh = (
        _m.TOWER, _m.PARAMS, _m.tier_boxes, _m.tower_signal_color, _m.signal_frame_fresh)

K = np.array([[400., 0, 319.5], [0, 400., 239.5], [0, 0, 1]])   # r07 카메라 640×480 시뮬 값


def look_at(eye, target):
    """카메라 광학 프레임(x 오른쪽, y 아래, z 앞)의 월드 자세."""
    eye, target = np.asarray(eye, float), np.asarray(target, float)
    z = target - eye; z /= np.linalg.norm(z)
    x = np.cross(z, [0, 0, 1.]); x /= np.linalg.norm(x)
    y = np.cross(z, x)
    T = np.eye(4); T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, y, z, eye
    return T


CENTER = np.array([*TOWER['xy_m'], TOWER['base_z_m'] + 0.138])


def render(T, lit=(), size=(480, 640), tower_depth=True):
    """신호등 칸을 칠한 가짜 영상과 깊이. 켜진 칸 밝기 255, 꺼진 칸 60, 배경 30."""
    rgb = np.full((*size, 3), 30, np.uint8)
    depth = np.full(size, 5.0, np.float32)
    full = dict(PARAMS, inner=1.0, min_tier_px=0)
    boxes, _ = tier_boxes(T, K, rgb.shape, TOWER, full)
    for c, (x0, y0, x1, y1, z) in boxes.items():
        rgb[y0:y1, x0:x1] = (255, 255, 255) if c in lit else (60, 60, 60)
        if tower_depth:
            depth[y0:y1, x0:x1] = z
    return rgb, depth


class TowerSignalTests(unittest.TestCase):
    def setUp(self):
        self.T = look_at(CENTER + [0.0, -0.6, 0.15], CENTER)   # 60 cm 앞, 15 cm 위에서 내려다봄

    def test_each_single_color(self):
        for c in 'RYG':
            rgb, depth = render(self.T, lit=c)
            self.assertEqual(tower_signal_color(rgb, self.T, K, depth)['color'], c)

    def test_all_off(self):
        rgb, depth = render(self.T)
        self.assertEqual(tower_signal_color(rgb, self.T, K, depth)['color'], 'OFF')

    def test_red_is_top_tier(self):
        boxes, _ = tier_boxes(self.T, K, (480, 640))
        self.assertLess(boxes['R'][1], boxes['Y'][1]); self.assertLess(boxes['Y'][1], boxes['G'][1])

    def test_two_lit_is_unknown(self):
        rgb, depth = render(self.T, lit='RG')
        r = tower_signal_color(rgb, self.T, K, depth)
        self.assertEqual(r['color'], 'UNKNOWN'); self.assertTrue(r['reason'].startswith('multiple_lit'))

    def test_not_visible_is_unknown(self):
        away = look_at(CENTER + [0.0, -0.6, 0.15], CENTER + [2.0, 0, 0])   # 다른 곳을 봄
        rgb, _ = render(self.T, lit='R')
        self.assertEqual(tower_signal_color(rgb, away, K)['color'], 'UNKNOWN')
        behind = look_at(CENTER + [0.0, -0.6, 0.15], CENTER + [0, -2.0, 0])
        self.assertEqual(tower_signal_color(rgb, behind, K)['reason'], 'behind_camera')

    def test_too_far_is_unknown(self):
        far = look_at(CENTER + [0.0, -12.0, 0.15], CENTER)
        rgb = np.full((480, 640, 3), 30, np.uint8)
        self.assertEqual(tower_signal_color(rgb, far, K)['reason'], 'too_small')

    def test_occlusion_is_unknown(self):
        rgb, depth = render(self.T, lit='Y')
        depth[:, :] = 0.2                       # 신호등 앞을 무언가가 가림
        r = tower_signal_color(rgb, self.T, K, depth)
        self.assertEqual(r['color'], 'UNKNOWN'); self.assertTrue(r['reason'].startswith('occluded'))

    def test_without_depth_still_works(self):
        rgb, _ = render(self.T, lit='G')
        self.assertEqual(tower_signal_color(rgb, self.T, K)['color'], 'G')

    def test_bad_image(self):
        self.assertEqual(tower_signal_color(np.zeros((480, 640)), self.T, K)['color'], 'UNKNOWN')


    def _paint_fraction(self, rgb, color, fraction, value=(255, 255, 255)):
        import cv2
        module=sys.modules.get(tower_signal_color.__module__,globals().get('_m'))
        boxes,polygons,_=module.tier_rois(self.T,K,rgb.shape)
        x0,y0,x1,y1,_=boxes[color]
        mask=np.zeros(rgb.shape[:2],np.uint8)
        cv2.fillConvexPoly(mask,np.rint(polygons[color]).astype(np.int32),1)
        ys,xs=np.nonzero(mask)
        rgb[ys,xs]=60
        n=int(len(xs)*fraction)
        rgb[ys[:n],xs[:n]]=value

    def test_dominant_light_survives_adjacent_bloom(self):
        for active, neighbor in [('R', 'Y'), ('Y', 'G'), ('G', 'Y')]:
            rgb, depth = render(self.T, lit=active)
            self._paint_fraction(rgb, neighbor, 0.25)
            r = tower_signal_color(rgb, self.T, K, depth)
            self.assertGreaterEqual(r['ratios'][neighbor], PARAMS['on_ratio'])
            self.assertEqual(r['color'], active)
            self.assertEqual(r['decision'], 'dominant_bloom')

    def test_similar_bright_tiers_remain_unknown(self):
        rgb, depth = render(self.T, lit='R')
        self._paint_fraction(rgb, 'Y', 0.85)
        self.assertEqual(tower_signal_color(rgb, self.T, K, depth)['color'], 'UNKNOWN')

    def test_all_white_tiers_remain_unknown(self):
        rgb, depth = render(self.T, lit='RYG')
        self.assertEqual(tower_signal_color(rgb, self.T, K, depth)['color'], 'UNKNOWN')

    def test_small_reflection_does_not_count_as_on(self):
        rgb, depth = render(self.T)
        self._paint_fraction(rgb, 'R', 0.05)
        self.assertEqual(tower_signal_color(rgb, self.T, K, depth)['color'], 'OFF')

    def test_colored_light_does_not_require_white_saturation(self):
        rgb, depth = render(self.T)
        self._paint_fraction(rgb, 'R', 1.0, (255, 30, 20))
        self._paint_fraction(rgb, 'Y', 0.25)
        r = tower_signal_color(rgb, self.T, K, depth)
        self.assertEqual(r['white_ratios']['R'], 0)
        self.assertEqual(r['color'], 'R')

    def test_near_threshold_runner_stays_unknown(self):
        rgb, depth = render(self.T)
        self._paint_fraction(rgb, 'R', 0.20)
        self._paint_fraction(rgb, 'Y', 0.14)
        self.assertEqual(tower_signal_color(rgb, self.T, K, depth)['color'], 'UNKNOWN')

    def test_bright_cap_area_outside_inner_roi_is_ignored(self):
        rgb, depth = render(self.T)
        full, _ = tier_boxes(self.T, K, rgb.shape, params=dict(PARAMS, inner=1.0))
        inner, _ = tier_boxes(self.T, K, rgb.shape)
        x0, y0, x1, _, _ = full['R']
        rgb[max(0, y0 - 8):inner['R'][1], x0:x1] = 255
        self.assertEqual(tower_signal_color(rgb, self.T, K, depth)['color'], 'OFF')


    def test_oblique_view_front_centers_are_in_their_own_tiers(self):
        eye = np.array([0.6480880799, 0.925, 0.8767])
        T = look_at(eye, eye + [-np.sqrt(3) / 2, 0, -0.5])
        boxes, reason = tier_boxes(T, K, (480, 640))
        self.assertIsNone(reason)
        xy = np.array(TOWER['xy_m'])
        front = xy + TOWER['lens_diameter_m'] / 2 * (eye[:2] - xy) / np.linalg.norm(eye[:2] - xy)
        for color, (z0, z1) in TOWER['tiers'].items():
            point = np.array([*front, TOWER['base_z_m'] + (z0 + z1) / 2, 1.0])
            cam = np.linalg.inv(T) @ point
            uv = (K @ cam[:3])[:2] / cam[2]
            for other, (x0, y0, x1, y1, _) in boxes.items():
                inside = x0 <= uv[0] < x1 and y0 <= uv[1] < y1
                self.assertEqual(inside, other == color)

    def test_vertical_view_is_unknown(self):
        T = np.diag([1., -1., -1., 1.])
        T[:3, 3] = [*TOWER['xy_m'], 1.0]
        boxes, reason = tier_boxes(T, K, (480, 640))
        self.assertIsNone(boxes)
        self.assertEqual(reason, 'view_too_vertical')


class CameraSamplingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = Path(__file__).resolve().parents[1] / 'scripts/test_tower_signal_isaac.py'
        spec = importlib.util.spec_from_file_location('tower_isaac_sampling', path)
        cls.script = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.script)

    def test_same_seed_and_view_are_reproducible(self):
        a, sa = self.script.random_camera_pose(33, 2)
        b, sb = self.script.random_camera_pose(33, 2)
        np.testing.assert_array_equal(a, b)
        self.assertEqual(sa, sb)

    def test_view_index_changes_camera(self):
        a, _ = self.script.random_camera_pose(33, 0)
        b, _ = self.script.random_camera_pose(33, 1)
        self.assertFalse(np.allclose(a, b))

    def test_ranges_and_rotation(self):
        for index in range(20):
            T, sample = self.script.random_camera_pose(33, index)
            for key, (low, high) in self.script.RANDOM_VIEW_RANGES.items():
                self.assertGreaterEqual(sample[key], low)
                self.assertLessEqual(sample[key], high)
            np.testing.assert_allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(T[:3, :3]), 1.0)

    def test_sampling_keeps_robot_pose_fixed(self):
        before = self.script.START_POSE
        for index in range(5):
            self.script.random_camera_pose(33, index)
        self.assertEqual(self.script.START_POSE, before)


class FreshnessTests(unittest.TestCase):
    """기존 test_signal_observation.py 와 같은 packet 형식."""
    def setUp(self):
        roi = dict(run_id='a', stamp_ns=1000000000, capture_monotonic_ns=100, camera='r07_camera1')
        self.p = dict(capture_run_id='a', stamp_ns=1000000000, frame_capture_monotonic_ns=100, signal_color='UNKNOWN',
                      signal_observation=dict(run_id='a', stamp_ns=1000000000, capture_monotonic_ns=100,
                                              camera='r07_camera1', clock_domain='synthetic_packet_20hz',
                                              color='UNKNOWN', roi_source=roi))

    def test_fresh_even_if_producer_color_unknown(self):
        self.assertTrue(signal_frame_fresh(self.p, now_ns=130, max_age_ns=30))
        self.assertEqual(self.p['signal_observation']['color'], 'UNKNOWN')   # 원본 안 바뀜

    def test_stale_or_mismatch(self):
        self.assertFalse(signal_frame_fresh(self.p, now_ns=131, max_age_ns=30))
        self.p['signal_observation']['run_id'] = 'b'
        self.assertFalse(signal_frame_fresh(self.p, now_ns=130, max_age_ns=30))
        self.assertFalse(signal_frame_fresh({}, now_ns=130, max_age_ns=30))


if __name__ == '__main__':
    unittest.main()
