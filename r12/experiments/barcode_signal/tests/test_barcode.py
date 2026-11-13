import sys
import unittest
from pathlib import Path

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from barcode_support import ALLOWED, ROOT, StableCode, decode_rgb, label_corners, projected_bars_px
from test_tower_signal_isaac import look_at, K


class BarcodeTests(unittest.TestCase):
    def test_all_four_photographic_assets(self):
        for code in sorted(ALLOWED):
            with self.subTest(code=code):
                image = np.asarray(Image.open(ROOT / 'references' / (code + '.png')).convert('RGB'))
                self.assertEqual({r['text'] for r in decode_rgb(image)['reads']}, {code})

    def test_fallback_records_raw_and_scales_positions(self):
        import cv2
        path = ROOT / 'references/P2-02.png'
        small = cv2.resize(np.asarray(Image.open(path)), (150, 50), interpolation=cv2.INTER_AREA)
        base = decode_rgb(small)
        result = decode_rgb(small, upscale=2)
        self.assertEqual(result['raw_read_count'], len(base['reads']))
        self.assertEqual(result['attempted_upscale'], 2)
        for read in result['reads']:
            for x, y in read['corners']:
                self.assertLessEqual(x, small.shape[1])
                self.assertLessEqual(y, small.shape[0])

    def test_reject_bad_upscale(self):
        with self.assertRaises(ValueError):
            decode_rgb(np.zeros((10, 10, 3), np.uint8), upscale=3)

    def test_render_warmup_waits_for_content(self):
        from barcode_sweep import wait_for_render
        calls = []
        def render():
            calls.append(1)
            rgb = np.zeros((10,10,3),np.uint8)
            if len(calls) >= 6:
                rgb[5,5] = 100
            return rgb, None
        self.assertEqual(wait_for_render(render),6)

    def test_render_warmup_rejects_black_buffer(self):
        from barcode_sweep import wait_for_render
        with self.assertRaises(RuntimeError):
            wait_for_render(lambda: (np.zeros((10,10,3),np.uint8),None), maximum=8)

    def test_empty_frame(self):
        self.assertEqual(decode_rgb(np.full((480, 640, 3), 255, np.uint8))['reads'], [])

    def test_reject_bad_dtype(self):
        with self.assertRaises(ValueError):
            decode_rgb(np.zeros((10, 10, 3), np.float32))

    def test_reject_bad_shape(self):
        with self.assertRaises(ValueError):
            decode_rgb(np.zeros((10, 10), np.uint8))

    def test_stability_and_missing_reset(self):
        tracker = StableCode()
        reads = [dict(text='P2-01')]
        self.assertIsNone(tracker.observe(reads))
        self.assertIsNone(tracker.observe(reads))
        self.assertEqual(tracker.observe(reads), 'P2-01')
        self.assertIsNone(tracker.observe([]))
        self.assertIsNone(tracker.observe(reads))

    def test_conflicting_and_unknown_clear_stability(self):
        tracker = StableCode(required=1)
        self.assertIsNone(tracker.observe([dict(text='P2-01'), dict(text='P2-02')]))
        self.assertIsNone(tracker.observe([dict(text='P2-99')]))
        self.assertEqual(tracker.observe([dict(text='P2-04')]), 'P2-04')

    def test_code_change_restarts_count(self):
        tracker = StableCode()
        tracker.observe([dict(text='P2-01')])
        tracker.observe([dict(text='P2-01')])
        self.assertIsNone(tracker.observe([dict(text='P2-02')]))

    def test_true_dimensions_and_level_mount(self):
        corners = label_corners([.15, 1.03, .14])
        self.assertAlmostEqual(np.linalg.norm(corners[1]-corners[0]), .075)
        self.assertAlmostEqual(np.linalg.norm(corners[3]-corners[0]), .025)
        self.assertEqual(corners[0, 2], corners[1, 2])

    def test_projection_diagnostic(self):
        center = np.array([.15, 1.03, .14])
        T = look_at(center + [.2, 0, 0], center)
        projection = projected_bars_px(T, K, center)
        self.assertAlmostEqual(projection['width_px'], 120.)
        self.assertAlmostEqual(projection['height_px'], 30.)

    def test_behind_camera(self):
        center = np.array([.15, 1.03, .14])
        T = look_at(center + [.2, 0, 0], center + [1., 0, 0])
        self.assertIsNone(projected_bars_px(T, K, center))


if __name__ == '__main__':
    unittest.main()
