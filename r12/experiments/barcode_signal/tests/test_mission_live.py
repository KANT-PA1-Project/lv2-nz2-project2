"""Mock camera and GUI only. No hardware or Isaac is started."""
from concurrent.futures import Future
import itertools
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import live_tower_mission as live
import mission_live_support as adapter
from replay_mission_summary import replay

SEQ = "RGYRGYGRGRGG"


class ImmediatePool:
    def __init__(self, **kwargs):
        pass

    def submit(self, function, *args, **kwargs):
        future = Future()
        try:
            future.set_result(function(*args, **kwargs))
        except Exception as exc:
            future.set_exception(exc)
        return future

    def shutdown(self, **kwargs):
        pass


class MockLiveTests(unittest.TestCase):
    def run_camera(self, output, lose_target=False):
        pipe = MagicMock()
        sensor = MagicMock()
        values = dict(enable_auto_exposure=0, enable_auto_white_balance=0,
                      exposure=150., white_balance=4600.)
        sensor.get_info.return_value = "RGB Camera"
        sensor.get_option.side_effect = values.__getitem__
        dev = MagicMock()
        dev.query_sensors.return_value = [sensor]
        profile = MagicMock()
        profile.get_device.return_value = dev
        profile.get_stream.return_value.as_video_stream_profile.return_value.get_intrinsics.return_value = SimpleNamespace(
            fx=400., fy=400., ppx=320., ppy=240.)
        pipe.start.return_value = profile
        camera = SimpleNamespace(index=-30)

        def frame():
            camera.index += 1
            rgb = np.zeros((480, 640, 3), np.uint8)
            for i, color in enumerate([(120, 0, 0), (140, 100, 0), (0, 100, 50)]):
                rgb[20+i*60:80+i*60, 20:80] = color
            i = camera.index-8
            if 0 <= i < 480 and i % 40 < 20:
                color = SEQ[i//40]
                tier = "RYG".index(color)
                rgb[20+tier*60:80+tier*60, 20:80] = 255
            if lose_target and camera.index >= 20:
                rgb[:] = 0
            color_frame = SimpleNamespace(get_data=lambda: rgb[:, :, ::-1].copy(),
                                          get_timestamp=lambda: camera.index*50.)
            return SimpleNamespace(get_color_frame=lambda: color_frame)

        pipe.wait_for_frames.side_effect = lambda *args: frame()
        keys = lambda delay: ord("o") if camera.index == 1 else (
            ord("m") if camera.index == 7 else ord("q") if camera.index >= 529 else 255)
        option = SimpleNamespace(**{k:k for k in values})
        rs = SimpleNamespace(pipeline=lambda:pipe, config=MagicMock(),
                             stream=SimpleNamespace(color=1), format=SimpleNamespace(bgr8=1),
                             option=option, camera_info=SimpleNamespace(name=1, serial_number=2))
        clock = itertools.count(0, .3)
        with patch.dict(sys.modules, pyrealsense2=rs),              patch.object(sys, "argv", ["live", "--mission", "--no-auto", "--exposure", "150",
                                       "--wb", "4600", "--out", str(output)]),              patch.object(live.hsv_finder, "detect", return_value=(20, 20, 80, 200)),              patch.object(live.cv2, "imshow"), patch.object(live.cv2, "destroyAllWindows"),              patch.object(live.cv2, "waitKey", side_effect=keys),              patch.object(adapter, "ThreadPoolExecutor", ImmediatePool),              patch.object(adapter, "decode_rgb", return_value=dict(reads=[dict(text="P2-01")], decode_ms=1.)),              patch.object(adapter.time, "monotonic", side_effect=lambda:next(clock)),              patch("builtins.print"):
            live.main()
        pipe.stop.assert_called_once()
        summary = json.loads((output / "mission_summary.json").read_text())
        log = next(output.glob("mission_*.jsonl"))
        reproduced = replay(json.loads(line) for line in log.read_text().splitlines())
        self.assertEqual(summary, reproduced)
        return summary

    def test_full_live_loop_with_mock_camera(self):
        with tempfile.TemporaryDirectory() as directory:
            result = self.run_camera(Path(directory))
            self.assertEqual(result["status"], "ready")
            self.assertEqual(result["sequence"], list(SEQ))
            self.assertEqual([(x["location"], x["group"], x["count"]) for x in result["locations"]],
                             [("L1", "Y", 2), ("L2", "R", 4), ("L3", "G", 6)])

    def test_target_disappearance_invalidates_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            result = self.run_camera(Path(directory), lose_target=True)
            self.assertEqual(result["status"], "error")
            self.assertEqual(result["locations"], [])

    def test_auto_lock_does_not_claim_operator_confirmation(self):
        image = np.zeros((480, 640, 3), np.uint8)
        for i, color in enumerate([(0, 0, 120), (0, 100, 140), (50, 100, 0)]):
            image[20+i*60:80+i*60, 20:80] = color
        locker = live.Locker(auto=True, auto_frames=3)
        with patch.object(live.hsv_finder, "detect", return_value=(20, 20, 80, 200)), patch("builtins.print"):
            for _ in range(3):
                locker.feed(image)
        self.assertTrue(locker.locked)
        self.assertFalse(locker.confirmed_off)
