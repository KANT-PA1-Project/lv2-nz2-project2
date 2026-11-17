"""ROS 독립 어댑터. 검출·12회 기록 규칙은 tower_signal 원본을 재사용한다."""
import importlib
import importlib.util
import math
import os
from pathlib import Path
import sys
from types import SimpleNamespace

import cv2


def load_runtime(root, r12_src=""):
    root = Path(root).expanduser().resolve()
    if not (root / "scripts/live_mission_optimized.py").is_file():
        raise ValueError(f"perception_root에 scripts/live_mission_optimized.py가 필요합니다: {root}")
    if r12_src:
        os.environ["R12_SRC"] = str(Path(r12_src).expanduser().resolve())
    sys.path.insert(0, str(root / "scripts"))
    # 서로 다른 작업 사본의 전역 모듈을 한 프로세스에서 섞지 않는다.
    for name in ("tower_live_hsv", "mission_summary", "barcode_support"):
        previous = sys.modules.get(name)
        if previous is not None and Path(previous.__file__).resolve().parent != root / "scripts":
            raise RuntimeError(f"다른 작업 사본의 {name}이 이미 로드됐습니다")
    spec = importlib.util.spec_from_file_location("_ros_barcode_signal_runtime", root / "scripts/live_mission_optimized.py")
    live = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(live)
    mission = importlib.import_module("mission_summary")
    # 의존성 오류는 카메라 처리 전에 알린다. vendor 또는 requirements-barcode.txt 설치본 사용.
    importlib.import_module("barcode_support")
    return live, mission.MissionSession


class BarcodeSignalCore:
    """자세 제어를 하지 않는다. prepare_signal 호출자는 정지·소등을 외부에서 보장한다."""
    def __init__(self, live, session_class, *, fixed_settings=False, search_hz=10.,
                 signal_width=640, auto_retry=True, exposure_token="fixed_by_camera_driver"):
        self.live, self.session_class = live, session_class
        self.rule = live.load_color_rule()
        self.fixed_settings = fixed_settings
        self.search_hz, self.signal_width = search_hz, signal_width
        self.auto_retry, self.exposure_token = auto_retry, exposure_token
        self.generation = 0
        self.reset()

    def reset(self):
        self.generation += 1
        self.session = self.session_class()
        self.phase, self.requested_view = "idle", "SIGNAL_VIEW"
        self.patch = self.preparation = None
        self.signal_active = self.barcode_active = False
        self.first_flash_s = self.last_signal_s = None
        self.camera_key = None
        self.raw_color, self.visible, self.reason = "UNKNOWN", False, "waiting for an explicit view command"
        self.live.finder.clear_tiers()
        self._summary_key = self._summary = None

    def _clear_partial(self):
        code = self.session.barcode
        self.session = self.session_class()
        self.session.barcode = code
        if code is not None:
            self.session.candidate, self.session.matches = code, 3
        self.first_flash_s = self.last_signal_s = None
        self.patch, self.camera_key = None, None
        self.generation += 1
        self.live.finder.clear_tiers()

    def _new_preparation(self, recovering=False):
        args = SimpleNamespace(auto_start=True, prepare_timeout=self.live.PREPARE_TIMEOUT_S,
                               search_hz=self.search_hz, exposure=self.exposure_token, wb=self.exposure_token)
        self.preparation = self.live.Preparation(self.rule, args, self.generation,
                            stable_off_s=self.live.RETRY_OFF_S if recovering else self.live.STABLE_OFF_S)

    def prepare_signal(self):
        if self.signal_active:
            return False, "Signal observation already active"
        if self.session.recorder.complete:
            return False, "Signal already saved; reset is required for a new round"
        if not self.fixed_settings:
            return False, "fixed_camera_settings_confirmed must be true after configuring the camera driver"
        if self.session.error:
            return False, "Reset the error before preparing a new round"
        self._clear_partial()
        self.barcode_active, self.signal_active = False, True
        self.phase, self.requested_view = "preparing_signal", "SIGNAL_VIEW"
        self.raw_color, self.visible = "UNKNOWN", False
        self.reason = "Caller guarantees stationary camera and tower OFF until READY"
        self._new_preparation()
        return True, self.reason

    def scan_barcode(self):
        if self.signal_active:
            return False, "Pause before moving; an active signal observation cannot be interrupted by barcode scanning"
        if self.session.error:
            return False, "Reset the error before scanning"
        if self.session.barcode is not None:
            return False, "Barcode already saved; reset is required to read a different mission barcode"
        self.generation += 1
        self.barcode_active = True
        self.phase, self.requested_view = "scanning_barcode", "BARCODE_VIEW"
        self.reason = "Caller confirms arrival at a stationary barcode view"
        return True, self.reason

    def pause(self):
        if self.signal_active and not self.session.recorder.complete:
            self._clear_partial()  # 이동 전후의 부분 점멸 기록을 연결하지 않는다.
        else:
            self.generation += 1
        self.signal_active = self.barcode_active = False
        self.patch = self.preparation = None
        self.phase = "paused"
        self.raw_color, self.visible, self.reason = "UNKNOWN", False, "Paused for motion; completed records retained"
        self.live.finder.clear_tiers()
        return True, self.reason

    def signal_fault(self, reason):
        if not self.signal_active or self.session.recorder.complete:
            return
        self.raw_color, self.visible = "UNKNOWN", False
        if self.auto_retry:
            self._clear_partial()
            self._new_preparation(recovering=True)
            self.phase = "recovering_signal"
            self.reason = f"{reason}; partial sequence discarded, waiting for 2 s visible OFF"
        else:
            self.session.fail(reason)
            self.signal_active = False
            self.phase, self.reason = "error", str(reason)

    def feed_signal(self, bgr, stamp_s, frame_id):
        if not self.signal_active or self.session.recorder.complete:
            return
        if not math.isfinite(stamp_s):
            self.signal_fault("Non-finite image timestamp")
            return
        key = (str(frame_id), tuple(bgr.shape))
        if self.camera_key is not None and key != self.camera_key:
            self.signal_fault("Camera frame or image dimensions changed")
        if self.last_signal_s is not None and not 0 < stamp_s-self.last_signal_s <= self.live.MAX_FRAME_GAP_S:
            self.signal_fault("Nonmonotonic or missing signal images")
        if not self.signal_active:
            return
        self.camera_key, self.last_signal_s = key, stamp_s
        scale = min(1., self.signal_width/bgr.shape[1])
        work = cv2.resize(bgr, (round(bgr.shape[1]*scale), round(bgr.shape[0]*scale)),
                          interpolation=cv2.INTER_AREA) if scale < 1 else bgr
        if self.patch is None:
            try:
                box, acquired = self.preparation.step(work, stamp_s)
            except ValueError as exc:
                self.signal_fault(str(exc))
                return
            self.visible = box is not None
            self.raw_color, self.reason = "UNKNOWN", self.preparation.reason
            if acquired is None:
                return
            patch, evidence = acquired
            recovery = self.phase == "recovering_signal"
            required = self.live.RETRY_OFF_S if recovery else self.live.STABLE_OFF_S
            if evidence.get("stable_off_s", 0) < required or evidence.get("baseline_frames", 0) < self.live.BASELINE_FRAMES:
                self.signal_fault("Incomplete OFF preparation evidence")
                return
            if not self.session.arm(stamp_s, "OFF", confirmed_off=True, fixed_settings=self.fixed_settings):
                self.signal_fault("Signal recorder refused preparation")
                return
            self.patch = patch
            self.phase, self.raw_color, self.visible, self.reason = "recording_signal", "OFF", True, "READY"
            return
        reading = self.patch.read_pixels(self.patch.pixels(work))
        self.raw_color = reading["color"]
        self.visible = reading.get("reason") != "fixed_roi_body_lost"
        self.reason = str(reading.get("reason") or reading.get("decision") or self.raw_color)
        if self.raw_color in ("R", "Y", "G") and self.first_flash_s is None:
            self.first_flash_s = stamp_s
        self.session.observe(stamp_s, self.raw_color)
        if self.session.error:
            self.signal_fault(self.session.error)
            return
        if self.first_flash_s is not None and stamp_s-self.first_flash_s >= self.live.SIGNAL_CYCLE_TIMEOUT_S and not self.session.recorder.complete:
            self.signal_fault("Signal cycle timed out before valid 12-flash completion")
            return
        if self.session.recorder.complete:
            self.signal_active = False
            self.patch = self.preparation = None
            self.live.finder.clear_tiers()
            self.phase = "ready" if self.session.barcode is not None else "waiting_barcode_view"
            self.requested_view = "NONE" if self.session.barcode is not None else "BARCODE_VIEW"
            self.reason = "12 flashes saved; camera may move"

    def feed_barcodes(self, reads, stamp_s):
        if not self.barcode_active or self.session.error:
            return
        self.session.observe_barcodes(reads, time_s=stamp_s)
        if self.session.barcode is not None:
            self.barcode_active = False
            self.phase = "ready" if self.session.recorder.complete else "waiting_signal_view"
            self.requested_view = "NONE" if self.session.recorder.complete else "SIGNAL_VIEW"
            self.reason = "Barcode saved; decoding stopped"

    def summary(self):
        key = (self.generation, self.session.armed, self.session.barcode, self.session.error,
               self.session.recorder.complete, tuple(self.session.recorder.sequence))
        if key != self._summary_key:
            self._summary_key, self._summary = key, self.session.snapshot()
        return self._summary
