"""Camera-only HSV mission check; no map pose, Isaac or robot required.

Manual: o -> lock -> m -> start flashes. Automatic: --auto-start, keep OFF
until READY. That flag declares a prestart-OFF protocol, not visual certainty.
This diagnostic uses full-frame HSV acquisition, distinct from the projected
search path in live_tower_check.py. It is not tutor integration validation.
"""
import argparse
from pathlib import Path
import time
import cv2
import numpy as np

from live_tower_check import load, ts
from live_signal_guard import body_present

HERE = Path(__file__).resolve().parents[1]
WIDTH, HEIGHT, FPS = 640, 480, 30
FRAME_TIMEOUT_MS = 2000
import tower_live_hsv as hsv_finder  # camera-only acquisition bands
COLORS = dict(R=(0, 0, 255), Y=(0, 220, 255), G=(0, 200, 0),
              OFF=(200, 200, 200), UNKNOWN=(255, 0, 255))


def tier_boxes_from_lens_box(box, shape):
    x0, y0, x1, y1 = map(float, box)
    w, h = x1-x0, (y1-y0)/3
    boxes = {}
    for i, c in enumerate("RYG"):
        b = (int(np.floor(x0+w*.2)), int(np.floor(y0+i*h+h*.2)),
             int(np.ceil(x1-w*.2)), int(np.ceil(y0+(i+1)*h-h*.2)))
        if b[0] < 0 or b[1] < 0 or b[2] > shape[1] or b[3] > shape[0] or b[2] <= b[0] or b[3] <= b[1]:
            return None
        boxes[c] = (*b, 0.)
    return boxes


class Locker:
    def __init__(self, auto=False, auto_frames=30, exposure_token=""):
        self.auto, self.auto_frames, self.exposure_token = auto, auto_frames, exposure_token
        self.reset()

    def reset(self):
        self.boxes = self.ref = self.key = None
        self.frames, self.lens_boxes = [], []
        self.manual = self.confirmed_off = False
        self.confirmation_source = None
        self.last_reason = "tower OFF -> press o"

    @property
    def locked(self):
        return self.ref is not None

    def start_manual(self):
        self.frames, self.lens_boxes, self.manual = [], [], True

    def feed(self, bgr):
        box = hsv_finder.detect(bgr)
        if box is None:
            self.frames, self.lens_boxes = [], []
            self.last_reason = "tower not found"
            return None
        rgb = bgr[:, :, ::-1]
        boxes = tier_boxes_from_lens_box(box, bgr.shape)
        if boxes is None:
            self.last_reason = "invalid lens box"
            return box
        if not self.manual:
            if not self.auto or ts.color_from_boxes(rgb, boxes)["color"] != "OFF":
                self.frames, self.lens_boxes = [], []
                return box
        if self.lens_boxes and np.abs(np.subtract(box, self.lens_boxes[0])).max() > 3:
            self.frames, self.lens_boxes = [], []
        self.frames.append(rgb.copy())
        self.lens_boxes.append(box)
        need = 5 if self.manual else self.auto_frames
        self.last_reason = f"collecting {len(self.frames)}/{need}"
        if len(self.frames) >= need:
            source = "operator_o" if self.manual else "visual_candidate"
            if self.lock_reference(self.frames, self.lens_boxes, source=source):
                self.last_reason = "locked; verify OFF, then press m"
                print("[live]", self.last_reason)
            self.frames, self.lens_boxes, self.manual = [], [], False
        return box

    def lock_reference(self, frames, lens_boxes, *, source):
        """OFF certainty comes from the operator or prestart protocol, not HSV."""
        boxes = tier_boxes_from_lens_box(np.median(lens_boxes, axis=0), frames[0].shape)
        if boxes is None:
            return False
        self.key = f"live:{self.exposure_token}:{boxes}"
        self.ref = ts.make_off_reference(frames[-5:], boxes, context_key=self.key, confirmed_off=True)
        self.boxes = boxes
        self.confirmed_off = source in ("operator_o", "prestart_off_contract")
        self.confirmation_source = source
        self.last_reason = "locked; OFF reference ready"
        return True

    def read(self, rgb):
        res = ts.color_from_boxes(rgb, self.boxes, off_reference=self.ref, context_key=self.key)
        if not body_present(rgb, self.boxes):
            res.update(color="UNKNOWN", reason="fixed_roi_body_lost")
        return res


class AutoPreparation:
    """Acquisition under an explicit prestart-OFF contract; never infer certainty.

    Uses camera timestamps, >=1s continuous candidate OFF, <=3px box drift,
    <=0.25s frame gaps, and five baseline frames. First flash must follow READY.
    """
    def __init__(self, off_seconds=1., timeout=30.):
        self.off_seconds, self.timeout = off_seconds, timeout
        self.started = self.last_time = None
        self.clear_candidate()

    def clear_candidate(self):
        self.since = self.first_box = None
        self.frames, self.boxes = [], []
        self.reason = "AUTO: searching for unlit tower"

    def feed(self, bgr, t_s, locker):
        if not np.isfinite(t_s):
            raise ValueError("Automatic preparation: invalid timestamp")
        if self.started is None:
            self.started = t_s
        if t_s-self.started > self.timeout:
            raise ValueError("Automatic preparation timed out; no READY issued")
        if self.last_time is not None and not 0 < t_s-self.last_time <= .25:
            self.clear_candidate()
            if t_s <= self.last_time:
                raise ValueError("Automatic preparation: nonmonotonic timestamp")
        self.last_time = t_s
        box = hsv_finder.detect(bgr)
        if box is None:
            self.clear_candidate()
            return None, None
        boxes = tier_boxes_from_lens_box(box, bgr.shape)
        rgb = bgr[:, :, ::-1]
        if boxes is None or not body_present(rgb, boxes) or ts.color_from_boxes(rgb, boxes)["color"] != "OFF":
            self.clear_candidate()
            self.reason = "AUTO: waiting for unlit lenses"
            return box, None
        if self.first_box is not None and np.abs(np.subtract(box, self.first_box)).max() > 3:
            self.clear_candidate()
        if self.since is None:
            self.since, self.first_box = t_s, box
        self.frames.append(rgb.copy())
        self.boxes.append(box)
        self.frames, self.boxes = self.frames[-5:], self.boxes[-5:]
        elapsed = t_s-self.since
        self.reason = f"AUTO: stable OFF candidate {elapsed:.1f}/{self.off_seconds:.1f}s"
        if elapsed < self.off_seconds or len(self.frames) < 5:
            return box, None
        if not locker.lock_reference(self.frames, self.boxes, source="prestart_off_contract"):
            self.clear_candidate()
            return box, None
        if locker.read(rgb)["color"] != "OFF":
            locker.reset()
            self.clear_candidate()
            return box, None
        evidence = dict(prestart_off_guaranteed=True, stable_off_s=elapsed,
                        baseline_frames=len(self.frames), max_jitter_px=3)
        return box, evidence


def warmup_camera(pipe, *, budget_s=30., frame_count=30):
    """Bounded startup only. Never retries or skips frames after mission arm."""
    deadline = time.monotonic() + budget_s
    received, timeouts = 0, 0
    print(f"[live] Waiting for RGB startup: budget {budget_s:g}s", flush=True)
    while received < frame_count:
        remaining_ms = int((deadline-time.monotonic())*1000)
        if remaining_ms <= 0:
            raise RuntimeError(
                f"RGB startup timeout: {received}/{frame_count} frames in {budget_s:g}s. "
                "Close other camera applications, reconnect D435 directly using a USB 3 data cable.")
        try:
            frames = pipe.wait_for_frames(min(10000, remaining_ms))
        except RuntimeError as exc:
            if "Frame didn't arrive" not in str(exc):
                raise
            timeouts += 1
            print(f"[live] Startup wait timeout #{timeouts}; {received}/{frame_count} frames", flush=True)
            continue
        if not frames.get_color_frame():
            continue
        received += 1
        if received == 1:
            print("[live] First color frame received", flush=True)
    print(f"[live] Startup ready: {received} frames, {timeouts} wait timeouts", flush=True)


def main():
    import pyrealsense2 as rs
    from mission_live_support import MissionLive
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mission", action="store_true", help="호환 옵션; 미션 요약은 기본 활성화")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--no-auto", action="store_true", help="manual OFF acquisition with o/m")
    mode.add_argument("--auto-start", action="store_true", help="operator guarantees OFF and a fixed camera until READY; auto acquire and arm")
    parser.add_argument("--prepare-timeout", type=float, default=30., help="automatic preparation deadline in seconds")
    parser.add_argument("--exposure", type=float, default=150.)
    parser.add_argument("--wb", type=float, default=4600.)
    parser.add_argument("--serial")
    parser.add_argument("--out", default=str(HERE / "reports/live-mission"))
    parser.add_argument("--barcode-upscale", type=int, choices=(1, 2), default=1)
    parser.add_argument("--startup-timeout", type=float, default=30., help="initial RGB warmup budget in seconds")
    parser.add_argument("--max-frames", type=int, help="stop after N frames; diagnostic only")
    parser.add_argument("--no-gui", action="store_true", help="camera startup check; no keyboard arm")
    args = parser.parse_args()
    if args.max_frames is not None and args.max_frames < 1:
        parser.error("--max-frames must be positive")
    if not np.isfinite(args.startup_timeout) or not 1 <= args.startup_timeout <= 60:
        parser.error("--startup-timeout must be 1..60 seconds")
    if not np.isfinite(args.prepare_timeout) or not 1 <= args.prepare_timeout <= 300:
        parser.error("--prepare-timeout must be 1..300 seconds")
    pipe, cfg = rs.pipeline(), rs.config()
    mission, writer = None, None
    error, started = None, False
    locker = Locker(auto=False, exposure_token=f"exp={args.exposure},wb={args.wb}")
    preparation = AutoPreparation(timeout=args.prepare_timeout)
    finished = False
    signal_finished = False
    if args.serial:
        cfg.enable_device(args.serial)
    cfg.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
    try:
        profile = pipe.start(cfg)
        started = True
        device = profile.get_device()
        if hasattr(rs.camera_info, "usb_type_descriptor") and device.supports(rs.camera_info.usb_type_descriptor):
            print("[live] USB connection:", device.get_info(rs.camera_info.usb_type_descriptor), flush=True)
        sensor = next(s for s in device.query_sensors()
                      if s.get_info(rs.camera_info.name) == "RGB Camera")
        settings = [(rs.option.enable_auto_exposure, 0), (rs.option.exposure, args.exposure),
                    (rs.option.enable_auto_white_balance, 0), (rs.option.white_balance, args.wb)]
        for option, value in settings:
            sensor.set_option(option, value)
        fixed = all(abs(sensor.get_option(option)-value) < 1 for option, value in settings)
        if not fixed:
            raise RuntimeError("Manual exposure/WB readback mismatch")
        warmup_camera(pipe, budget_s=args.startup_timeout)
        mission = MissionLive(args.out, upscale=args.barcode_upscale)
        print("[live] D435 started 640x480; o=confirmed OFF lock, m=arm, r=reset, v=video, q=quit")
        if args.auto_start:
            print("[live] AUTO: keep tower OFF and camera fixed until READY; no o/m keys required", flush=True)
        index = 0
        while True:
            frame = pipe.wait_for_frames(FRAME_TIMEOUT_MS).get_color_frame()
            if not frame:
                continue
            bgr = np.asanyarray(frame.get_data()).copy()
            rgb, t_s = bgr[:, :, ::-1], frame.get_timestamp()/1000.
            view, color = bgr.copy(), None
            auto_evidence = None
            if finished:
                line = "MISSION REQUIREMENTS READY; tower observation stopped"
            elif signal_finished:
                line = "SIGNAL SAVED; waiting for barcode; camera may move"
            elif locker.locked:
                result = locker.read(rgb)
                color = result["color"]
                for c, (x0, y0, x1, y1, _) in locker.boxes.items():
                    cv2.rectangle(view, (x0, y0), (x1, y1), COLORS[c], 2)
                line = f"{color} {result.get('reason') or result.get('decision')}"
            elif args.auto_start and not mission.session.armed and not mission.session.error:
                try:
                    box, auto_evidence = preparation.feed(bgr, t_s, locker)
                except ValueError as exc:
                    mission.session.fail(exc)
                    box = None
                if box is not None:
                    cv2.rectangle(view, tuple(map(int, box[:2])), tuple(map(int, box[2:])), (255, 0, 255), 2)
                color = "OFF" if auto_evidence is not None else None
                line = preparation.reason
            else:
                box = locker.feed(bgr)
                if box is not None:
                    cv2.rectangle(view, tuple(map(int, box[:2])), tuple(map(int, box[2:])), (255, 0, 255), 2)
                line = locker.last_reason
            snapshot = mission.session.snapshot() if finished else mission.process(rgb, t_s, color)
            if auto_evidence is not None and not mission.session.error:
                accepted = mission.arm(
                    t_s, color, confirmed_off=locker.confirmed_off, fixed_settings=fixed,
                    confirmation_source="prestart_off_contract", preparation=auto_evidence)
                if accepted:
                    print("[live] READY: automatic preparation complete; operator may start signal", flush=True)
                else:
                    mission.session.fail("Automatic arm rejected; no READY issued")
                snapshot = mission.publish()
            if snapshot["signal_complete"] and not snapshot["error"] and not signal_finished:
                signal_finished = True
                locker.reset()
                preparation.clear_candidate()
                view = bgr.copy()
                line = "SIGNAL SAVED; waiting for barcode; camera may move"
                print("[live] SIGNAL COMPLETE: tower boxes/reference cleared; barcode-only mode", flush=True)
            if snapshot["status"] == "ready" and not finished:
                finished = True
                locker.reset()
                view = bgr.copy()
                line = "MISSION REQUIREMENTS READY; tower observation stopped"
                print("[live] COMPLETE: requirements saved; tower acquisition and reading stopped", flush=True)
            lines = [line, f"{snapshot['status']} {snapshot['barcode']} {snapshot['direction']}",
                     f"{len(snapshot['sequence'])}/12 " + "".join(snapshot["sequence"]),
                     snapshot["error"] or " ".join(f"{x['location']}={x['group']}:{x['count']}"
                                                  for x in snapshot["locations"])]
            if writer is not None:
                writer.write(bgr)
            index += 1
            if args.no_gui:
                if index == 1:
                    print(f"[live] First RGB frame: {rgb.shape}, timestamp={t_s:.3f}s")
                key = 255
            else:
                for i, text in enumerate(lines):
                    cv2.putText(view, text, (10, 25+i*24), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 0), 3)
                    cv2.putText(view, text, (10, 25+i*24), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
                cv2.imshow("tower mission (camera-only HSV)", view)
                key = cv2.waitKey(1) & 255
            if key in (ord("q"), 27) or (args.max_frames is not None and index >= args.max_frames):
                break
            if key == ord("o") and not locker.locked and not args.auto_start and not signal_finished and not finished:
                locker.start_manual()
                print("[live] Operator confirms OFF; collecting reference")
            elif key == ord("m") and not args.auto_start and not signal_finished and not finished:
                accepted = mission.arm(t_s, color, confirmed_off=locker.confirmed_off, fixed_settings=fixed)
                print("[live]", "ARMED: operator may start signal" if accepted else "ARM REJECTED: confirm OFF with o first")
            elif key == ord("r"):
                locker.reset()
                mission.reset()
                preparation = AutoPreparation(timeout=args.prepare_timeout)
                finished = False
                signal_finished = False
            elif key == ord("v"):
                if writer is not None:
                    writer.release()
                    writer = None
                else:
                    path = Path(args.out) / f"camera_{time.time_ns()}.mp4"
                    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), FPS, (WIDTH, HEIGHT))
                    if not writer.isOpened():
                        writer.release()
                        writer = None
                        print("[live] Video writer failed")
                    else:
                        print("[live] Video:", path)
        print(f"[live] Stopped cleanly after {index} frames")
    except BaseException as exc:
        error = f"Camera/live failure: {exc}"
        raise
    finally:
        if mission is not None:
            mission.close(None if finished else error)
        if writer is not None:
            writer.release()
        if started:
            pipe.stop()
        if not args.no_gui:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
