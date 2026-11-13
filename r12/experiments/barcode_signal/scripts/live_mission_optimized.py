"""HSV/Code128 mission checker; no model, YOLO, torch or CUDA dependency.

--auto-start declares that the operator keeps the tower OFF and camera fixed
until READY. Image darkness alone is not proof of OFF. Record 12 flashes plus
3 seconds final OFF, release the tower ROI, then finish barcode decoding.
"""
import argparse
from collections import deque
import importlib.util
import hashlib
import json
from pathlib import Path
import time

import cv2
import numpy as np

HERE = Path(__file__).resolve().parents[1]

# Runtime defaults. Capture resolution may be increased for barcode detail.
CAPTURE_WIDTH, CAPTURE_HEIGHT, CAPTURE_FPS = 640, 480, 30
SIGNAL_WIDTH = 640
SEARCH_HZ, DISPLAY_HZ, BARCODE_HZ = 10., 10., 5.
STABLE_OFF_S, PREPARE_TIMEOUT_S = 1., 30.
BASELINE_FRAMES, JITTER_PX, MAX_FRAME_GAP_S = 5, 3, .25
STARTUP_TIMEOUT_S, FRAME_TIMEOUT_MS = 30., 2000
FULL_SCAN_EVERY_S = 2.
# Normal OFF is 1 s; use 2 s to distinguish the longer cycle boundary.
RETRY_OFF_S = 2.0
# Conservative watchdog after the first observed flash, not after READY.
SIGNAL_CYCLE_TIMEOUT_S = 12 * (1. + 1.) + 3.
COLORS = {"R": (0, 0, 255), "Y": (0, 220, 255), "G": (0, 200, 0)}
import tower_live_hsv as finder


def load_color_rule():
    path = HERE / "src/arm_practice/tower_signal.py"
    spec = importlib.util.spec_from_file_location("_optimized_tower_color", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def lens_boxes(box, shape):
    x0, y0, x1, y1 = map(float, box)
    w, h = x1-x0, (y1-y0)/3.
    result = {}
    for i, c in enumerate("RYG"):
        b = (int(np.floor(x0+.2*w)), int(np.floor(y0+(i+.2)*h)),
             int(np.ceil(x1-.2*w)), int(np.ceil(y0+(i+.8)*h)))
        if b[0] < 0 or b[1] < 0 or b[2] > shape[1] or b[3] > shape[0] or b[2] <= b[0] or b[3] <= b[1]:
            return None
        result[c] = (*b, 0.)
    return result


class SignalPatch:
    """Store only a small fixed pixel crop, with crop-relative ROI identity."""
    def __init__(self, rule, box, shape, context):
        self.rule = rule
        self.boxes = lens_boxes(box, shape)
        if self.boxes is None:
            raise ValueError("Invalid lens box")
        self.bounds = (min(b[0] for b in self.boxes.values()),
                       min(b[1] for b in self.boxes.values()),
                       max(b[2] for b in self.boxes.values()),
                       max(b[3] for b in self.boxes.values()))
        x, y, _, _ = self.bounds
        self.local = {c: (a-x, b-y, d-x, e-y, z) for c, (a, b, d, e, z) in self.boxes.items()}
        self.context = f"optimized:{context}:{self.bounds}:{self.local}"
        self.reference = None

    def pixels(self, bgr):
        x0, y0, x1, y1 = self.bounds
        return cv2.cvtColor(bgr[y0:y1, x0:x1], cv2.COLOR_BGR2RGB)

    def present(self, rgb):
        # One HSV conversion for all three lens interiors.
        evidence = finder.masks(cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV))
        support = [float(evidence[c][y0:y1, x0:x1].mean())
                   for c, (x0, y0, x1, y1, _) in self.local.items()]
        return sum(x >= .35 for x in support) >= 2

    def read_pixels(self, rgb):
        if not self.present(rgb):
            return {"color": "UNKNOWN", "reason": "fixed_roi_body_lost"}
        return self.rule.color_from_boxes(
            rgb, self.local, off_reference=self.reference,
            context_key=self.context if self.reference is not None else None)

    def lock(self, frames):
        self.reference = self.rule.make_off_reference(
            list(frames), self.local, context_key=self.context, confirmed_off=True)


class Preparation:
    """Fresh location checks at search_hz; OFF/presence checks every frame."""
    def __init__(self, rule, args, epoch=0, stable_off_s=STABLE_OFF_S):
        self.rule, self.args, self.epoch = rule, args, epoch
        self.stable_off_s = stable_off_s
        self.started = self.last_time = self.last_search = None
        self.last_box = None
        self.manual_requested = False
        self.clear()

    def clear(self):
        self.patch = self.anchor = self.since = None
        self.frames = deque(maxlen=BASELINE_FRAMES)
        self.reason = "searching; keep tower OFF"

    def step(self, bgr, t_s):
        if not np.isfinite(t_s):
            raise ValueError("Invalid camera timestamp")
        if self.started is None:
            self.started = t_s
        if self.last_time is not None and not 0 < t_s-self.last_time <= MAX_FRAME_GAP_S:
            self.clear()
            self.last_box = None
            self.last_search = None
            if t_s <= self.last_time:
                raise ValueError("Nonmonotonic camera timestamp")
        self.last_time = t_s
        if self.args.auto_start and t_s-self.started > self.args.prepare_timeout:
            raise ValueError("Automatic preparation timeout; no READY issued")
        if self.last_search is None or t_s-self.last_search >= 1/self.args.search_hz:
            self.last_box = finder.detect(bgr)
            self.last_search = t_s
        box = self.last_box
        if box is None or t_s-self.last_search > MAX_FRAME_GAP_S:
            self.clear()
            return box, None
        if not self.args.auto_start and not self.manual_requested:
            self.reason = "tower found; confirm OFF and press o"
            return box, None
        if self.anchor is not None and np.abs(np.subtract(box, self.anchor)).max() > JITTER_PX:
            self.clear()
        if self.patch is None:
            self.anchor = tuple(box)
            self.patch = SignalPatch(
                self.rule, box, bgr.shape,
                f"{self.epoch}:exp={self.args.exposure}:wb={self.args.wb}")
        rgb = self.patch.pixels(bgr)
        if self.patch.read_pixels(rgb)["color"] != "OFF":
            self.clear()
            self.reason = "waiting for unlit lens candidate"
            return box, None
        if self.since is None:
            self.since = t_s
        self.frames.append(rgb)  # RGB is already a small independent crop.
        elapsed = t_s-self.since
        duration = self.stable_off_s if self.args.auto_start else 0.
        self.reason = f"OFF candidate {elapsed:.1f}/{duration:.1f}s; {len(self.frames)}/{BASELINE_FRAMES}"
        if len(self.frames) < BASELINE_FRAMES or elapsed < duration:
            return box, None
        self.patch.lock(self.frames)
        if self.patch.read_pixels(rgb)["color"] != "OFF":
            self.clear()
            return box, None
        evidence = {"prestart_off_guaranteed": self.stable_off_s == STABLE_OFF_S,
                    "recovery_long_off_observed": self.stable_off_s == RETRY_OFF_S,
                    "stable_off_s": elapsed,
                    "baseline_frames": len(self.frames), "max_jitter_px": JITTER_PX}
        patch = self.patch
        self.clear()
        return box, (patch, evidence)


def barcode_regions(bgr, max_candidates=6):
    """Find bar patterns and deskew crops, retaining real surrounding pixels."""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    ds = min(1., 1280. / gray.shape[1])
    small = cv2.resize(gray, None, fx=ds, fy=ds, interpolation=cv2.INTER_AREA) if ds < 1 else gray
    gx = np.abs(cv2.Sobel(small, cv2.CV_32F, 1, 0, ksize=3))
    gy = np.abs(cv2.Sobel(small, cv2.CV_32F, 0, 1, ksize=3))
    gradient = cv2.convertScaleAbs(np.maximum(gx-gy, 0))
    _, binary = cv2.threshold(gradient, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    proposals = []
    for size in ((13, 3), (25, 5)):
        joined = cv2.morphologyEx(binary, cv2.MORPH_CLOSE,
                                 cv2.getStructuringElement(cv2.MORPH_RECT, size))
        contours, _ = cv2.findContours(joined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv2.contourArea(contour) < 80:
                continue
            points = cv2.boxPoints(cv2.minAreaRect(contour)).astype(np.float32)/ds
            edges = np.roll(points, -1, axis=0)-points
            lengths = np.linalg.norm(edges, axis=1)
            longest = int(np.argmax(lengths))
            width, height = float(lengths[longest]), float(lengths[(longest+1) % 4])
            if not (width >= 40 and height >= 8 and 2 <= width/max(height, 1) <= 16):
                continue
            if width*height > .2*gray.size:
                continue
            axis = edges[longest]/width
            if axis[0] < 0:
                axis = -axis
            if axis[0] < .65:  # Upright label with moderate camera roll.
                continue
            vertical = np.float32([-axis[1], axis[0]])
            center = points.mean(axis=0)
            hw, hh = width/2+max(10., .18*width), height/2+max(4., .15*height)
            quad = np.float32([center-axis*hw-vertical*hh, center+axis*hw-vertical*hh,
                               center+axis*hw+vertical*hh, center-axis*hw+vertical*hh])
            if (quad[:, 0].min() < 0 or quad[:, 1].min() < 0
                    or quad[:, 0].max() >= gray.shape[1] or quad[:, 1].max() >= gray.shape[0]):
                continue
            x, y, w, h = cv2.boundingRect(contour)
            strength = float(gradient[y:y+h, x:x+w].mean())
            proposals.append((strength*np.sqrt(width*height), quad, hw, hh))
    selected, regions = [], []
    for _, quad, hw, hh in sorted(proposals, key=lambda p: p[0], reverse=True):
        lo, hi = quad.min(axis=0), quad.max(axis=0)
        duplicate = False
        for old_lo, old_hi in selected:
            overlap = np.maximum(0, np.minimum(hi, old_hi)-np.maximum(lo, old_lo)).prod()
            union = (hi-lo).prod()+(old_hi-old_lo).prod()-overlap
            if overlap/max(union, 1) > .4:
                duplicate = True
                break
        if duplicate:
            continue
        w, h = int(np.ceil(2*hw)), int(np.ceil(2*hh))
        target = np.float32([[0, 0], [w-1, 0], [w-1, h-1], [0, h-1]])
        transform = cv2.getPerspectiveTransform(quad, target)
        patch = cv2.warpPerspective(bgr, transform, (w, h), flags=cv2.INTER_CUBIC)
        regions.append((patch, np.linalg.inv(transform), quad))
        selected.append((lo, hi))
        if len(regions) >= max_candidates:
            break
    return regions


def barcode_label_regions(patch):
    """Find a bright paper outline inside a stripe candidate, without a wall-color prior.

    Keep pixels inside the detected paper; no generated white padding and no
    fixed percentage removal of the stripe area. At most one label is returned.
    """
    gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
    _, white = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    white = cv2.morphologyEx(white, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    height, width = gray.shape
    proposals = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < .15*gray.size or cv2.pointPolygonTest(contour, (width/2, height/2), False) < 0:
            continue
        hull = cv2.convexHull(contour)
        approx = cv2.approxPolyDP(hull, .02*cv2.arcLength(hull, True), True)
        corners = approx.reshape(-1, 2).astype(np.float32) if len(approx) == 4 else cv2.boxPoints(cv2.minAreaRect(hull))
        center = corners.mean(axis=0)
        angles = np.arctan2(corners[:, 1]-center[1], corners[:, 0]-center[0])
        corners = corners[np.argsort(angles)]
        corners = np.roll(corners, -int(np.argmin(corners.sum(axis=1))), axis=0)
        w = int(round(max(np.linalg.norm(corners[1]-corners[0]), np.linalg.norm(corners[2]-corners[3]))))
        h = int(round(max(np.linalg.norm(corners[3]-corners[0]), np.linalg.norm(corners[2]-corners[1]))))
        if w < max(40, .45*width) or h < 8 or not 1.8 <= w/max(h, 1) <= 18:
            continue
        if w >= .98*width and h >= .98*height:
            continue
        proposals.append((area, corners, w, h))
    if not proposals:
        return []
    _, corners, w, h = max(proposals, key=lambda p: p[0])
    target = np.float32([[0, 0], [w-1, 0], [w-1, h-1], [0, h-1]])
    transform = cv2.getPerspectiveTransform(corners, target)
    if not np.isfinite(transform).all() or abs(np.linalg.det(transform)) < 1e-12:
        return []
    label = cv2.warpPerspective(patch, transform, (w, h), flags=cv2.INTER_CUBIC)
    return [(label, np.linalg.inv(transform), corners)]


def decode_with_reader(bgr, origin, upscale, capture_s, full_scan, read_code128, backend):
    """Shared crop/rotation pipeline; the reader supplies decoded Code128 results."""
    started = time.perf_counter()
    reads = read_code128(bgr)
    scale, raw_count = 1, len(reads)
    stage = "full_native" if reads else "none"
    inverse = np.eye(3, dtype=np.float64)
    if not reads and upscale == 2:
        scale = 2
        enlarged = cv2.resize(bgr, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
        reads = read_code128(enlarged)
        if reads:
            stage = "full_upscale"
    candidates, selected_quad = [], None
    label_refinements = 0
    if not reads:
        candidates = barcode_regions(bgr)
        for patch, candidate_inverse, quad in candidates:
            scale = 1
            reads = read_code128(patch)
            candidate_stage = "candidate_native"
            if not reads and upscale == 2:
                scale = 2
                enlarged = cv2.resize(patch, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
                reads = read_code128(enlarged)
                candidate_stage = "candidate_upscale"
            if reads:
                inverse, stage = candidate_inverse, candidate_stage
                selected_quad = (quad + np.float32(origin)).tolist()
                break
            for label, label_inverse, label_quad in barcode_label_regions(patch):
                label_refinements += 1
                scale = 1
                reads = read_code128(label)
                label_stage = "label_native"
                if not reads and upscale == 2:
                    scale = 2
                    enlarged = cv2.resize(label, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
                    reads = read_code128(enlarged)
                    label_stage = "label_upscale"
                if reads:
                    inverse, stage = candidate_inverse @ label_inverse, label_stage
                    native_quad = cv2.perspectiveTransform(label_quad[None], candidate_inverse)[0]
                    selected_quad = (native_quad + np.float32(origin)).tolist()
                    break
            if reads:
                break
    decoded = []
    for result in reads:
        points = np.float32([[p.x/scale, p.y/scale] for p in
                            (result.position.top_left, result.position.top_right,
                             result.position.bottom_right, result.position.bottom_left)])
        native_points = cv2.perspectiveTransform(points[None], inverse)[0]
        native_points += np.float32(origin)
        decoded.append({"text": result.text, "format": str(result.format),
                        "allowed": result.text in {"P2-01", "P2-02", "P2-03", "P2-04"},
                        "corners": native_points.tolist()})
    return {"reads": decoded, "raw_read_count": raw_count, "decode_scale": scale,
            "decode_stage": stage, "decoder_backend": backend, "candidate_count": len(candidates),
            "selected_candidate_quad": selected_quad, "label_refinement_count": label_refinements,
            "decode_ms": (time.perf_counter()-started)*1000,
            "capture_s": capture_s, "full_scan": full_scan}


def decode_bgr(bgr, origin, upscale, capture_s, full_scan):
    """Default ZXing backend for the original executable."""
    from barcode_support import zxingcpp
    return decode_with_reader(
        bgr, origin, upscale, capture_s, full_scan,
        lambda image: zxingcpp.read_barcodes(image, formats=zxingcpp.BarcodeFormat.Code128),
        "zxingcpp")


def make_runtime(args, barcode_decoder=None):
    barcode_decoder = decode_bgr if barcode_decoder is None else barcode_decoder
    # Existing pure mission rules, archival and replay format are reused.
    from mission_live_support import MissionLive

    class Runtime(MissionLive):
        def __init__(self):
            super().__init__(args.out, upscale=args.barcode_upscale)
            self.last_full_scan = float("-inf")
            self.decode_times = deque(maxlen=10000)

        def process_native(self, bgr, t_s, color):
            if self.future is not None and self.future.done():
                try:
                    result = self.future.result()
                    if self.future_epoch == self.epoch:
                        self.decode_times.append(result["decode_ms"])
                        self.session.observe_barcodes(result["reads"], time_s=result["capture_s"])
                        self.event(kind="barcode", t_s=t_s, **result)
                except Exception as exc:
                    self.session.fail(f"Barcode decoder failed: {exc}")
                self.future = None
            now = time.monotonic()
            if self.future is None and now >= self.next_decode and not self.session.error:
                full = args.barcode_roi is None or now-self.last_full_scan >= args.full_scan_every
                if full:
                    crop, origin = bgr, (0, 0)
                    self.last_full_scan = now
                else:
                    x, y, w, h = args.barcode_roi
                    crop, origin = bgr[y:y+h, x:x+w], (x, y)
                self.future_epoch = self.epoch
                self.future = self.pool.submit(
                    barcode_decoder, np.array(crop, copy=True, order="C"),
                    origin, self.upscale, t_s, full)
                self.next_decode = now+1/args.barcode_hz
            if color is not None and not self.session.recorder.complete:
                self.last_signal_time = t_s
                self.session.observe(t_s, color)
                if self.session.armed:
                    self.event(kind="signal", t_s=t_s, color=color)
            return self.publish()

        def retry_signal(self, t_s, detail):
            # Keep a confirmed barcode; discard in-flight decodes from the failed epoch.
            failed = self.session.snapshot()
            code = self.session.barcode
            if len(failed["sequence"]) == 12 and not failed["signal_complete"]:
                print("[signal] 12 flashes detected, but final OFF was not confirmed; restarting", flush=True)
            self.event(kind="signal_retry", t_s=t_s, failed_round=self.epoch,
                       failed_snapshot=failed, detail=detail,
                       required_quiet_s=RETRY_OFF_S)
            self.epoch += 1
            self.session = type(self.session)()
            self.session.barcode = code
            if code is not None:
                self.session.candidate, self.session.matches = code, 3
            self.signal_saved = False
            self.last_signal_time = None
            self.publish()
            self.log.flush()

    return Runtime()


def signal_recording_hint(session, t_s, fallback):
    """A 12th ON edge is provisional until the terminal OFF is confirmed."""
    recorder = session.recorder
    if not session.armed or session.error or recorder.complete or len(recorder.sequence) != 12:
        return fallback
    if recorder.off_since is None:
        return "12/12; WAIT final OFF; KEEP STILL"
    elapsed = min(3., max(0., t_s-recorder.off_since))
    return f"12/12; final OFF {elapsed:.1f}/3.0s; KEEP STILL"


def camera_warmup(pipe, budget):
    deadline, received = time.monotonic()+budget, 0
    print("[camera] Waiting for first RGB frames", flush=True)
    while received < 30:
        remaining = int((deadline-time.monotonic())*1000)
        if remaining <= 0:
            raise RuntimeError("RGB startup timeout; check camera use and USB connection")
        try:
            frames = pipe.wait_for_frames(min(10000, remaining))
        except RuntimeError as exc:
            if "Frame didn't arrive" not in str(exc):
                raise
            print("[camera] Startup wait timeout; retrying within budget", flush=True)
            continue
        if frames.get_color_frame():
            received += 1


def parser(default_out=None, description=None):
    p = argparse.ArgumentParser(description=description or __doc__)
    mode = p.add_mutually_exclusive_group()
    mode.add_argument("--auto-start", action="store_true", help="keep OFF until READY; automatic prepare/arm")
    mode.add_argument("--no-auto", action="store_true", help="manual o/m preparation")
    p.add_argument("--no-signal-retry", action="store_true",
                   help="disable automatic failed-signal retry in --auto-start mode")
    p.add_argument("--width", type=int, default=CAPTURE_WIDTH)
    p.add_argument("--height", type=int, default=CAPTURE_HEIGHT)
    p.add_argument("--fps", type=int, default=CAPTURE_FPS)
    p.add_argument("--signal-width", type=int, default=SIGNAL_WIDTH)
    p.add_argument("--search-hz", type=float, default=SEARCH_HZ)
    p.add_argument("--display-hz", type=float, default=DISPLAY_HZ)
    p.add_argument("--barcode-hz", type=float, default=BARCODE_HZ)
    p.add_argument("--barcode-roi", type=int, nargs=4, metavar=("X", "Y", "W", "H"),
                   help="optional barcode region in native capture pixels, including quiet margins")
    p.add_argument("--full-scan-every", type=float, default=FULL_SCAN_EVERY_S)
    p.add_argument("--barcode-upscale", type=int, choices=(1, 2), default=1)
    p.add_argument("--prepare-timeout", type=float, default=PREPARE_TIMEOUT_S)
    p.add_argument("--startup-timeout", type=float, default=STARTUP_TIMEOUT_S)
    p.add_argument("--exposure", type=float, default=150.)
    p.add_argument("--wb", type=float, default=4600.)
    p.add_argument("--serial")
    p.add_argument("--out", type=Path, default=default_out or HERE / "reports/live-optimized")
    p.add_argument("--no-show", "--no-gui", action="store_true")
    p.add_argument("--max-frames", type=int)
    a = p.parse_args()
    for name in ("width", "height", "fps", "signal_width"):
        if getattr(a, name) < 1:
            p.error(f"--{name.replace('_', '-')} must be positive")
    for name in ("search_hz", "display_hz", "barcode_hz", "full_scan_every",
                 "prepare_timeout", "startup_timeout", "exposure", "wb"):
        value = getattr(a, name)
        if not np.isfinite(value) or value <= 0:
            p.error(f"--{name.replace('_', '-')} must be positive and finite")
    if not 4 <= a.search_hz <= 60:
        p.error("--search-hz must be 4..60 (location freshness)")
    if a.prepare_timeout > 300 or a.startup_timeout > 60:
        p.error("Preparation/startup timeouts must be <=300/60 seconds")
    if a.max_frames is not None and a.max_frames < 1:
        p.error("--max-frames must be positive")
    if a.barcode_roi:
        x, y, w, h = a.barcode_roi
        if min(x, y) < 0 or min(w, h) < 1 or x+w > a.width or y+h > a.height:
            p.error("--barcode-roi is outside the native capture image")
    return a


class LiveFPS:
    """Observed host rates over a rolling window, not requested camera FPS."""
    def __init__(self, window_s=2.):
        self.window_s = window_s
        self.started = None
        self.events = {name: deque() for name in ("RX", "SIG", "GUI")}
        self.counts = {name: 0 for name in self.events}

    def mark(self, name):
        now = time.monotonic()
        if self.started is None:
            self.started = now
        self.events[name].append(now)
        self.counts[name] += 1
        self.trim(now)

    def trim(self, now):
        for events in self.events.values():
            while events and events[0] < now-self.window_s:
                events.popleft()

    def values(self, now=None):
        now = time.monotonic() if now is None else now
        self.trim(now)
        elapsed = 0. if self.started is None else min(self.window_s, now-self.started)
        if elapsed < .5:
            return {name: 0. for name in self.events}
        return {name: len(events)/elapsed for name, events in self.events.items()}

    def label(self, now=None):
        values = self.values(now)
        return "FPS RX {RX:.1f} | SIG {SIG:.1f} | GUI {GUI:.1f} (2s)".format(**values)


def save_metrics(path, timings, frames, decode_times, error, fps):
    def stats(values):
        return ({"samples": len(values), "median_ms": float(np.median(values)),
                 "p95_ms": float(np.percentile(values, 95))} if values else {"samples": 0})
    data = {"frames": frames, "frame_processing_excluding_capture_and_gui": stats(timings),
            "barcode_decode": stats(decode_times), "error": error,
            "last_window_fps": fps.values(), "fps_window_s": fps.window_s,
            "fps_event_counts": fps.counts,
            "scope": "local runtime measurements, not model accuracy or Jetson certification"}
    (path / "runtime_metrics.json").write_text(json.dumps(data, indent=2)+chr(10))


def main(*, barcode_decoder=None, decoder_name="zxingcpp", entrypoint=None,
         default_out=None, decoder_info=None):
    args = parser(default_out=default_out,
                  description=f"{decoder_name}: HSV/Code128 live mission checker" if entrypoint else None)
    import pyrealsense2 as rs
    rule = load_color_rule()
    runtime = None
    pipe, cfg = rs.pipeline(), rs.config()
    started, error, patch = False, None, None
    finished = signal_done = False
    retry_wait = False
    first_flash_s = None
    last_signal_detail = None
    frames, next_display, last_flush = 0, 0., time.monotonic()
    timings = deque(maxlen=10000)
    fps = LiveFPS()
    last_fps_report = time.monotonic()
    writer = None
    prep = Preparation(rule, args)
    args.out.mkdir(parents=True, exist_ok=True)
    if args.serial:
        cfg.enable_device(args.serial)
    cfg.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
    try:
        profile = pipe.start(cfg)
        started = True
        device = profile.get_device()
        sensor = next(s for s in device.query_sensors()
                      if s.get_info(rs.camera_info.name) == "RGB Camera")
        options = [(rs.option.enable_auto_exposure, 0), (rs.option.exposure, args.exposure),
                   (rs.option.enable_auto_white_balance, 0), (rs.option.white_balance, args.wb)]
        for option, value in options:
            sensor.set_option(option, value)
        fixed = all(abs(sensor.get_option(option)-value) < 1 for option, value in options)
        if not fixed:
            raise RuntimeError("Manual exposure/WB readback mismatch")
        camera_warmup(pipe, args.startup_timeout)
        runtime = make_runtime(args, barcode_decoder=barcode_decoder)
        config = vars(args).copy()
        config["out"] = str(args.out)
        config.update(signal_rule=str(HERE / "src/arm_practice/tower_signal.py"),
                      model_required=False, decoder_backend=decoder_name,
                      decoder_info=decoder_info or {},
                      decoder_entrypoint=str(Path(entrypoint or __file__).resolve()))
        sources = [Path(__file__).resolve(), HERE / "src/arm_practice/tower_signal.py",
                   HERE / "scripts/tower_live_hsv.py", HERE / "scripts/mission_live_support.py",
                   HERE / "scripts/mission_summary.py", HERE / "scripts/barcode_support.py",
                   HERE / "configs/barcodes.json"]
        if entrypoint:
            sources.append(Path(entrypoint).resolve())
        config["source_sha256"] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
        encoded_config = json.dumps(config, indent=2)+chr(10)
        (args.out / "runtime_config.json").write_text(encoded_config)
        (args.out / f"runtime_config_{runtime.run_id}.json").write_text(encoded_config)
        print(f"[camera] Native {args.width}x{args.height}; signal width <= {args.signal_width}", flush=True)
        print("[keys] q=quit, r=new round, v=video; manual: o=confirm OFF, m=arm", flush=True)
        if args.auto_start:
            print("[AUTO] Keep tower OFF and camera fixed until READY", flush=True)
        while True:
            frame = pipe.wait_for_frames(FRAME_TIMEOUT_MS).get_color_frame()
            if not frame:
                continue
            t0 = time.perf_counter()
            fps.mark("RX")
            native = np.asanyarray(frame.get_data())  # View owned by current frame.
            t_s = frame.get_timestamp()/1000.
            display_due = not args.no_show and time.monotonic() >= next_display
            scale = min(1., args.signal_width/native.shape[1])
            work = None
            if not signal_done or display_due:
                work = (cv2.resize(native, (int(round(native.shape[1]*scale)),
                                       int(round(native.shape[0]*scale))), interpolation=cv2.INTER_AREA)
                        if scale < 1 else native)
            color, new_reference, candidate = None, None, None
            if signal_done:
                line = "SIGNAL SAVED; MOVE TO BARCODE" if not finished else "MISSION READY"
            elif runtime.session.error:
                line = "ERROR: reset starts a NEW round"
            elif patch is not None:
                result = patch.read_pixels(patch.pixels(work))
                fps.mark("SIG")
                color = result["color"]
                last_signal_detail = result
                if runtime.session.armed and color in ("R", "Y", "G") and first_flash_s is None:
                    first_flash_s = t_s
                line = f"{color}: {result.get('reason') or result.get('decision')}"
            else:
                try:
                    candidate, new_reference = prep.step(work, t_s)
                except ValueError as exc:
                    runtime.session.fail(exc)
                line = ("RETRY: " if retry_wait else "") + prep.reason
                if new_reference is not None:
                    patch, evidence = new_reference
                    color = "OFF"
                    print("[signal] Reference fixed", flush=True)
            snapshot = runtime.session.snapshot() if finished else runtime.process_native(native, t_s, color)
            if new_reference is not None and args.auto_start and not runtime.session.error:
                accepted = runtime.arm(
                    t_s, color, confirmed_off=True, fixed_settings=fixed,
                    confirmation_source="retry_long_off" if retry_wait else "prestart_off_contract",
                    preparation=evidence)
                if accepted:
                    print("READY: automatic preparation complete; operator may start signal", flush=True)
                else:
                    runtime.session.fail("Automatic arm rejected; no READY")
                snapshot = runtime.publish()
            # Retry signal failures only. Barcode faults remain latched.
            watchdog = (runtime.session.armed and not runtime.session.error
                        and not runtime.session.recorder.complete
                        and first_flash_s is not None
                        and t_s-first_flash_s >= SIGNAL_CYCLE_TIMEOUT_S)
            if watchdog:
                runtime.session.fail("Signal cycle timed out before valid 12-flash completion")
                snapshot = runtime.publish()
            retry_enabled = args.auto_start and not args.no_signal_retry
            if (retry_enabled and not signal_done and
                    (runtime.session.recorder.error is not None or watchdog)):
                runtime.retry_signal(t_s, last_signal_detail)
                patch = None
                first_flash_s = None
                last_signal_detail = None
                retry_wait = True
                prep = Preparation(rule, args, runtime.epoch, stable_off_s=RETRY_OFF_S)
                snapshot = runtime.session.snapshot()
                line = "RETRY: waiting for stable long OFF; partial sequence discarded"
                print(f"[signal] RETRY: partial sequence discarded; wait for {RETRY_OFF_S:g} s stable OFF", flush=True)
            elif (retry_enabled and retry_wait and runtime.session.error ==
                  "Automatic preparation timeout; no READY issued"):
                # Keep waiting if the tower is hidden/ON; never arm on a timer alone.
                runtime.retry_signal(t_s, {"reason": "retry_preparation_timeout"})
                prep = Preparation(rule, args, runtime.epoch, stable_off_s=RETRY_OFF_S)
                snapshot = runtime.session.snapshot()
                line = "RETRY: still waiting for a visible stable OFF tower"
            if new_reference is not None and snapshot["status"] == "recording":
                retry_wait = False
                first_flash_s = None
            if snapshot["signal_complete"] and not snapshot["error"] and not signal_done:
                signal_done = True
                patch = None
                prep.clear()
                candidate = None
                line = "SIGNAL SAVED; MOVE TO BARCODE"
                runtime.log.flush()
                print("SIGNAL COMPLETE: saved; camera may move to barcode", flush=True)
            if snapshot["status"] == "ready" and not finished:
                finished = True
                line = "MISSION READY"
                runtime.log.flush()
                print("[mission] COMPLETE: L1/L2/L3 requirements saved", flush=True)
            line = signal_recording_hint(runtime.session, t_s, line)
            frames += 1
            timings.append((time.perf_counter()-t0)*1000)
            now = time.monotonic()
            if now-last_flush >= 1.:
                runtime.log.flush()
                last_flush = now
            if args.no_show and now-last_fps_report >= 1.:
                print("[fps]", fps.label(now), flush=True)
                last_fps_report = now
            if writer is not None:
                writer.write(native)
            key = 255
            if not args.no_show:
                if display_due:
                    view = work.copy()  # No GUI copy on frames not displayed.
                    if patch is not None:
                        for c, (x0, y0, x1, y1, _) in patch.boxes.items():
                            cv2.rectangle(view, (x0, y0), (x1, y1), COLORS[c], 2)
                    elif candidate is not None and not signal_done:
                        cv2.rectangle(view, tuple(map(int, candidate[:2])),
                                      tuple(map(int, candidate[2:])), (255, 0, 255), 2)
                    lines = [fps.label(), line, f"{snapshot['status']}  {snapshot['barcode']}  {snapshot['sortOrder']}",
                             f"{len(snapshot['sequence'])}/12 " + "".join(snapshot["sequence"]),
                             snapshot["error"] or " ".join(
                                 f"{x['location']}={x['group']}:{x['count']}" for x in snapshot["locations"])]
                    for i, text in enumerate(lines):
                        cv2.putText(view, text, (8, 22+i*23), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 0), 3)
                        cv2.putText(view, text, (8, 22+i*23), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
                    cv2.imshow(f"HSV mission ({decoder_name})", view)
                    fps.mark("GUI")
                    next_display = now+1/args.display_hz
                key = cv2.waitKey(1) & 255
            if key in (ord("q"), 27) or (args.max_frames is not None and frames >= args.max_frames):
                break
            if key == ord("r"):
                runtime.reset()
                prep = Preparation(rule, args, runtime.epoch)
                patch, signal_done, finished = None, False, False
                retry_wait, first_flash_s, last_signal_detail = False, None, None
            elif key == ord("o") and not args.auto_start and patch is None and not signal_done:
                prep = Preparation(rule, args, runtime.epoch)
                prep.manual_requested = True
                print("[signal] Operator confirms OFF", flush=True)
            elif key == ord("m") and not args.auto_start and patch is not None and not signal_done:
                accepted = runtime.arm(t_s, color, confirmed_off=True, fixed_settings=fixed)
                print("READY: operator may start signal" if accepted else "ARM REJECTED", flush=True)
            elif key == ord("v"):
                if writer is not None:
                    writer.release()
                    writer = None
                else:
                    path = args.out / f"camera_{time.time_ns()}.mp4"
                    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"),
                                             args.fps, (args.width, args.height))
                    if not writer.isOpened():
                        writer.release()
                        writer = None
                        print("[video] Writer failed", flush=True)
                    else:
                        print("[video]", path, flush=True)
    except BaseException as exc:
        error = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        try:
            if runtime is not None:
                try:
                    runtime.close(None if finished else error)
                finally:
                    save_metrics(args.out, timings, frames, runtime.decode_times, error, fps)
        finally:
            if writer is not None:
                writer.release()
            if started:
                pipe.stop()
            if not args.no_show:
                cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
