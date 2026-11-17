#!/usr/bin/python3
"""
barcode_signal  (NZ2 인지) — HSV 신호등·ZXing Code128·12회 기록 통합 초안.

입력: signal_image_topic(상단), barcode_image_topic(그리퍼), sensor_msgs/Image.
출력: /perception/traffic_light, /perception/barcodes, /perception/signal_result,
      /perception/mission_requirements, ~/status, /vision/tower_state.
서비스: ~/prepare_signal, ~/scan_barcode, ~/pause, ~/reset (std_srvs/Trigger).
카메라 드라이버·YOLO·모터·Nav2를 시작하지 않는다. 인지 규칙은 perception_root에서 재사용.
초안: 토픽명·자세 완료 통보 방식은 통합 시 합의 필요. 자세 이동은 외부 담당.
"""
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nz2_interfaces.msg import TrafficLight, Barcode, BarcodeArray

from barcode_signal_core import BarcodeSignalCore, load_runtime


@dataclass
class ImageSample:
    message: Image
    received_s: float

    @property
    def stamp_ns(self):
        return self.message.header.stamp.sec*1_000_000_000 + self.message.header.stamp.nanosec


def image_to_bgr(message):
    """8bit RGB/BGR만 허용. step 패딩을 지키는 메시지 소유 view(작업자에서는 별도 복사)."""
    if message.encoding.lower() not in ("bgr8", "rgb8"):
        raise ValueError(f"Expected rgb8/bgr8 Image, got {message.encoding}")
    h, w, step = int(message.height), int(message.width), int(message.step)
    if h <= 0 or w <= 0 or step < 3*w or len(message.data) < h*step:
        raise ValueError("Malformed color Image dimensions/step/data")
    image = np.ndarray((h, w, 3), dtype=np.uint8, buffer=message.data, strides=(step, 3, 1))
    return image if message.encoding.lower() == "bgr8" else image[:, :, ::-1]


def default_root():
    for parent in Path(__file__).resolve().parents:
        candidate = parent / "Claude outputs/tower_signal"
        if (candidate / "scripts/live_mission_optimized.py").is_file():
            return str(candidate)
    return ""


class BarcodeSignalNode(Node):
    def __init__(self):
        super().__init__("barcode_signal")
        def param(name, value):
            return self.declare_parameter(name, value, descriptor=ParameterDescriptor(read_only=True)).value
        root = param("perception_root", "") or default_root()
        self.live, session_class = load_runtime(root, param("r12_src", ""))
        self.signal_topic = param("signal_image_topic", "/mast/color/image_raw")
        self.barcode_topic = param("barcode_image_topic", "/gripper/color/image_raw")
        self.process_hz = float(param("process_hz", 30.))
        self.barcode_hz = float(param("barcode_hz", 5.))
        self.status_hz = float(param("status_hz", 1.))
        self.max_age_s = float(param("max_image_age_s", .25))
        self.max_decode_age_s = float(param("max_decode_age_s", 2.))
        self.full_scan_every_s = float(param("full_scan_every_s", 2.))
        self.upscale = int(param("barcode_upscale", 2))
        self.roi = list(param("barcode_roi", [0, 0, 0, 0]))
        self.log_dir = Path(param("log_dir", "~/nz2_logs/barcode_signal")).expanduser()
        self.auto_start = bool(param("auto_start", False))
        self.search_hz = float(param("search_hz", 10.))
        width = int(param("signal_width", 640))
        if (not all(np.isfinite(x) and x > 0 for x in [self.process_hz, self.barcode_hz, self.status_hz,
                    self.max_age_s, self.max_decode_age_s, self.full_scan_every_s, self.search_hz])
                or self.upscale not in (1, 2) or width < 64
                or len(self.roi) != 4 or any(x < 0 for x in self.roi)
                or not self.signal_topic or not self.barcode_topic):
            raise ValueError("Invalid barcode_signal parameters")
        self.core = BarcodeSignalCore(self.live, session_class,
                    fixed_settings=bool(param("fixed_camera_settings_confirmed", False)),
                    exposure_token=param("camera_settings_token", "exposure150_wb4600"),
                    search_hz=self.search_hz, signal_width=width,
                    auto_retry=bool(param("auto_retry_signal", True)))
        for field in ["STATE_OFF", "STATE_UNKNOWN", "raw_state", "visible"]:
            if not hasattr(TrafficLight(), field):
                raise RuntimeError("TrafficLight.msg needs OFF/UNKNOWN, raw_state and visible; rebuild nz2_interfaces")
        self.states = {"R": TrafficLight.STATE_RED, "Y": TrafficLight.STATE_YELLOW,
                       "G": TrafficLight.STATE_GREEN, "OFF": TrafficLight.STATE_OFF,
                       "UNKNOWN": TrafficLight.STATE_UNKNOWN}
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="barcode_decode")
        self.future = self.job = None
        self.next_decode_s = self.last_full_scan_s = float("-inf")
        self.latest, self.subscriptions_by_topic, self.last_stamp, self.stale_notified = {}, {}, {}, set()
        self.raw_previous, self.confirmed_frames = "UNKNOWN", 0
        self._summary_encoded = self._signal_encoded = None
        self._last_summary_object = None
        self._session_id = time.time_ns()
        self.last_status_s = float("-inf")
        sensor_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                               reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE)
        self.sensor_qos = sensor_qos
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.light_pub = self.create_publisher(TrafficLight, "/perception/traffic_light", 10)
        self.legacy_pub = self.create_publisher(String, "/vision/tower_state", 10)
        self.barcode_pub = self.create_publisher(BarcodeArray, "/perception/barcodes", 10)
        self.signal_pub = self.create_publisher(String, "/perception/signal_result", latched)
        self.mission_pub = self.create_publisher(String, "/perception/mission_requirements", latched)
        self.status_pub = self.create_publisher(String, "~/status", 10)
        for name, method in [("prepare_signal", self.core.prepare_signal), ("scan_barcode", self.core.scan_barcode),
                             ("pause", self.core.pause), ("reset", self._reset)]:
            self.create_service(Trigger, "~/"+name, self._service(method))
        self.timer = self.create_timer(1/self.process_hz, self._tick)
        self._publish_state(force=True)
        if self.auto_start:
            ok, message = self.core.prepare_signal()
            self.get_logger().info(message) if ok else self.get_logger().warning(message)
            self._sync_subscriptions()
        self.get_logger().info("Draft node ready. prepare_signal caller must guarantee OFF, fixed camera settings and stationary view.")

    def _reset(self):
        self.core.reset()
        self._session_id = time.time_ns()
        self.last_stamp.clear()
        self.next_decode_s = self.last_full_scan_s = float("-inf")
        self._summary_encoded = self._signal_encoded = None
        # transient_local의 지난 미션 결과를 명시적으로 철회한다.
        self.signal_pub.publish(String(data=json.dumps({"signal_complete": False, "sequence": [], "session_id": self._session_id})))
        return True, "New mission round; previous records cleared"

    def _service(self, method):
        def callback(request, response):
            del request
            response.success, response.message = method()
            self._sync_subscriptions()
            self._publish_state(force=True)
            return response
        return callback

    def _sync_subscriptions(self):
        wanted = set()
        if self.core.signal_active:
            wanted.add(self.signal_topic)
        if self.core.barcode_active:
            wanted.add(self.barcode_topic)
        for topic in list(self.subscriptions_by_topic):
            if topic not in wanted:
                self.destroy_subscription(self.subscriptions_by_topic.pop(topic))
                self.latest.pop(topic, None)
                self.stale_notified.discard(topic)
        for topic in wanted:
            if topic not in self.subscriptions_by_topic:
                self.subscriptions_by_topic[topic] = self.create_subscription(
                    Image, topic, lambda msg, key=topic: self._receive(key, msg), self.sensor_qos)

    def _receive(self, topic, message):
        # 콜백에서는 참조 하나만 보관. 변환·복사·해독 작업을 대기열에 쌓지 않는다.
        self.latest[topic] = ImageSample(message, time.monotonic())

    def _fresh(self, sample, now):
        if sample is None or sample.stamp_ns <= 0 or now-sample.received_s > self.max_age_s:
            return False
        age = (self.get_clock().now().nanoseconds-sample.stamp_ns)/1e9
        return -.05 <= age <= self.max_age_s

    @staticmethod
    def _header(message, stamp_ns, frame_id):
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(int(stamp_ns), 1_000_000_000)
        message.header.frame_id = frame_id

    def _publish_light(self, sample=None):
        color = self.core.raw_color if sample is not None else "UNKNOWN"
        visible = self.core.visible if sample is not None else False
        self.confirmed_frames = min(255, self.confirmed_frames+1) if color == self.raw_previous else 1
        self.raw_previous = color
        msg = TrafficLight()
        self._header(msg, sample.stamp_ns if sample is not None else self.get_clock().now().nanoseconds,
                     sample.message.header.frame_id if sample is not None else "")
        msg.raw_state, msg.visible = self.states[color], visible
        msg.confirmed_frames = self.confirmed_frames if visible else 0
        msg.state = msg.raw_state if visible and self.confirmed_frames >= 3 else TrafficLight.STATE_UNKNOWN
        # 시간적 일치도이며 실측 확률 보정값은 아니다.
        msg.confidence = min(self.confirmed_frames/3., 1.) if visible and color != "UNKNOWN" else 0.
        self.light_pub.publish(msg)
        self.legacy_pub.publish(String(data=color))

    def _poll_decode(self, now):
        if self.future is None or not self.future.done():
            return
        job, self.job = self.job, None
        future, self.future = self.future, None
        try:
            result = future.result()
            if job["generation"] != self.core.generation or not self.core.barcode_active:
                return
            if now-job["received_s"] > self.max_decode_age_s:
                return
            self.core.feed_barcodes(result["reads"], job["stamp_ns"]/1e9)
            msg = BarcodeArray()
            self._header(msg, job["stamp_ns"], job["frame_id"])
            for read in result["reads"]:
                if not read["allowed"]:
                    continue
                code = Barcode()
                self._header(code, job["stamp_ns"], job["frame_id"])
                code.data, code.format = read["text"], read["format"]
                code.confidence = 1. if self.core.session.barcode == code.data else 0.
                code.position.x = code.position.y = code.position.z = float("nan")
                msg.codes.append(code)
            self.barcode_pub.publish(msg)
        except Exception as exc:
            if job["generation"] == self.core.generation and self.core.barcode_active:
                self.core.session.fail(f"Barcode decoder failed: {exc}")
                self.core.phase = "error"
                self.core.barcode_active = False
                self.get_logger().error(str(exc))

    def _submit_decode(self, sample, now):
        if self.future is not None or now < self.next_decode_s:
            return
        bgr = image_to_bgr(sample.message)
        x, y, w, h = self.roi
        full = w == 0 or h == 0 or now-self.last_full_scan_s >= self.full_scan_every_s
        full = full or x+w > bgr.shape[1] or y+h > bgr.shape[0]
        if full:
            crop, origin = bgr, (0, 0)
            self.last_full_scan_s = now
        else:
            crop, origin = bgr[y:y+h, x:x+w], (x, y)
        self.job = dict(generation=self.core.generation, received_s=sample.received_s,
                        stamp_ns=sample.stamp_ns, frame_id=sample.message.header.frame_id)
        # 작업자 수명 동안 ROS 메시지가 교체되므로 여기서만 소유 복사본을 만든다.
        self.future = self.pool.submit(self.live.decode_bgr, np.array(crop, copy=True, order="C"),
                                      origin, self.upscale, sample.stamp_ns/1e9, full)
        self.next_decode_s = now+1/self.barcode_hz

    def _tick(self):
        now = time.monotonic()
        self._poll_decode(now)
        for topic, kind in [(self.signal_topic, "signal"), (self.barcode_topic, "barcode")]:
            active = self.core.signal_active if kind == "signal" else self.core.barcode_active
            if not active:
                continue
            sample = self.latest.get(topic)
            if not self._fresh(sample, now):
                if kind == "signal" and topic not in self.stale_notified:
                    if self.core.last_signal_s is not None:
                        self.core.signal_fault("Signal image stale or missing")
                    else:
                        self.core.reason = "Waiting for the first fresh stamped signal image"
                    self._publish_light()
                self.stale_notified.add(topic)
                continue
            self.stale_notified.discard(topic)
            key = (kind, topic)
            if sample.stamp_ns == self.last_stamp.get(key):
                continue
            if sample.stamp_ns < self.last_stamp.get(key, -1):
                if kind == "signal":
                    self.core.signal_fault("ROS image clock moved backwards; prepare a new reference")
                self.last_stamp[key] = sample.stamp_ns
                continue
            try:
                if kind == "signal":
                    bgr = image_to_bgr(sample.message)
                    self.core.feed_signal(bgr, sample.stamp_ns/1e9, sample.message.header.frame_id)
                    self._publish_light(sample)
                    self.last_stamp[key] = sample.stamp_ns
                elif self.future is None and now >= self.next_decode_s:
                    self._submit_decode(sample, now)
                    self.last_stamp[key] = sample.stamp_ns
            except Exception as exc:
                self.core.pause()
                self.core.session.fail(f"Image processing failed: {exc}")
                self.core.phase = "error"
                self.get_logger().error(str(exc))
        self._sync_subscriptions()
        self._publish_state()

    def _atomic_json(self, name, data):
        self.log_dir.mkdir(parents=True, exist_ok=True)
        path = self.log_dir/name
        temporary = path.with_suffix(".tmp")
        temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
        temporary.replace(path)

    def _publish_state(self, force=False):
        plain = self.core.summary()
        summary = dict(plain, session_id=self._session_id)
        changed = plain is not self._last_summary_object
        encoded = json.dumps(summary, ensure_ascii=False, sort_keys=True) if force or changed else self._summary_encoded
        if force or changed:
            self._last_summary_object = plain
            self.mission_pub.publish(String(data=encoded))
            self._summary_encoded = encoded
            if summary["status"] == "ready":
                self._atomic_json(f"mission_{self._session_id}.json", summary)
                self._atomic_json("mission_summary.json", summary)
        if (force or changed) and summary["signal_complete"] and not summary["error"]:
            signal = {k: summary[k] for k in ["sequence", "counts", "signal_complete", "session_id"]}
            signal_encoded = json.dumps(signal, ensure_ascii=False, sort_keys=True)
            if signal_encoded != self._signal_encoded:
                self.signal_pub.publish(String(data=signal_encoded))
                self._atomic_json(f"signal_{self._session_id}.json", signal)
                self._signal_encoded = signal_encoded
        now = time.monotonic()
        if force or now-self.last_status_s >= 1/self.status_hz:
            status = dict(phase=self.core.phase, requested_view=self.core.requested_view,
                          reason=self.core.reason, signal_active=self.core.signal_active,
                          barcode_active=self.core.barcode_active, signal_complete=summary["signal_complete"],
                          barcode=summary["barcode"], error=summary["error"],
                          decoder_busy=self.future is not None, fixed_settings_confirmed=self.core.fixed_settings)
            self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))
            if not self.core.signal_active:
                self._publish_light()
            self.last_status_s = now

    def destroy_node(self):
        self.pool.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BarcodeSignalNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
