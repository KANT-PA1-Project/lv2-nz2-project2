"""신호등 판독 노드 (Jetson). OBSERVE 하위 상태 LOCATE → LOCK → SYNC → CAPTURE → VOTE.

- ~/start 호출 전에는 카메라를 구독하지 않는다(경기 시작 전 관측 금지 규정).
- LOCATE: YOLO(외부 노드)의 신호등 bbox를 locate_frames개 모아 중앙값 → 램프 ROI 3개 고정.
          실패 시 fallback_fixed_roi로 진행.
- LOCK : 바코드 확정 대기(옵션) → 노출 고정 → 안정 대기.
- SYNC/CAPTURE/VOTE: core.signal_decoder. 시간은 이미지 header.stamp(촬영 시각)만 사용.
결과: /mission/signal_result (std_msgs/String JSON, transient local)
"""
import json
import time
from typing import List, Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2DArray

from .core.lamp_classifier import LampClassifier, LampClassifierParams
from .core.signal_decoder import STAGE_CONFIRMED, SignalDecoder, SignalParams, count_colors

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


class SignalDecoderNode(Node):
    def __init__(self):
        super().__init__("signal_decoder")
        d = self.declare_parameter
        d("image_topic", "/camera/front/color/image_raw")
        d("bbox_topic", "/perception/traffic_light/detections")
        d("bbox_class_id", "traffic_light")
        d("barcode_result_topic", "/mission/barcode_result")
        d("wait_barcode_before_lock", True)
        d("locate_frames", 15)
        d("locate_timeout_s", 10.0)
        d("fallback_fixed_roi", True)
        d("fixed_roi_xywh", [0, 0, 0, 0])          # [확인] 출발 자세 기하 계산값
        d("lamp_order", ["R", "Y", "G"])
        d("roi_core_ratio", 0.6)
        d("roi_stride_px", 3)
        d("camera_node", "/camera/front/camera")   # [확인] realsense2_camera 노드 이름
        d("auto_exposure_param", "rgb_camera.enable_auto_exposure")
        d("exposure_param", "rgb_camera.exposure")
        d("exposure_signal", 0)                    # 0 = 변경 안 함 [확인: 실물 앞에서 결정]
        d("exposure_settle_s", 0.5)
        d("min_avg_fps", 10.0)
        for k, v in vars(SignalParams()).items():
            d(k, v)
        for k in ("on_ratio_thresh", "lit_ratio", "baseline_window_s"):
            d(k, getattr(LampClassifierParams(), k))

        self.bridge = CvBridge()
        self.pub_result = self.create_publisher(String, "/mission/signal_result", LATCHED)
        self.pub_status = self.create_publisher(String, "/mission/signal_status", 10)
        self.create_service(Trigger, "~/start", self._srv_start)
        self.create_service(Trigger, "~/stop", self._srv_stop)
        self.create_subscription(String, self.p("barcode_result_topic"), self._on_barcode, LATCHED)
        self.create_timer(0.5, self._publish_status)

        self.stage = "IDLE"
        self.sub_img = None
        self.sub_bbox = None
        self.barcode_ok = False
        self._bboxes: List[List[float]] = []
        self._locate_t0 = 0.0
        self._lock_t0 = 0.0
        self.rois: Optional[List[List[int]]] = None
        self.clf: Optional[LampClassifier] = None
        self.dec: Optional[SignalDecoder] = None
        self._n_img, self._t_first_img, self._t_start_cmd = 0, None, None

    def p(self, name):
        return self.get_parameter(name).value

    # ------------------------------------------------------------ 서비스
    def _srv_start(self, req, res):
        if self.stage not in ("IDLE", "STOPPED"):
            res.success, res.message = True, f"이미 진행 중: {self.stage}"
            return res
        sp = SignalParams(**{k: self.p(k) for k in vars(SignalParams())})
        cp = LampClassifierParams(lamp_order=tuple(self.p("lamp_order")),
                                  on_ratio_thresh=self.p("on_ratio_thresh"), lit_ratio=self.p("lit_ratio"),
                                  baseline_window_s=self.p("baseline_window_s"))
        self.dec, self.clf = SignalDecoder(sp), LampClassifier(cp)
        self._t_start_cmd = self.get_clock().now().nanoseconds * 1e-9
        self.stage, self._locate_t0, self._bboxes = "LOCATE", time.monotonic(), []
        # 여기서 처음으로 카메라를 구독한다
        self.sub_img = self.create_subscription(Image, self.p("image_topic"), self._on_image, qos_profile_sensor_data)
        self.sub_bbox = self.create_subscription(Detection2DArray, self.p("bbox_topic"), self._on_bbox, 10)
        self.get_logger().info("신호 판독 시작 (LOCATE)")
        res.success, res.message = True, "started"
        return res

    def _srv_stop(self, req, res):
        self._unsubscribe()
        self.stage = "STOPPED"
        res.success, res.message = True, "stopped"
        return res

    def _unsubscribe(self):
        for s in (self.sub_img, self.sub_bbox):
            if s is not None:
                self.destroy_subscription(s)
        self.sub_img = self.sub_bbox = None

    # ------------------------------------------------------------ 콜백
    def _on_barcode(self, msg: String):
        try:
            self.barcode_ok = json.loads(msg.data).get("status") in ("confirmed", "unregistered", "failed")
        except json.JSONDecodeError:
            pass

    def _on_bbox(self, msg: Detection2DArray):
        if self.stage != "LOCATE":
            return
        for det in msg.detections:
            if det.results and det.results[0].hypothesis.class_id == self.p("bbox_class_id"):
                b = det.bbox
                self._bboxes.append([b.center.position.x - b.size_x / 2, b.center.position.y - b.size_y / 2,
                                     b.size_x, b.size_y])
                break

    def _on_image(self, msg: Image):
        self._n_img += 1
        if self.stage == "LOCATE":
            self._step_locate()
        elif self.stage == "LOCK":
            self._step_lock()
        elif self.stage == "RUN":
            self._step_run(msg)

    # ------------------------------------------------------------ 단계
    def _step_locate(self):
        if len(self._bboxes) >= self.p("locate_frames"):
            box = np.median(np.array(self._bboxes), axis=0).tolist()
            self._set_rois(box, "yolo")
        elif time.monotonic() - self._locate_t0 > self.p("locate_timeout_s"):
            roi = list(self.p("fixed_roi_xywh"))
            if self.p("fallback_fixed_roi") and roi[2] > 0 and roi[3] > 0:
                self.get_logger().warning("YOLO 신호등 검출 실패 → 고정 ROI 폴백")
                self._set_rois(roi, "fixed")
            else:
                self._finish(False, "locate_failed")

    def _set_rois(self, box, source):
        x, y, w, h = box
        k = self.p("roi_core_ratio")
        rois = []
        for i in range(3):  # 위→아래
            cy = y + h * (i + 0.5) / 3.0
            cw, ch = w * k, (h / 3.0) * k
            rois.append([int(x + (w - cw) / 2), int(cy - ch / 2), max(1, int(cw)), max(1, int(ch))])
        self.rois = rois
        self.get_logger().info(f"램프 ROI 확정({source}): {rois}")
        if self.sub_bbox is not None:
            self.destroy_subscription(self.sub_bbox)   # 이후 YOLO 재확인 안 함(저노출에서 실패)
            self.sub_bbox = None
        self.stage, self._lock_t0, self._exposure_set = "LOCK", time.monotonic(), False

    def _step_lock(self):
        if self.p("wait_barcode_before_lock") and not self.barcode_ok:
            return
        if not self._exposure_set:
            self._set_exposure()
            self._exposure_set, self._lock_t0 = True, time.monotonic()
            return
        if time.monotonic() - self._lock_t0 >= self.p("exposure_settle_s"):
            self.stage = "RUN"
            self.get_logger().info("노출 고정 완료 → SYNC")

    def _set_exposure(self):
        val = int(self.p("exposure_signal"))
        if val <= 0:
            self.get_logger().warning("exposure_signal=0: 노출을 바꾸지 않음 (실물 측정 후 설정 필요)")
            return
        try:
            from rclpy.parameter_client import AsyncParameterClient
            cli = AsyncParameterClient(self, self.p("camera_node"))
            cli.set_parameters([Parameter(self.p("auto_exposure_param"), value=False),
                                Parameter(self.p("exposure_param"), value=val)])
            self.get_logger().info(f"노출 고정 요청: {val}")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"노출 설정 실패: {e}")

    def _step_run(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        s = max(1, int(self.p("roi_stride_px")))
        vals = []
        for x, y, w, h in self.rois:
            patch = img[max(0, y):y + h:s, max(0, x):x + w:s]
            vals.append(float(patch.max(axis=2).mean()) if patch.size else 0.0)   # HSV의 V = max(B,G,R)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9               # 촬영 시각만 사용
        if self._t_first_img is None:
            self._t_first_img = t
        self.dec.feed(t, self.clf.classify(t, vals))
        if self.dec.done:
            self._finish(self.dec.stage == STAGE_CONFIRMED, self.dec.fail_reason)

    # ------------------------------------------------------------ 결과
    def _finish(self, ok: bool, reason: str):
        self._unsubscribe()
        self.stage = "DONE" if ok else "FAILED"
        seq = self.dec.result if (ok and self.dec) else None
        out = {"status": "confirmed" if ok else "failed", "sequence": seq,
               "counts": count_colors(seq) if seq else None, "reason": reason,
               "start_cmd_time": self._t_start_cmd, "first_image_stamp": self._t_first_img,
               "summary": self.dec.summary() if self.dec else None,
               "frames": [f.to_dict() for f in self.dec.frames] if self.dec else []}
        self.pub_result.publish(String(data=json.dumps(out, ensure_ascii=False)))
        self.get_logger().info(f"신호 판독 종료: {out['status']} {seq} {reason}")

    def _publish_status(self):
        st = {"stage": self.stage, "images": self._n_img}
        if self.dec:
            st.update(self.dec.summary())
        self.pub_status.publish(String(data=json.dumps(st, ensure_ascii=False)))


def main():
    rclpy.init()
    node = SignalDecoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
