"""바코드(Code 128) 판독 노드 (Jetson, CPU).

- ~/start 호출 전에는 카메라를 구독하지 않는다.
- 같은 문자열이 confirm_count회 연속 판독되면 확정. 판독 실패 프레임은 연속을 끊지 않고,
  다른 문자열이 나오면 연속을 0부터 다시 센다.
- 확정 문자열이 registered_codes에 없으면 status=unregistered (FSM이 SAFE_STOP 판단).
- 노출은 자동(정상) 상태에서 읽는다. 신호 노드가 저노출로 바꾸기 전에 끝나야 한다.
결과: /mission/barcode_result (std_msgs/String JSON, transient local)
"""
import json
import time

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


def _make_decoder():
    """pyzbar 우선, 없으면 zxing-cpp. 둘 다 없으면 None."""
    try:
        from pyzbar import pyzbar

        def dec(gray):
            return [r.data.decode("utf-8", "replace") for r in pyzbar.decode(gray)
                    if r.type in ("CODE128", "CODE-128")]
        return dec, "pyzbar"
    except ImportError:
        pass
    try:
        import zxingcpp

        def dec(gray):
            return [r.text for r in zxingcpp.read_barcodes(gray) if "128" in str(r.format)]
        return dec, "zxingcpp"
    except ImportError:
        return None, "none"


class BarcodeReaderNode(Node):
    def __init__(self):
        super().__init__("barcode_reader")
        d = self.declare_parameter
        d("image_topic", "/camera/front/color/image_raw")
        d("roi_xywh", [0, 0, 0, 0])     # 0이면 전체 화면 [확인: 바코드 예상 위치로 잘라 CPU 절약]
        d("rate_hz", 5.0)
        d("confirm_count", 3)
        d("timeout_s", 20.0)
        d("registered_codes", ["P2-01", "P2-02", "P2-03", "P2-04"])
        self.bridge = CvBridge()
        self.decode, self.backend = _make_decoder()
        self.pub = self.create_publisher(String, "/mission/barcode_result", LATCHED)
        self.create_service(Trigger, "~/start", self._srv_start)
        self.sub = None
        self._last, self._streak, self._t0, self._t_last = None, 0, 0.0, 0.0
        self._reads = []
        self.active = False

    def p(self, n):
        return self.get_parameter(n).value

    def _srv_start(self, req, res):
        if self.decode is None:
            self._publish("failed", None, "no_decoder_backend(pyzbar/zxingcpp)")
            res.success, res.message = False, "디코더 없음"
            return res
        if not self.active:
            self.active, self._t0, self._last, self._streak, self._reads = True, time.monotonic(), None, 0, []
            self.sub = self.create_subscription(Image, self.p("image_topic"), self._on_image, qos_profile_sensor_data)
            self.get_logger().info(f"바코드 판독 시작 ({self.backend})")
        res.success, res.message = True, "started"
        return res

    def _on_image(self, msg: Image):
        now = time.monotonic()
        if now - self._t0 > self.p("timeout_s"):
            self._publish("failed", None, "timeout")
            return
        if now - self._t_last < 1.0 / max(0.5, self.p("rate_hz")):
            return
        self._t_last = now
        import cv2
        gray = cv2.cvtColor(self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8"), cv2.COLOR_BGR2GRAY)
        x, y, w, h = self.p("roi_xywh")
        if w > 0 and h > 0:
            gray = gray[y:y + h, x:x + w]
        codes = self.decode(gray)
        if not codes:
            return                        # 판독 실패는 연속을 끊지 않음
        code = codes[0]
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._reads.append([round(stamp, 3), code])
        self._streak = self._streak + 1 if code == self._last else 1
        self._last = code
        if self._streak >= self.p("confirm_count"):
            status = "confirmed" if code in self.p("registered_codes") else "unregistered"
            self._publish(status, code, "")

    def _publish(self, status, code, reason):
        if self.sub is not None:
            self.destroy_subscription(self.sub)
            self.sub = None
        self.active = False
        out = {"status": status, "code": code, "reason": reason, "reads": self._reads[-20:]}
        self.pub.publish(String(data=json.dumps(out, ensure_ascii=False)))
        self.get_logger().info(f"바코드 판독 종료: {status} {code} {reason}")


def main():
    rclpy.init()
    node = BarcodeReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
