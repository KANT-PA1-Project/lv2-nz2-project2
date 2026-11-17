"""신호 점멸 해독 노드 (Jetson). 무한루프 12회 점멸을 프레임 동기화 → K회 투표로 확정한다.

영상 처리(신호등 위치·점등 판정·노출)는 nz2_perception(traffic_light_filter)이 맡는다(PR #22 계약).
이 노드는 그 결과 /perception/traffic_light (nz2_interfaces/TrafficLight)의 프레임별 판정 raw_state만 받는다.

- ~/start 호출 전에는 /perception/traffic_light를 구독하지 않는다(경기 시작 전 관측 금지 규정).
- 시간은 TrafficLight.header.stamp(촬영 시각)만 사용한다.
- '안 보임'(visible=false 또는 STATE_UNKNOWN)은 꺼짐으로 세지 않고 판정 불가(AMBIG)로 넣는다.
결과: /mission/signal_result (std_msgs/String JSON, transient local)
"""
import json

import rclpy
from nz2_interfaces.msg import TrafficLight
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .core.lamp_classifier import LABEL_AMBIG, LABEL_OFF
from .core.signal_decoder import STAGE_CONFIRMED, SignalDecoder, SignalParams, count_colors

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)

STATE_TO_LABEL = {
    TrafficLight.STATE_RED: "R",
    TrafficLight.STATE_YELLOW: "Y",
    TrafficLight.STATE_GREEN: "G",
    TrafficLight.STATE_OFF: LABEL_OFF,
}


def to_label(msg: TrafficLight) -> str:
    """프레임 판정 → 해독기 라벨. 안 보이거나 모르면 꺼짐이 아니라 AMBIG."""
    if not msg.visible:
        return LABEL_AMBIG
    return STATE_TO_LABEL.get(msg.raw_state, LABEL_AMBIG)


class SignalDecoderNode(Node):
    def __init__(self):
        super().__init__("signal_decoder")
        d = self.declare_parameter
        d("traffic_light_topic", "/perception/traffic_light")
        for k, v in vars(SignalParams()).items():
            d(k, v)

        self.pub_result = self.create_publisher(String, "/mission/signal_result", LATCHED)
        self.pub_status = self.create_publisher(String, "/mission/signal_status", 10)
        self.create_service(Trigger, "~/start", self._srv_start)
        self.create_service(Trigger, "~/stop", self._srv_stop)
        self.create_timer(0.5, self._publish_status)

        self.stage = "IDLE"
        self.sub = None
        self.dec = None
        self._n_msg, self._t_first, self._t_start_cmd = 0, None, None

    def p(self, name):
        return self.get_parameter(name).value

    # ------------------------------------------------------------ 서비스
    def _srv_start(self, req, res):
        if self.stage not in ("IDLE", "STOPPED"):
            res.success, res.message = True, f"이미 진행 중: {self.stage}"
            return res
        self.dec = SignalDecoder(SignalParams(**{k: self.p(k) for k in vars(SignalParams())}))
        self._t_start_cmd = self.get_clock().now().nanoseconds * 1e-9
        self.stage = "RUN"
        # 여기서 처음으로 신호등 판정을 구독한다
        self.sub = self.create_subscription(TrafficLight, self.p("traffic_light_topic"), self._on_light,
                                            qos_profile_sensor_data)
        self.get_logger().info("신호 해독 시작")
        res.success, res.message = True, "started"
        return res

    def _srv_stop(self, req, res):
        self._unsubscribe()
        self.stage = "STOPPED"
        res.success, res.message = True, "stopped"
        return res

    def _unsubscribe(self):
        if self.sub is not None:
            self.destroy_subscription(self.sub)
            self.sub = None

    # ------------------------------------------------------------ 콜백
    def _on_light(self, msg: TrafficLight):
        if self.stage != "RUN":
            return
        self._n_msg += 1
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9   # 촬영 시각만 사용
        if self._t_first is None:
            self._t_first = t
        self.dec.feed(t, to_label(msg))
        if self.dec.done:
            self._finish(self.dec.stage == STAGE_CONFIRMED, self.dec.fail_reason)

    # ------------------------------------------------------------ 결과
    def _finish(self, ok: bool, reason: str):
        self._unsubscribe()
        self.stage = "DONE" if ok else "FAILED"
        seq = self.dec.result if ok else None
        out = {"status": "confirmed" if ok else "failed", "sequence": seq,
               "counts": count_colors(seq) if seq else None, "reason": reason,
               "start_cmd_time": self._t_start_cmd, "first_sample_stamp": self._t_first,
               "summary": self.dec.summary(),
               "frames": [f.to_dict() for f in self.dec.frames]}
        self.pub_result.publish(String(data=json.dumps(out, ensure_ascii=False)))
        self.get_logger().info(f"신호 해독 종료: {out['status']} {seq} {reason}")

    def _publish_status(self):
        st = {"stage": self.stage, "samples": self._n_msg}
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
