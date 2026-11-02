"""dry_run 전용 가짜 서버 (개발 PC). 팔·Nav2·퍽 인지를 흉내 내 FSM 전체 흐름을 검증한다.

- arm_skill(ArmSkill) / navigate_to_pose(NavigateToPose) 액션 서버
- observe_box 요청 시 시나리오의 퍽 관측을 /mission/puck_observations로 발행
- fail_prob로 무작위 실패(GRASP_FAIL·DROPPED·MISMATCH)를 주입해 회복 경로를 시험
실제 로봇에서는 절대 실행하지 않는다.
"""
import math
import random
import threading
import time

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from nz2_interfaces.action import ArmSkill
from nz2_interfaces.msg import PuckObservation, PuckObservationArray

BOX_XY = {0: (1468, 950), 1: (3670, 3152), 2: (2936, 5354), 3: (734, 5354), 4: (2202, 5354), 5: (3670, 3886)}


class FakeSkillServer(Node):
    def __init__(self):
        super().__init__("fake_skill_server")
        d = self.declare_parameter
        d("puck_colors", ["red"] * 6 + ["green"] * 6)
        d("puck_shapes", ["cyl", "cube"] * 6)
        d("time_scale", 0.05)          # 실제 시간 × 이 값으로 빨리 감기
        d("nav_speed_mps", 0.25)
        d("arm_time_s", 7.0)
        d("fail_prob", 0.0)
        d("obs_conf", 0.95)
        d("seed", 1)
        self.rng = random.Random(self.p("seed"))
        self.lock = threading.Lock()
        self.pos = (734.0, 950.0)
        cg = ReentrantCallbackGroup()
        self.pub_obs = self.create_publisher(PuckObservationArray, "/mission/puck_observations", 10)
        ActionServer(self, ArmSkill, "arm_skill", self._arm, callback_group=cg)
        ActionServer(self, NavigateToPose, "navigate_to_pose", self._nav, callback_group=cg)
        self.get_logger().warn("FAKE 서버 실행 중 — 실제 로봇에서 사용 금지")

    def p(self, n):
        return self.get_parameter(n).value

    def _sleep(self, s):
        time.sleep(max(0.01, s * self.p("time_scale")))

    def _nav(self, gh):
        x = gh.request.pose.pose.position.x * 1000.0
        y = gh.request.pose.pose.position.y * 1000.0
        dist = math.hypot(x - self.pos[0], y - self.pos[1]) * 1.35 / 1000.0
        steps = 5
        for _ in range(steps):
            fb = NavigateToPose.Feedback()
            gh.publish_feedback(fb)
            self._sleep(dist / self.p("nav_speed_mps") / steps)
        self.pos = (x, y)
        gh.succeed()
        return NavigateToPose.Result()

    def _arm(self, gh):
        req = gh.request
        self._sleep(self.p("arm_time_s"))
        res = ArmSkill.Result()
        res.success, res.detected_puck_id = True, -1
        fail = self.rng.random() < self.p("fail_prob")
        if req.skill == "observe_box":
            self._publish_box(req.box_id)
        elif req.skill == "pick":
            if fail:
                res.success, res.error_code = False, self.rng.choice(["GRASP_FAIL", "DROPPED"])
            else:
                res.detected_puck_id = req.puck_id
                res.detected_color = self.p("puck_colors")[req.puck_id]
                res.detected_shape = self.p("puck_shapes")[req.puck_id]
        elif req.skill in ("place", "unload", "load") and fail:
            res.success, res.error_code = False, "DROPPED"
        elif req.skill == "check_place":
            res.place_error_mm = self.rng.uniform(1.0, 14.0 if fail else 8.0)
        elif req.skill == "verify_location" and fail:
            res.success, res.error_code = False, "MISMATCH"
        res.message = f"fake {req.skill}"
        gh.succeed()
        return res

    def _publish_box(self, box):
        msg = PuckObservationArray()
        for pid in (2 * box, 2 * box + 1):
            o = PuckObservation()
            o.stamp = self.get_clock().now().to_msg()
            o.puck_id, o.box_id = pid, box
            o.color, o.shape = self.p("puck_colors")[pid], self.p("puck_shapes")[pid]
            o.conf_color = o.conf_shape = float(self.p("obs_conf"))
            bx, by = BOX_XY[box]
            o.x_mm, o.y_mm = bx + (-40.0 if pid % 2 == 0 else 40.0), float(by)
            msg.observations.append(o)
        self.pub_obs.publish(msg)


def main():
    rclpy.init()
    node = FakeSkillServer()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
