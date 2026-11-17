#!/usr/bin/env python3
"""오도메트리 정확도 점검 (Gazebo 정답 자세 vs /odom)

Gazebo 의 실제 로봇 자세(sim_nav.launch.py 가 넣는 OdometryPublisher → /model/robot/ground_truth)와 /odom 을
시작 시점 기준 상대 이동량으로 비교해 1 s 마다 출력. 끝(Ctrl-C)에 요약.

  오차가 회전할 때 커지면  → 회전 오도메트리(조향 정렬 전 바퀴 회전에 의한 미끄럼, 기하 오차)
  직진·게걸음에서 비례해 커지면 → 바퀴 반지름·부호 등 스케일 오차

사용: (sim_nav.launch.py 실행 중) ros2 run nz2_swerve_nav odom_check.py --ros-args -p use_sim_time:=true
파라미터: gt_topic(/model/robot/ground_truth), odom_topic(/odom)
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def rel(p0, p):
    """p0 기준 상대 자세 (dx, dy: p0 좌표계, dyaw)"""
    dx, dy = p[0] - p0[0], p[1] - p0[1]
    c, s = math.cos(-p0[2]), math.sin(-p0[2])
    return (c * dx - s * dy, s * dx + c * dy, wrap(p[2] - p0[2]))


class OdomCheck(Node):
    def __init__(self):
        super().__init__('odom_check')
        gt_topic = self.declare_parameter('gt_topic', '/model/robot/ground_truth').value
        odom_topic = self.declare_parameter('odom_topic', '/odom').value
        self.gt = self.od = None
        self.gt0 = self.od0 = None
        self.gt_yaw_unwrap = self.od_yaw_unwrap = 0.0
        self.prev_gt_yaw = self.prev_od_yaw = None
        self.dist = 0.0
        self.prev_gt_xy = None
        self.max_pos_err = self.max_yaw_err = 0.0
        self.create_subscription(Odometry, gt_topic, self.on_gt, 10)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        self.create_timer(1.0, self.report)
        self.get_logger().info(f'정답 {gt_topic} vs {odom_topic}. Ctrl-C 로 요약')

    def on_gt(self, msg):
        p = msg.pose.pose
        self.gt = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_odom(self, msg):
        p = msg.pose.pose
        self.od = (p.position.x, p.position.y, yaw_of(p.orientation))

    def report(self):
        if self.gt is None or self.od is None:
            self.get_logger().warning(f'수신 대기: 정답={"O" if self.gt else "X"} odom={"O" if self.od else "X"}'
                                   ' (정답 X → ros2 topic hz /model/robot/ground_truth, 런치 재시작 확인)')
            return
        if self.gt0 is None:
            self.gt0, self.od0 = self.gt, self.od
            self.prev_gt_yaw, self.prev_od_yaw = self.gt[2], self.od[2]
            self.prev_gt_xy = self.gt[:2]
            self.get_logger().info('기준 자세 저장. 이제 움직이세요')
            return
        # 누적 회전량(감기 풀기)과 이동 거리
        self.gt_yaw_unwrap += wrap(self.gt[2] - self.prev_gt_yaw)
        self.od_yaw_unwrap += wrap(self.od[2] - self.prev_od_yaw)
        self.prev_gt_yaw, self.prev_od_yaw = self.gt[2], self.od[2]
        self.dist += math.dist(self.gt[:2], self.prev_gt_xy)
        self.prev_gt_xy = self.gt[:2]

        g = rel(self.gt0, self.gt)
        o = rel(self.od0, self.od)
        ep = math.hypot(g[0] - o[0], g[1] - o[1])
        ey = wrap(g[2] - o[2])
        self.max_pos_err = max(self.max_pos_err, ep)
        self.max_yaw_err = max(self.max_yaw_err, abs(ey))
        self.get_logger().info(
            f'정답 x {g[0]:+.3f} y {g[1]:+.3f} yaw {math.degrees(g[2]):+7.1f}° | '
            f'odom x {o[0]:+.3f} y {o[1]:+.3f} yaw {math.degrees(o[2]):+7.1f}° | '
            f'오차 위치 {ep * 1000:5.0f} mm, yaw {math.degrees(ey):+6.1f}° | '
            f'누적 회전 정답 {math.degrees(self.gt_yaw_unwrap):+.0f}° odom {math.degrees(self.od_yaw_unwrap):+.0f}°, 이동 {self.dist:.2f} m')

    def summary(self):
        if self.gt0 is None:
            return
        ratio = (self.od_yaw_unwrap / self.gt_yaw_unwrap) if abs(self.gt_yaw_unwrap) > 0.3 else float('nan')
        print('\n===== 요약 =====')
        print(f'이동 거리(정답) {self.dist:.2f} m, 최대 위치 오차 {self.max_pos_err * 1000:.0f} mm, 최대 yaw 오차 {math.degrees(self.max_yaw_err):.1f}°')
        print(f'누적 회전 정답 {math.degrees(self.gt_yaw_unwrap):+.0f}°, odom {math.degrees(self.od_yaw_unwrap):+.0f}° '
              f'→ odom/정답 비 {ratio:.3f} (1.000 이 이상적)')
        g = rel(self.gt0, self.gt)
        o = rel(self.od0, self.od)
        dg, do = math.hypot(g[0], g[1]), math.hypot(o[0], o[1])
        if dg > 0.3 and abs(self.gt_yaw_unwrap) < math.radians(5):
            print(f'직선 변위 정답 {dg:.3f} m, odom {do:.3f} m → odom/정답 비 {do / dg:.3f} (회전 없는 시험만 유효)')


def main():
    rclpy.init()
    n = OdomCheck()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            n.summary()
        except Exception as e:  # Ctrl-C 를 두 번 누르는 등
            print(f'요약 출력 실패: {e!r}')
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
