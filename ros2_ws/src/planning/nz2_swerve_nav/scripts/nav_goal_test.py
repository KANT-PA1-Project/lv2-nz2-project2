#!/usr/bin/env python3
# nz2_swerve_nav/scripts/nav_goal_test.py
# 5단계 Nav2 시험: nav_test_goals.yaml 의 경로(route) 목표를 차례로 NavigateToPose 로 보내고 결과를 집계
#
#   목표마다: 성공 여부, 걸린 시간, 복구 횟수, 최대 속도,
#             벽까지 최소 여유(/scan 점 ↔ /swerve/footprint 다각형, 실격 위험 지표),
#             최종 오차 = AMCL(/amcl_pose) − 목표, 정답(/model/robot/ground_truth, 시뮬만) − 목표, AMCL − 정답
#   결과: 화면 표 + ~/nz2_nav_results/<시각>_<route>.json
#
#   ros2 run nz2_swerve_nav nav_goal_test.py --ros-args -p route:=basic -p use_sim_time:=true
#   파라미터: goals_file(비우면 패키지 config/nav_test_goals.yaml), route(basic|facing|all), repeat(1),
#            goal_timeout(150 s), gt_topic(/model/robot/ground_truth, 비우면 정답 비교 안 함),
#            spawn_x·spawn_y·spawn_yaw(정답 원점 판정용, 기본 출발점), warn_clearance(0.03 m)
#            max_start_err(0.15 m), max_start_err_deg(10°): 시작 전 AMCL−정답 차이가 이보다 크면 시험 거부 (실격 방지)
#            reset_amcl_to_truth(false): true 면 거부 대신 정답 자세로 /initialpose 를 보내 AMCL 재설정 (시뮬 전용)
#   Ctrl+C: 진행 중 목표 취소(→ 컨트롤러 정지) 후 그때까지 결과 저장
import json
import math
import os
import time

import numpy as np
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PolygonStamped, PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger

STATUS = {GoalStatus.STATUS_SUCCEEDED: '성공', GoalStatus.STATUS_ABORTED: '중단(실패)',
          GoalStatus.STATUS_CANCELED: '취소'}


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def poly_clearance(px, py, poly):
    """점들(px,py) 과 다각형(poly Nx2) 사이 최소 거리. 다각형 안의 점은 0"""
    if len(px) == 0 or len(poly) < 3:
        return float('inf')
    a = poly
    b = np.roll(poly, -1, axis=0)
    d = np.full(len(px), np.inf)
    inside = np.zeros(len(px), dtype=bool)
    for (ax, ay), (bx, by) in zip(a, b):
        ex, ey = bx - ax, by - ay
        t = np.clip(((px - ax) * ex + (py - ay) * ey) / max(ex * ex + ey * ey, 1e-12), 0.0, 1.0)
        d = np.minimum(d, np.hypot(px - (ax + t * ex), py - (ay + t * ey)))
        # 홀짝 규칙 (내부 판정)
        cond = ((ay > py) != (by > py)) & (px < (bx - ax) * (py - ay) / np.where(by - ay == 0, 1e-12, by - ay) + ax)
        inside ^= cond
    d[inside] = 0.0
    return float(d.min())


class NavGoalTest(Node):
    def __init__(self):
        super().__init__('nav_goal_test')
        d = self.declare_parameter
        default_goals = os.path.join(get_package_share_directory('nz2_swerve_nav'), 'config', 'nav_test_goals.yaml')
        self.goals_file = os.path.expanduser(d('goals_file', '').value or default_goals)
        self.route = d('route', 'basic').value
        self.repeat = int(d('repeat', 1).value)
        self.goal_timeout = float(d('goal_timeout', 150.0).value)
        self.gt_topic = d('gt_topic', '/model/robot/ground_truth').value
        self.spawn = (float(d('spawn_x', 0.734).value), float(d('spawn_y', 0.950).value),
                      float(d('spawn_yaw', 0.0).value))
        self.warn_clr = float(d('warn_clearance', 0.03).value)
        self.max_start_err = float(d('max_start_err', 0.15).value)
        self.max_start_err_deg = float(d('max_start_err_deg', 10.0).value)
        self.reset_amcl = bool(d('reset_amcl_to_truth', False).value)

        with open(self.goals_file, encoding='utf-8') as f:
            routes = yaml.safe_load(f)['routes']
        if self.route not in routes:
            raise RuntimeError(f'route "{self.route}" 없음. 가능: {list(routes)}')
        self.goals = routes[self.route] * self.repeat

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.amcl = None
        self.gt = None
        self.gt_offset = None        # 정답 좌표 → mission 좌표 변환 (x, y, yaw)
        self.footprint = np.zeros((0, 2))
        self.fp_frame = ''
        self.leg = None              # 진행 중 목표 통계
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl, latched)
        self.pub_init = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.create_subscription(PolygonStamped, 'swerve/footprint', self._on_fp, latched)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        if self.gt_topic:
            self.create_subscription(Odometry, self.gt_topic, self._on_gt, 10)

    # ── 구독 ──
    def _on_amcl(self, m):
        p = m.pose.pose
        self.amcl = (p.position.x, p.position.y, yaw_of(p.orientation))

    def _on_fp(self, m):
        self.footprint = np.array([[p.x, p.y] for p in m.polygon.points])
        self.fp_frame = m.header.frame_id

    def _on_scan(self, m):
        if self.leg is None or len(self.footprint) < 3:
            return
        if self.fp_frame and m.header.frame_id and m.header.frame_id != self.fp_frame:
            if not self.leg.get('frame_warned'):
                self.get_logger().warning(f'scan frame {m.header.frame_id} ≠ footprint frame {self.fp_frame} → 여유 계산 생략')
                self.leg['frame_warned'] = True
            return
        r = np.asarray(m.ranges, dtype=float)
        ang = m.angle_min + np.arange(len(r)) * m.angle_increment
        ok = np.isfinite(r) & (r > m.range_min) & (r < min(m.range_max, 1.0))  # 1 m 안만 (속도)
        c = poly_clearance(r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok]), self.footprint)
        self.leg['min_clearance'] = min(self.leg['min_clearance'], c)

    def _on_odom(self, m):
        if self.leg is None:
            return
        v = m.twist.twist
        self.leg['max_v'] = max(self.leg['max_v'], math.hypot(v.linear.x, v.linear.y))
        self.leg['max_wz'] = max(self.leg['max_wz'], abs(v.angular.z))

    def _on_gt(self, m):
        p = m.pose.pose
        raw = (p.position.x, p.position.y, yaw_of(p.orientation))
        if self.gt_offset is None:
            # gz OdometryPublisher 가 월드 좌표인지, 스폰 위치 원점인지 첫 값으로 판정
            sx, sy, syaw = self.spawn
            if math.hypot(raw[0] - sx, raw[1] - sy) < 0.15:
                self.gt_offset = (0.0, 0.0, 0.0)
                self.get_logger().info('정답 자세 = 월드(mission) 좌표로 판정')
            elif math.hypot(raw[0], raw[1]) < 0.05:
                self.gt_offset = self.spawn
                self.get_logger().info(f'정답 자세 = 스폰 원점 좌표로 판정 → 스폰 {self.spawn} 더함')
            else:
                self.gt_offset = (0.0, 0.0, 0.0)
                self.get_logger().warning(f'정답 첫 값 {raw[:2]} 가 스폰·원점 어느 쪽도 아님 → 그대로 사용 (확인 필요)')
        ox, oy, oyaw = self.gt_offset
        c, s = math.cos(oyaw), math.sin(oyaw)
        self.gt = (ox + c * raw[0] - s * raw[1], oy + s * raw[0] + c * raw[1], wrap(oyaw + raw[2]))

    # ── 실행 ──
    def spin_for(self, sec):
        end = time.monotonic() + sec
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _nav2_active(self, timeout=90.0):
        """lifecycle_manager 두 개가 모두 활성인지 (is_active 서비스). Nav2 가 덜 떴는데 목표를 보내면 전부 '거부'"""
        for mgr in ('lifecycle_manager_localization', 'lifecycle_manager_navigation'):
            cli = self.create_client(Trigger, f'/{mgr}/is_active')
            t0 = time.monotonic()
            ok = False
            while rclpy.ok() and time.monotonic() - t0 < timeout:
                if cli.wait_for_service(timeout_sec=1.0):
                    fut = cli.call_async(Trigger.Request())
                    rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
                    if fut.done() and fut.result() is not None and fut.result().success:
                        ok = True
                        break
                self.get_logger().info(f'{mgr} 활성 대기 중...', throttle_duration_sec=10.0)
                self.spin_for(1.0)
            if not ok:
                self.get_logger().error(
                    f'{mgr} 가 {timeout:.0f} s 안에 활성화 안 됨 → 시험 중단. 터미널 1 로그에서 '
                    "'Managed nodes are active' 2번 확인, 없으면 launch 재시작")
                return False
        self.get_logger().info('Nav2 활성 확인 (위치추정·내비게이션)')
        return True

    def run(self):
        self.get_logger().info(f'목표 파일 {self.goals_file}, 경로 "{self.route}" {len(self.goals)}개')
        if not self._nav2_active():
            return
        if not self.client.wait_for_server(timeout_sec=60.0):
            raise RuntimeError('navigate_to_pose 액션 서버 없음 (Nav2 활성화 확인: ros2 lifecycle get /bt_navigator)')
        t0 = time.monotonic()
        while rclpy.ok() and self.amcl is None and time.monotonic() - t0 < 30.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.amcl is None:
            self.get_logger().warning('/amcl_pose 아직 없음 (로봇이 움직이면 나옴) → 계속')
        if not self._check_start():
            return
        results = []
        try:
            for i, g in enumerate(self.goals, 1):
                results.append(self.run_goal(i, g))
                self.spin_for(2.0)   # 정지 후 AMCL 수렴·정답 비교 안정화
                self._finish(results[-1])
                self._print_row(results[-1])
        except KeyboardInterrupt:
            self.get_logger().warning('Ctrl+C → 목표 취소')
            if self._gh is not None:
                self._gh.cancel_goal_async()
                try:
                    self.spin_for(1.0)
                except KeyboardInterrupt:
                    pass
        self._report(results)

    _gh = None

    def _amcl_vs_truth(self):
        if self.amcl is None or self.gt is None:
            return None
        return (math.hypot(self.amcl[0] - self.gt[0], self.amcl[1] - self.gt[1]),
                abs(math.degrees(wrap(self.amcl[2] - self.gt[2]))))

    def _check_start(self):
        """시작 전 AMCL 이 실제 위치와 맞는지 확인. 틀린 위치로 주행하면 '빈 곳'이 실제로는 벽 → 실격"""
        if not self.gt_topic:
            return True
        t0 = time.monotonic()
        while rclpy.ok() and self.gt is None and time.monotonic() - t0 < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        e = self._amcl_vs_truth()
        if e is None:
            if self.amcl is None:
                self.get_logger().error('/amcl_pose 없음 → 시험 중단 (AMCL 이 활성인지, /scan 이 나오는지 확인)')
                return False
            self.get_logger().warning('정답 자세 없음 → 시작 위치 검사 생략')
            return True
        self.get_logger().info(f'시작 검사: AMCL {tuple(round(v, 3) for v in self.amcl)} / 정답 {tuple(round(v, 3) for v in self.gt)} '
                               f'→ 차이 {e[0] * 1000:.0f} mm, {e[1]:.1f}°')
        if e[0] <= self.max_start_err and e[1] <= self.max_start_err_deg:
            return True
        if not self.reset_amcl:
            self.get_logger().error(
                f'AMCL 위치가 실제와 {e[0] * 1000:.0f} mm / {e[1]:.1f}° 다름 → 시험 거부. '
                'RViz 2D Pose Estimate 를 잘못 눌렀을 가능성. launch 재시작 또는 -p reset_amcl_to_truth:=true (시뮬 전용)')
            return False
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'          # stamp 0 = 최신 TF 사용 (미래 외삽 경고 방지)
        m.pose.pose.position.x, m.pose.pose.position.y = self.gt[0], self.gt[1]
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(self.gt[2] / 2), math.cos(self.gt[2] / 2)
        m.pose.covariance[0] = m.pose.covariance[7] = 0.01 ** 2
        m.pose.covariance[35] = math.radians(2.0) ** 2
        self.get_logger().warning('정답 자세로 AMCL 재설정 (/initialpose)')
        self.pub_init.publish(m)
        self.amcl = None
        self.spin_for(3.0)
        e = self._amcl_vs_truth()
        if e is None or e[0] > self.max_start_err or e[1] > self.max_start_err_deg:
            self.get_logger().error(f'재설정 후에도 불일치 {e} → 시험 거부')
            return False
        self.get_logger().info(f'재설정 완료: 차이 {e[0] * 1000:.0f} mm, {e[1]:.1f}°')
        return True

    def run_goal(self, i, g):
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        yaw = math.radians(float(g['yaw_deg']))
        ps.pose.position.x, ps.pose.position.y = float(g['x']), float(g['y'])
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        goal.pose = ps
        self.get_logger().info(f'[{i}/{len(self.goals)}] {g["name"]} → ({g["x"]:.3f}, {g["y"]:.3f}, {g["yaw_deg"]}°)')
        self.leg = {'name': g['name'], 'goal': [float(g['x']), float(g['y']), float(g['yaw_deg'])],
                    'min_clearance': float('inf'), 'max_v': 0.0, 'max_wz': 0.0, 'recoveries': 0}
        fb = {'n': 0}

        def on_fb(m):
            fb['n'] = m.feedback.number_of_recoveries

        fut = self.client.send_goal_async(goal, feedback_callback=on_fb)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.leg.update(status='거부', time_s=0.0)
            leg, self.leg = self.leg, None
            return leg
        self._gh = gh
        t0 = time.monotonic()
        res_fut = gh.get_result_async()
        while rclpy.ok() and not res_fut.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() - t0 > self.goal_timeout:
                self.get_logger().warning(f'{self.goal_timeout:.0f} s 초과 → 취소')
                gh.cancel_goal_async()
                self.spin_for(1.0)
                break
        status = res_fut.result().status if res_fut.done() else GoalStatus.STATUS_CANCELED
        err = ''
        if res_fut.done():
            r = res_fut.result().result
            err = f'{getattr(r, "error_code", "")} {getattr(r, "error_msg", "")}'.strip()
        self._gh = None
        self.leg.update(status=STATUS.get(status, str(status)), time_s=round(time.monotonic() - t0, 1),
                        recoveries=fb['n'], error=err)
        return self.leg

    def _finish(self, leg):
        gx, gy, gyaw = leg['goal'][0], leg['goal'][1], math.radians(leg['goal'][2])
        if self.amcl:
            ax, ay, ayaw = self.amcl
            leg['amcl_err_mm'] = round(1000 * math.hypot(ax - gx, ay - gy), 1)
            leg['amcl_err_deg'] = round(math.degrees(wrap(ayaw - gyaw)), 2)
        if self.gt:
            tx, ty, tyaw = self.gt
            leg['true_err_mm'] = round(1000 * math.hypot(tx - gx, ty - gy), 1)
            leg['true_err_deg'] = round(math.degrees(wrap(tyaw - gyaw)), 2)
            if self.amcl:
                leg['amcl_vs_true_mm'] = round(1000 * math.hypot(self.amcl[0] - tx, self.amcl[1] - ty), 1)
                leg['amcl_vs_true_deg'] = round(math.degrees(wrap(self.amcl[2] - tyaw)), 2)
        c = leg['min_clearance']
        leg['min_clearance_mm'] = None if math.isinf(c) else round(1000 * c, 1)
        leg.pop('min_clearance', None)
        leg.pop('frame_warned', None)
        self.leg = None

    def _print_row(self, r):
        f = lambda k, u='': '-' if r.get(k) is None else f'{r[k]}{u}'  # noqa: E731
        warn = ''
        if r.get('min_clearance_mm') is not None and r['min_clearance_mm'] < 1000 * self.warn_clr:
            warn = '  ⚠ 벽 근접'
        self.get_logger().info(
            f'  → {r["status"]}  {r.get("time_s", 0)} s  복구 {r.get("recoveries", 0)}  '
            f'최소 여유 {f("min_clearance_mm", " mm")}  최대 {r["max_v"]:.2f} m/s {r["max_wz"]:.2f} rad/s | '
            f'AMCL 오차 {f("amcl_err_mm", " mm")} {f("amcl_err_deg", "°")} | '
            f'정답 오차 {f("true_err_mm", " mm")} {f("true_err_deg", "°")} | '
            f'AMCL−정답 {f("amcl_vs_true_mm", " mm")} {f("amcl_vs_true_deg", "°")}{warn}'
            + (f'  [{r["error"]}]' if r.get('error') and r['status'] != '성공' else ''))

    def _report(self, results):
        n_ok = sum(r['status'] == '성공' for r in results)
        self.get_logger().info(f'=== 경로 "{self.route}": 성공 {n_ok}/{len(results)} ===')
        clr = [r['min_clearance_mm'] for r in results if r.get('min_clearance_mm') is not None]
        tru = [r['true_err_mm'] for r in results if r.get('true_err_mm') is not None]
        avt = [r['amcl_vs_true_mm'] for r in results if r.get('amcl_vs_true_mm') is not None]
        if clr:
            self.get_logger().info(f'벽까지 최소 여유: 최저 {min(clr)} mm (scan_merger 가 몸 바깥 10 mm 안 점을 지우므로 ≈10 mm = 접촉 직전)')
        if tru:
            self.get_logger().info(f'실제 정지 오차(정답): 평균 {np.mean(tru):.1f} mm, 최대 {max(tru)} mm')
        if avt:
            self.get_logger().info(f'AMCL 위치 오차(정답 대비): 평균 {np.mean(avt):.1f} mm, 최대 {max(avt)} mm')
        out_dir = os.path.expanduser('~/nz2_nav_results')
        os.makedirs(out_dir, exist_ok=True)
        path = os.path.join(out_dir, time.strftime('%Y%m%d_%H%M%S') + f'_{self.route}.json')
        with open(path, 'w', encoding='utf-8') as f:
            json.dump({'route': self.route, 'goals_file': self.goals_file, 'results': results}, f,
                      ensure_ascii=False, indent=1)
        self.get_logger().info(f'저장: {path}')


def main():
    # Ctrl+C 를 rclpy 가 가로채면 컨텍스트가 먼저 닫혀 목표 취소를 못 보냄 → 직접 KeyboardInterrupt 로 받음
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = NavGoalTest()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
