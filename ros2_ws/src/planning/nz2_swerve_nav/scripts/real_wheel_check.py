#!/usr/bin/env python3
# nz2_swerve_nav/scripts/real_wheel_check.py — 실물 바퀴 띄움 자동 점검 (Jetson, 6단계)
#
# 전제: 차체를 받침대에 올려 바퀴 4개가 바닥에서 떠 있음 + real_nav.launch.py mode:=bringup 실행 중
#   ros2 run nz2_swerve_nav real_wheel_check.py --ros-args -p confirm_wheels_up:=true
#
# 순서 (각 단계 동작 run_s 초 → 정지 rest_s 초). 속도는 작게 (바퀴 띄움이라도 손 조심)
#   1 앞   vx +0.05 m/s  (+x = 팔 쪽)      2 왼쪽 vy +0.05 m/s (팔 쪽을 볼 때 왼쪽)
#   3 회전 wz +0.20 rad/s (위에서 볼 때 반시계)   4 명령 끊김 시험: 앞 0.05 발행 후 갑자기 중단 → 바퀴 정지까지 시간
# 판정 (단계 끝 1 s 평균)
#   조향: r12 명령(/r12/swerve_joint_commands) vs 측정(/r12/joint_states) 차이 < 0.05 rad
#   바퀴: 측정/명령 비 ≈ +1 (부호 −1 → wheel 방향 반대, 크기 ≈ 4 또는 0.25 → 펌웨어 wv 단위가 모터축)
#   odom: /odom twist 가 명령(base_footprint)과 같은 부호·비 0.8~1.2 (바퀴가 떠 있어도 odom 은 바퀴 속도로 계산되므로 같아야 함)
# 사람이 눈으로 볼 것은 단계마다 출력함 (모듈 위치 매핑은 프로그램으로 확인 불가)
import math
import signal
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

PREFIX = ['front_left', 'front_right', 'rear_left', 'rear_right']   # URDF 이름 (실제와 180° 반대)
REAL_POS = {'front_left': '실제 오른쪽 뒤', 'front_right': '실제 왼쪽 뒤',
            'rear_left': '실제 오른쪽 앞(팔 쪽)', 'rear_right': '실제 왼쪽 앞(팔 쪽)'}

STEPS = [
    ('앞 +x (팔 쪽)', (0.05, 0.0, 0.0),
     '4개 조향이 모두 같은 방향(앞뒤 일직선), 바퀴 아랫면이 팔 반대쪽으로 돎 (= 지면이면 팔 쪽으로 감)'),
    ('왼쪽 +y', (0.0, 0.05, 0.0),
     '4개 조향이 모두 옆(좌우)으로 90°, 바퀴 아랫면이 오른쪽으로 돎 (= 지면이면 왼쪽으로 감)'),
    ('반시계 회전 +wz', (0.0, 0.0, 0.20),
     '4개 조향이 차체 중심 원의 접선(대각 45°) 방향, 위에서 볼 때 반시계로 굴러야 함'),
]


class Checker(Node):
    def __init__(self):
        super().__init__('real_wheel_check')
        self.ok = self.declare_parameter('confirm_wheels_up', False).value
        self.run_s = self.declare_parameter('run_s', 3.0).value
        self.rest_s = self.declare_parameter('rest_s', 2.0).value
        self.pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.cmd_steer, self.cmd_wheel, self.meas_steer, self.meas_wheel = {}, {}, {}, {}
        self.odom = (0.0, 0.0, 0.0)
        self.js_time = None
        self.create_subscription(JointState, '/r12/swerve_joint_commands', self._on_cmd, 10)
        self.create_subscription(JointState, '/r12/joint_states', self._on_js, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)

    def _on_cmd(self, m):
        for i, n in enumerate(m.name):
            p = n.rsplit('_', 2)[0]
            if n.endswith('_steer_joint') and i < len(m.position):
                self.cmd_steer[p] = m.position[i]
            if n.endswith('_wheel_joint') and i < len(m.velocity):
                self.cmd_wheel[p] = m.velocity[i]

    def _on_js(self, m):
        self.js_time = time.monotonic()
        for i, n in enumerate(m.name):
            p = n.rsplit('_', 2)[0]
            if n.endswith('_steer_joint') and i < len(m.position):
                self.meas_steer[p] = m.position[i]
            if n.endswith('_wheel_joint') and i < len(m.velocity):
                self.meas_wheel[p] = m.velocity[i]

    def _on_odom(self, m):
        t = m.twist.twist
        self.odom = (t.linear.x, t.linear.y, t.angular.z)

    def send(self, v):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = v
        self.pub.publish(msg)

    def hold(self, v, secs, sample_last=1.0):
        """v 를 20 Hz 로 secs 동안 발행, 마지막 sample_last 초 평균을 돌려줌."""
        acc, n = None, 0
        t0 = time.monotonic()
        while time.monotonic() - t0 < secs and rclpy.ok():
            self.send(v)
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() - t0 >= secs - sample_last:
                snap = [self.cmd_steer.get(p, math.nan) for p in PREFIX] + \
                       [self.meas_steer.get(p, math.nan) for p in PREFIX] + \
                       [self.cmd_wheel.get(p, math.nan) for p in PREFIX] + \
                       [self.meas_wheel.get(p, math.nan) for p in PREFIX] + list(self.odom)
                acc = snap if acc is None else [a + b for a, b in zip(acc, snap)]
                n += 1
        return [a / max(n, 1) for a in acc] if acc else None

    def stop(self):
        for _ in range(5):
            self.send((0.0, 0.0, 0.0))
            time.sleep(0.02)


def fold(d):
    return (d + math.pi / 2) % math.pi - math.pi / 2


def main():
    rclpy.init()
    node = Checker()
    signal.signal(signal.SIGINT, lambda *_: (node.stop(), rclpy.shutdown(), exit(1)))
    if not node.ok:
        print('⚠ 바퀴를 띄웠으면 -p confirm_wheels_up:=true 로 다시 실행')
        return
    print('토픽 대기 (최대 5 s): /r12/joint_states, /odom')
    t0 = time.monotonic()
    while (node.js_time is None) and time.monotonic() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.js_time is None:
        print('✗ /r12/joint_states 없음 → r12_opencr_bridge 로그(보드 role 탐색, en) 확인')
        return
    node.hold((0.0, 0.0, 0.0), 1.0)
    summary = []
    for name, v, look in STEPS:
        print(f'\n=== {name}  명령 vx {v[0]:+.2f} vy {v[1]:+.2f} wz {v[2]:+.2f}\n  눈으로: {look}')
        r = node.hold(v, node.run_s)
        node.hold((0.0, 0.0, 0.0), node.rest_s)
        if r is None:
            print('  ✗ 데이터 없음')
            continue
        cs, ms, cw, mw, od = r[0:4], r[4:8], r[8:12], r[12:16], r[16:19]
        print('  모듈(URDF 이름/실제 위치)        조향 명령  측정   오차   | 바퀴 명령  측정   비')
        for i, p in enumerate(PREFIX):
            e = fold(ms[i] - cs[i])
            ratio = mw[i] / cw[i] if abs(cw[i]) > 1e-3 else math.nan
            flag = []
            if abs(e) > 0.05:
                flag.append('조향 오차')
            if not math.isnan(ratio):
                if ratio < 0:
                    flag.append('바퀴 부호 반대')
                elif not 0.8 < ratio < 1.2:
                    flag.append('바퀴 단위/배율')
            summary += [(name, p, f) for f in flag]
            print(f'  {p:<11} {REAL_POS[p]:<14} {cs[i]:+6.3f} {ms[i]:+6.3f} {e:+6.3f} | '
                  f'{cw[i]:+7.3f} {mw[i]:+7.3f} {ratio:+5.2f}  {" ".join(flag) or "OK"}')
        ok_od = True
        for k, (c, o) in enumerate(zip(v, od)):
            if abs(c) > 1e-6:
                rr = o / c
                ok_od &= 0.8 < rr < 1.2
                print(f'  odom {"vx vy wz".split()[k]}: 명령 {c:+.3f} / odom {o:+.3f}  비 {rr:+.2f}')
        if not ok_od:
            summary.append((name, 'odom', '부호/배율'))

    # 명령 끊김 시험: cmd_frame_adapter input_timeout 0.25 s → r12 timeout 0.25 → 브리지 0.25 (최악 ~0.75 s)
    print('\n=== 명령 끊김 시험: 앞 0.05 m/s 3 s 뒤 발행 중단 (이 스크립트가 멈춘 것처럼)')
    node.hold((0.05, 0.0, 0.0), 3.0)
    t_cut = time.monotonic()
    t_stop = None
    while time.monotonic() - t_cut < 3.0:
        rclpy.spin_once(node, timeout_sec=0.02)
        if t_stop is None and all(abs(node.meas_wheel.get(p, 0.0)) < 0.2 for p in PREFIX):
            t_stop = time.monotonic() - t_cut
    if t_stop is None:
        print('  ✗ 3 s 안에 바퀴가 안 섬 → 대회 투입 금지. 브리지 cmd_timeout_s·펌웨어 워치독 확인')
        summary.append(('끊김', '전체', '정지 안 함'))
    else:
        print(f'  ✓ 끊긴 뒤 {t_stop:.2f} s 에 바퀴 < 0.2 rad/s ({"OK" if t_stop < 1.0 else "느림: 1 s 초과"})')
        if t_stop >= 1.0:
            summary.append(('끊김', '전체', f'{t_stop:.2f} s'))
    node.stop()
    print('\n==== 요약 ====')
    if not summary:
        print('자동 판정 전부 OK. 눈으로 본 방향도 맞으면 지면 시험으로')
    for s in summary:
        print(f'  ✗ {s[0]} / {s[1]}: {s[2]}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
