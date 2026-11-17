#!/usr/bin/env python3
# nz2_swerve_nav/scripts/cmd_frame_adapter.py
# 실행 위치: 개발 PC(시뮬) / Jetson(실물)
#
# 역할: Nav2·텔레옵 /cmd_vel (TwistStamped, base_footprint, +x = 팔 쪽)
#       → 팀 r12_swerve_controller 입력 /r12/cmd_vel (TwistStamped, base_link, +x = base_link 앞)
#   - 좌표 변환: v_bl = R(-base_yaw) · v_fp  (base_yaw = π → (vx, vy) 부호 반전, wz 그대로)
#   - frame_id = base_link, stamp = 현재 시각 (r12 는 |now − stamp| > 0.25 s 면 버림)
#   - 입력 frame 이 accepted_frames 에 없으면 버림, NaN/Inf 버림
#   - 입력이 input_timeout 넘게 끊기면 0 발행
#   - /estop (std_msgs/Bool) true → 0 래치. 해제 = false 수신 + 입력이 0(또는 끊김)일 때만
#   - output_rate 로 계속 발행 (정지 중에도 0)
#
# (11/14 v2) 정렬 게이트 — 팀 r12 코드는 그대로 두고 여기서 흉내 냄
#   문제: r12 는 조향이 목표에 닿기 전에도 바퀴를 돌림 (측정 조향각을 안 봄)
#         → 방향 전환·±90° 경계 뒤집기 때마다 미끄러짐, 오도메트리·AMCL 틀어짐,
#           저속에서 명령 방향이 흔들리면 '조향만 계속 바뀌고 안 감' (facing 시험 L3 에서 151 s 정체)
#   해결 1) 데드밴드: |병진| < lin_deadband 이고 |wz| < ang_deadband 면 0 → r12 는 조향 유지, 바퀴 정지
#           (v4) 회전 중 |병진| < rot_lin_deadband 이면 병진 0 = 순수 회전 (조향 ±45° 고정)
#           회전 바퀴 속도(0.1 rad/s × 0.175 m = 0.018 m/s)와 잡음 병진(0.01 m/s)이 비슷하면 모듈 목표각이
#           매 순간 크게 바뀌어 게이트가 계속 걸림 → 목표에서 마지막 회전이 거의 안 됨 (facing 4차 bag)
#           (r12 HOLD_SPEED 1e-4 m/s 라 아주 작은 잡음 명령에도 조향을 다시 돌림)
#   해결 2) 정렬 게이트: 명령으로 계산한 모듈 목표각과 측정 조향각(joint_states) 차이(180° 접기)가
#           align_stop 보다 크면 → 전체 명령을 '가장 빠른 모듈이 align_creep_speed' 가 되도록 축소
#           (방향은 유지 → r12 는 계속 조향을 돌리고, 바퀴는 거의 정지)
#           그 아래에서는 cos(오차) 배 (WPILib CosineScale)
#           측정이 joint_state_timeout 넘게 없으면 게이트 끔 (경고)
#           배율은 내려갈 땐 즉시, 올라갈 땐 align_scale_rise [1/s] 로 천천히 (정렬 직후 급가속 방지)
#   (11/14 v3) ⚠ r12 의 가속 램프(command_acceleration_m_s2)는 반드시 0 으로 끌 것 (r12_*_overrides.yaml)
#           램프가 있으면 r12 가 병진만 늦게 줄이고 회전은 바로 줄여 → r12 가 실제로 향하는 조향각이
#           게이트가 기다리는 각과 달라짐 → 게이트가 영원히 안 풀림 (facing 2차: 회전 못 하고 150 s 정체)
#           가속 제한은 velocity_smoother(0.5 m/s²) 가 이미 함
#
# ⚠ r12 는 가속 램프(시뮬 0.5 m/s²)가 있어 0 을 보내도 바로 서지 않음
#   실물 비상정지는 반드시 하드웨어(모터 12 V 차단)로.
import math

# ---------- 순수 로직 (ROS 없이 시험 가능) ----------


def fp_to_bl(vx, vy, wz, base_yaw):
    """base_footprint 속도 → base_link 속도 (z 축 회전만 다름)."""
    c, s = math.cos(-base_yaw), math.sin(-base_yaw)
    return (c * vx - s * vy, s * vx + c * vy, wz)


def fold_err(a, b):
    """조향 a 와 b 의 차이, 180° 뒤집기 허용 → [0, π/2]."""
    d = (a - b + math.pi / 2) % math.pi - math.pi / 2
    return abs(d)


def module_targets(vx, vy, wz, positions):
    """base_link 속도 → 모듈별 (목표 각, 속도). 속도 ~0 이면 각 None."""
    out = []
    for (x, y) in positions:
        mvx, mvy = vx - wz * y, vy + wz * x
        sp = math.hypot(mvx, mvy)
        out.append((math.atan2(mvy, mvx) if sp > 1e-6 else None, sp))
    return out


class AdapterCore:
    ZERO_EPS = 1e-3

    def __init__(self, base_yaw=math.pi, input_timeout=0.25, accepted_frames=('base_footprint', ''),
                 positions=((0.124, 0.124), (0.124, -0.124), (-0.124, 0.124), (-0.124, -0.124)),
                 lin_deadband=0.01, ang_deadband=0.02,
                 gate_enable=True, align_stop=0.35, align_creep_speed=0.003, js_timeout=0.3,
                 scale_rise=2.0, rot_lin_deadband=0.02):
        self.base_yaw = base_yaw
        self.input_timeout = input_timeout
        self.accepted = tuple(accepted_frames)
        self.positions = [tuple(p) for p in positions]
        self.lin_db, self.ang_db = lin_deadband, ang_deadband
        self.gate_enable = gate_enable
        self.align_stop = align_stop
        self.creep = align_creep_speed
        self.js_timeout = js_timeout
        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_time = None
        self.estop = False
        self.release_req = False
        self.meas = None            # 측정 조향각 4개 (positions 순서)
        self.meas_time = None
        self.scale_rise = scale_rise
        self.rot_lin_db = rot_lin_deadband
        self.k_out = 1.0            # 실제로 곱한 배율 (상승 속도 제한)
        self.t_prev = None
        self.last_err = 0.0         # 진단용
        self.last_scale = 1.0

    def on_cmd(self, frame_id, v, t):
        """입력 1개 처리. 반환: None(수락) 또는 거부 사유 문자열."""
        if frame_id not in self.accepted:
            return f'frame "{frame_id}" 거부 (허용 {list(self.accepted)})'
        if not all(math.isfinite(x) for x in v):
            return 'NaN/Inf 거부'
        self.cmd = tuple(float(x) for x in v)
        self.cmd_time = t
        return None

    def on_steer(self, angles, t):
        if len(angles) == len(self.positions) and all(math.isfinite(a) for a in angles):
            self.meas = list(angles)
            self.meas_time = t

    def on_estop(self, active):
        if active:
            self.estop = True
            self.release_req = False
        else:
            self.release_req = True

    def stale(self, t):
        return self.cmd_time is None or (t - self.cmd_time) > self.input_timeout

    def meas_ok(self, t):
        return self.meas is not None and (t - self.meas_time) <= self.js_timeout

    def gate(self, vx, vy, wz, t):
        """정렬 게이트. 반환 (vx, vy, wz) 축소본."""
        self.last_err, self.last_scale = 0.0, 1.0
        if not self.gate_enable or not self.meas_ok(t):
            return vx, vy, wz
        tg = module_targets(vx, vy, wz, self.positions)
        errs = [fold_err(a, m) for (a, _), m in zip(tg, self.meas) if a is not None]
        if not errs:
            return vx, vy, wz
        err = max(errs)
        top = max(sp for _, sp in tg)
        if err > self.align_stop:
            k = min(1.0, self.creep / top) if top > 1e-9 else 0.0
        else:
            k = math.cos(err)
        # 내려갈 땐 즉시, 올라갈 땐 천천히
        dt = 0.0 if self.t_prev is None else max(0.0, t - self.t_prev)
        self.t_prev = t
        k = min(k, self.k_out + self.scale_rise * dt) if k > self.k_out else k
        self.k_out = k
        self.last_err, self.last_scale = err, k
        return vx * k, vy * k, wz * k

    def output(self, t):
        """base_link 기준 (vx, vy, wz). 정지 조건이면 0."""
        stale = self.stale(t)
        if self.estop and self.release_req:
            if stale or all(abs(x) < self.ZERO_EPS for x in self.cmd):
                self.estop = False
            self.release_req = False
        if self.estop or stale:
            self.last_err, self.last_scale = 0.0, 1.0
            return (0.0, 0.0, 0.0)
        vx, vy, wz = fp_to_bl(*self.cmd, self.base_yaw)
        lin = math.hypot(vx, vy)
        if lin < self.lin_db and abs(wz) < self.ang_db:
            self.last_err, self.last_scale = 0.0, 1.0
            return (0.0, 0.0, 0.0)
        if abs(wz) >= self.ang_db and lin < self.rot_lin_db:
            vx = vy = 0.0                    # 순수 회전
        return self.gate(vx, vy, wz, t)


# ---------- ROS 노드 ----------


def main(args=None):
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, Float64MultiArray

    class CmdFrameAdapter(Node):
        def __init__(self):
            super().__init__('cmd_frame_adapter')
            d = self.declare_parameter
            mx = d('module_x', [0.124, 0.124, -0.124, -0.124]).value
            my = d('module_y', [0.124, -0.124, 0.124, -0.124]).value
            self.steer_names = list(d('steer_joints', ['front_left_steer_joint', 'front_right_steer_joint',
                                                      'rear_left_steer_joint', 'rear_right_steer_joint']).value)
            self.core = AdapterCore(
                base_yaw=d('base_yaw', math.pi).value,
                input_timeout=d('input_timeout', 0.25).value,
                accepted_frames=d('accepted_frames', ['base_footprint', '']).value,
                positions=list(zip(mx, my)),
                lin_deadband=d('lin_deadband', 0.01).value,
                ang_deadband=d('ang_deadband', 0.02).value,
                gate_enable=d('align_gate', True).value,
                align_stop=d('align_stop', 0.35).value,
                align_creep_speed=d('align_creep_speed', 0.003).value,
                js_timeout=d('joint_state_timeout', 0.3).value,
                scale_rise=d('align_scale_rise', 2.0).value,
                rot_lin_deadband=d('rot_lin_deadband', 0.02).value,
            )
            js_topic = d('joint_state_topic', '/joint_states').value   # 실물: /r12/joint_states
            self.out_frame = d('output_frame', 'base_link').value
            rate = d('output_rate', 50.0).value
            self.pub = self.create_publisher(TwistStamped, 'r12/cmd_vel', 10)
            self.pub_diag = self.create_publisher(Float64MultiArray, 'cmd_frame_adapter/gate', 10)
            self.create_subscription(TwistStamped, 'cmd_vel', self._on_cmd, 10)
            self.create_subscription(JointState, js_topic, self._on_js, qos_profile_sensor_data)
            self.create_subscription(Bool, 'estop', self._on_estop,
                                     QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
            self.create_timer(1.0 / rate, self._tick)
            self.was_moving = False
            self.gate_on = False
            self.get_logger().info(
                f'cmd_frame_adapter: /cmd_vel(base_footprint) → /r12/cmd_vel({self.out_frame}), '
                f'base_yaw {self.core.base_yaw:.3f} rad, {rate:.0f} Hz | 정렬 게이트 '
                f'{"켬" if self.core.gate_enable else "끔"} (정지 {self.core.align_stop:.2f} rad, '
                f'측정 {js_topic}), 데드밴드 {self.core.lin_db} m/s / {self.core.ang_db} rad/s')

        def _now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def _on_cmd(self, m):
            v = (m.twist.linear.x, m.twist.linear.y, m.twist.angular.z)
            err = self.core.on_cmd(m.header.frame_id, v, self._now())
            if err:
                self.get_logger().warning(f'cmd_vel {err}', throttle_duration_sec=1.0)

        def _on_js(self, m):
            if not m.position:
                return
            idx = {n: i for i, n in enumerate(m.name)}
            if not all(n in idx for n in self.steer_names):
                return
            self.core.on_steer([m.position[idx[n]] for n in self.steer_names], self._now())

        def _on_estop(self, m):
            was = self.core.estop
            self.core.on_estop(m.data)
            if m.data and not was:
                self.get_logger().error('E-STOP 수신: /r12/cmd_vel 0 래치')

        def _tick(self):
            t = self._now()
            was_estop = self.core.estop
            vx, vy, wz = self.core.output(t)
            if was_estop and not self.core.estop:
                self.get_logger().warning('E-STOP 해제')
            moving = abs(vx) + abs(vy) + abs(wz) > 0.0
            if self.was_moving and not moving and self.core.stale(t) and not self.core.estop:
                self.get_logger().warning('cmd_vel 끊김 → 0 발행')
            self.was_moving = moving
            if self.core.gate_enable and not self.core.meas_ok(t) and not self.core.stale(t):
                self.get_logger().warning('조향 측정(joint_states) 없음 → 정렬 게이트 꺼진 상태로 통과',
                                          throttle_duration_sec=2.0)
            on = self.core.last_err > self.core.align_stop
            if on and not self.gate_on:
                self.get_logger().info(f'정렬 대기: 조향 오차 {math.degrees(self.core.last_err):.0f}° → 바퀴 거의 정지',
                                       throttle_duration_sec=0.5)
            self.gate_on = on
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = self.out_frame
            m.twist.linear.x, m.twist.linear.y, m.twist.angular.z = vx, vy, wz
            self.pub.publish(m)
            dm = Float64MultiArray()
            dm.data = [self.core.last_err, self.core.last_scale]
            self.pub_diag.publish(dm)

    rclpy.init(args=args)
    node = CmdFrameAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
