#!/usr/bin/env python3
# nz2_swerve_nav/scripts/r12_cmd_to_gz.py
# 실행 위치: 개발 PC (Gazebo 시뮬 전용. 실물은 팀 r12_opencr_bridge)
#
# 역할: 팀 r12_swerve_controller 출력 /r12/swerve_joint_commands (JointState 2통: 조향 position / 바퀴 velocity)
#       → 기존 gz 브리지 토픽 /swerve/sim/<joint>/cmd_pos (조향 rad), /swerve/sim/<joint>/cmd_vel (바퀴 rad/s)
#   - 조인트 이름은 r12 와 assembly_nz2.urdf 가 같음 (front_left_steer_joint …), 부호 규약도 같음
#     (조향 관절각 = base_link 기준 굴림 방향, 바퀴 + = 굴림 방향 전진)
#   - 명령이 cmd_timeout 넘게 끊기면 바퀴 0 (조향은 마지막 값 유지)
#   - /estop true 동안 바퀴 0 (시뮬 보조. r12 램프를 기다리지 않고 즉시 정지)
#   - 바퀴 속도 |w| ≤ max_wheel_rad_s (기본 32 rad/s = URDF 바퀴 한계, 1.06 m/s), NaN 무시
import math

PREFIX = ['front_left', 'front_right', 'rear_left', 'rear_right']
STEER = [p + '_steer_joint' for p in PREFIX]
WHEEL = [p + '_wheel_joint' for p in PREFIX]

# ---------- 순수 로직 ----------


class RelayCore:
    def __init__(self, cmd_timeout=0.25, max_wheel=32.0):
        self.cmd_timeout = cmd_timeout
        self.max_wheel = max_wheel
        self.steer = {}
        self.wheel = {}
        self.last_time = None
        self.estop = False

    def on_cmd(self, names, positions, velocities, t):
        got = False
        for i, n in enumerate(names):
            if n in STEER and i < len(positions) and math.isfinite(positions[i]):
                self.steer[n] = float(positions[i])
                got = True
            if n in WHEEL and i < len(velocities) and math.isfinite(velocities[i]):
                self.wheel[n] = max(-self.max_wheel, min(self.max_wheel, float(velocities[i])))
                got = True
        if got:
            self.last_time = t

    def stale(self, t):
        return self.last_time is None or (t - self.last_time) > self.cmd_timeout

    def output(self, t):
        """반환: (조향 {이름: rad}, 바퀴 {이름: rad/s}). 조향은 받은 적 있는 것만."""
        stop = self.estop or self.stale(t)
        wheels = {n: (0.0 if stop else self.wheel.get(n, 0.0)) for n in WHEEL}
        return dict(self.steer), wheels


# ---------- ROS 노드 ----------


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, Float64

    class R12CmdToGz(Node):
        def __init__(self):
            super().__init__('r12_cmd_to_gz')
            d = self.declare_parameter
            self.core = RelayCore(cmd_timeout=d('cmd_timeout', 0.25).value,
                                  max_wheel=d('max_wheel_rad_s', 32.0).value)
            in_topic = d('input_topic', '/r12/swerve_joint_commands').value
            prefix = d('sim_topic_prefix', '/swerve/sim').value
            rate = d('rate', 50.0).value
            self.pub_s = {n: self.create_publisher(Float64, f'{prefix}/{n}/cmd_pos', 10) for n in STEER}
            self.pub_w = {n: self.create_publisher(Float64, f'{prefix}/{n}/cmd_vel', 10) for n in WHEEL}
            self.create_subscription(JointState, in_topic, self._on_cmd, 10)
            self.create_subscription(Bool, 'estop', self._on_estop,
                                     QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
            self.create_timer(1.0 / rate, self._tick)
            self.warned = False
            self.get_logger().info(f'r12_cmd_to_gz: {in_topic} → {prefix}/<joint>/cmd_pos|cmd_vel, {rate:.0f} Hz')

        def _now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def _on_cmd(self, m):
            self.core.on_cmd(m.name, m.position, m.velocity, self._now())

        def _on_estop(self, m):
            if m.data and not self.core.estop:
                self.get_logger().error('E-STOP: 시뮬 바퀴 0')
            self.core.estop = m.data

        def _tick(self):
            t = self._now()
            if self.core.stale(t) and self.core.last_time is not None and not self.warned:
                self.get_logger().warning('r12 명령 끊김 → 바퀴 0')
                self.warned = True
            elif not self.core.stale(t):
                self.warned = False
            steer, wheel = self.core.output(t)
            for n, v in steer.items():
                self.pub_s[n].publish(Float64(data=v))
            for n, v in wheel.items():
                self.pub_w[n].publish(Float64(data=v))

    rclpy.init(args=args)
    node = R12CmdToGz()
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
