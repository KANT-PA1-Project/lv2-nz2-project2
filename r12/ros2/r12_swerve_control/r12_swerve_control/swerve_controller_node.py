# =============================================================
# 패키지 : r12_swerve_control
# 파일   : swerve_controller_node.py
# 역할   : 실물 R21 스월브 결정론적 컨트롤러.
#          /r12/cmd_vel(TwistStamped, base_link) → ±90° 제약 IK → /r12/swerve_joint_commands(JointState).
#
# [입력]  /r12/cmd_vel (geometry_msgs/TwistStamped): twist.linear.x,y, twist.angular.z. base_link 만.
#         stale 스탬프(>timeout_s)·비정상 frame·NaN 은 정지로 처리(r12 계약).
# [출력]  /r12/swerve_joint_commands (sensor_msgs/JointState) 2통, 학습 노드와 같은 규약:
#            ① name=STEER, position=조향각 rad   ② name=WHEEL, velocity=휠 각속도 rad/s
#         조인트 이름 = lh1/rh1/lh2/rh2 ×_steer_joint/_wheel_joint = FL FR RL RR.
# [설계]  r12_learned_swerve_node 는 시뮬/RL 정책용. 이 노드는 같은 토픽·규약을 쓰되
#         학습 없이 결정론적 해석 IK(swerve_kinematics)로 명령을 만든다. 하드웨어 브리지가
#         JointState 를 받아 OpenCR 로 내려보낸다. 한계·기하는 r12_navigation_contract.json.
# =============================================================
import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState

from r12_swerve_control.swerve_kinematics import inverse_kinematics, optimize_angle

PREFIX = ['lh1', 'rh1', 'lh2', 'rh2']            # FL FR RL RR
STEER = [p + '_steer_joint' for p in PREFIX]
WHEEL = [p + '_wheel_joint' for p in PREFIX]
HOLD_SPEED = 1e-4                                 # 이 미만이면 조향 유지, 바퀴만 정지


class SwerveControllerNode(Node):
    def __init__(self):
        super().__init__('r12_swerve_controller')
        d = self.declare_parameter
        # 기하 R21: 모듈 (x 전진, y 좌) FL FR RL RR, ±0.124 m
        hx = d('half_wheelbase_m', 0.124).value
        hy = d('half_track_m', 0.124).value
        self.positions = [(hx, hy), (hx, -hy), (-hx, hy), (-hx, -hy)]
        self.wheel_radius = d('wheel_radius_m', 0.033).value
        # 한계: r12_navigation_contract.json limits
        self.max_linear = d('linear_speed_m_s', 0.2).value
        self.max_angular = d('yaw_rate_rad_s', 0.4).value
        self.steer_limit = d('steering_angle_rad', 1.5707963267948966).value
        self.steer_rate = d('steering_rate_rad_s', 1.0).value
        self.wheel_limit = d('wheel_speed_rad_s', 10.0).value
        self.max_accel = d('command_acceleration_m_s2', 0.2).value
        self.timeout_s = d('timeout_s', 0.25).value
        self.hz = d('control_hz', 50.0).value

        self.angles = [0.0] * 4                   # 직전 조향각(optimize_angle nearest + slew 기준)
        self.cvx = 0.0                            # 램프된 본체 속도
        self.cvy = 0.0
        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_time = None
        self.pub = self.create_publisher(JointState, '/r12/swerve_joint_commands', 10)
        self.create_subscription(TwistStamped, '/r12/cmd_vel', self._on_cmd, 10)
        self.create_timer(1.0 / self.hz, self._tick)
        self.get_logger().info('r12_swerve_controller: 결정론적 ±90° 스월브 (R21)')

    def _on_cmd(self, msg):
        v = (msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z)
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # base_link 만, 유한값만, 스탬프 신선할 때만 (r12 계약: stale_stamp_rejected)
        if msg.header.frame_id not in ('base_link', '') or not all(map(math.isfinite, v)) \
                or abs(now - stamp) > self.timeout_s:
            self.cmd_time = None
            return
        self.cmd = v
        self.cmd_time = now

    def _stale(self):
        if self.cmd_time is None:
            return True
        return self.get_clock().now().nanoseconds * 1e-9 - self.cmd_time > self.timeout_s

    def _tick(self):
        dt = 1.0 / self.hz
        vx, vy, wz = (0.0, 0.0, 0.0) if self._stale() else self.cmd

        # 본체 속도 캡(IK 전)
        lin = math.hypot(vx, vy)
        if self.max_linear > 0.0 and lin > self.max_linear:
            s = self.max_linear / lin
            vx, vy = vx * s, vy * s
        if self.max_angular > 0.0:
            wz = max(-self.max_angular, min(self.max_angular, wz))

        # 선속도 가속 램프(급가속·급반전 전류 스파이크 억제)
        if self.max_accel > 0.0:
            dv = self.max_accel * dt
            self.cvx += max(-dv, min(dv, vx - self.cvx))
            self.cvy += max(-dv, min(dv, vy - self.cvy))
        else:
            self.cvx, self.cvy = vx, vy

        cap = self.max_linear if self.max_linear > 0.0 else 0.0
        raw = inverse_kinematics(self.cvx, self.cvy, wz, self.positions, cap)

        steers, wheels = [], []
        max_step = self.steer_rate * dt           # 조향 슬루 제한(계약 steering_rate_rad_s)
        for i, (angle, speed) in enumerate(raw):
            if speed < HOLD_SPEED:                 # 저속: 조향 유지, 바퀴 정지
                steer, drive = self.angles[i], 0.0
            else:
                target, sign = optimize_angle(angle, self.angles[i], self.steer_limit)
                # 한 틱에 steer_rate 이상 못 돈다
                if max_step > 0.0:
                    target = self.angles[i] + max(-max_step, min(max_step, target - self.angles[i]))
                steer = target
                # m/s → 휠 각속도 rad/s, 휠 한계로 클립
                w = sign * speed / self.wheel_radius
                drive = max(-self.wheel_limit, min(self.wheel_limit, w))
            self.angles[i] = steer
            steers.append(float(steer))
            wheels.append(float(drive))

        stamp = self.get_clock().now().to_msg()
        sm = JointState()
        sm.header.stamp = stamp
        sm.name = STEER
        sm.position = steers
        self.pub.publish(sm)
        wm = JointState()
        wm.header.stamp = stamp
        wm.name = WHEEL
        wm.velocity = wheels
        self.pub.publish(wm)


def main(args=None):
    rclpy.init(args=args)
    node = SwerveControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
