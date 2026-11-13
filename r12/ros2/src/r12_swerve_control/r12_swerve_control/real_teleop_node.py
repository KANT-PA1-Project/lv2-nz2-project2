# real_teleop_node.py
import math
import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# 키보드 조작 안내 메시지
INFO_MSG = """
--------------------------------------------------
R12 Swerve Keyboard Teleop
--------------------------------------------------
Moving:
   W : 전진 (Forward)
   S : 후진 (Backward)
   A : 좌측 횡이동 (Strafe Left)
   D : 우측 횡이동 (Strafe Right)

Turning (Yaw):
   Q : 제자리 반시계 회전 (Turn Left)
   E : 제자리 시계 회전 (Turn Right)

Speed Control:
   + / - : 최대 선속도 증가 / 감소
   [ / ] : 최대 회전속도 증가 / 감소
   Space : 정지 (Stop)

CTRL-C to quit.
--------------------------------------------------
"""

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('real_teleop')

        # ----- parameters -------------------------------------------------
        self.declare_parameter('rate', 50.0)          # publishing frequency [Hz]
        self.declare_parameter('max_linear_vel', 1.0) # m/s, max linear speed
        self.declare_parameter('max_angular_vel', 1.5)# rad/s, max turning speed
        self.declare_parameter('max_wheel_vel', 5.0)  # rad/s, max wheel speed limit

        # 로봇 기하학적 파라미터 (차량 앞뒤 축 거리, 좌우 축 거리 [m])
        self.declare_parameter('wheelbase', 0.3)      # L (Front to Rear distance)
        self.declare_parameter('track_width', 0.3)    # W (Left to Right distance)

        # ----- internal state ---------------------------------------------
        self.pub = self.create_publisher(
            JointState,
            '/r12/swerve_joint_commands',
            10
        )

        self.rate = self.get_parameter('rate').value
        self.timer = self.create_timer(1.0 / self.rate, self._publish_timer)

        # 현재 입력 상태 변수
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0

        # 터미널 설정 저장 (키보드 논블로킹 입력을 위해)
        self.settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info('Swerve Keyboard Teleop node started.')
        print(INFO_MSG)

    # -----------------------------------------------------------------
    def _get_key(self):
        """터미널에서 입력된 키를 논블로킹으로 읽어옴"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.001)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    # -----------------------------------------------------------------
    def _publish_timer(self):
        """키 입력을 처리하고 JointState 퍼블리시"""
        key = self._get_key()
        max_v = self.get_parameter('max_linear_vel').value
        max_w = self.get_parameter('max_angular_vel').value

        if key:
            if key == 'w':
                self.vx = max_v
                self.vy = 0.0
                self.omega = 0.0
            elif key == 's':
                self.vx = -max_v
                self.vy = 0.0
                self.omega = 0.0
            elif key == 'a':
                self.vx = 0.0
                self.vy = max_v
                self.omega = 0.0
            elif key == 'd':
                self.vx = 0.0
                self.vy = -max_v
                self.omega = 0.0
            elif key == 'q':
                self.vx = 0.0
                self.vy = 0.0
                self.omega = max_w
            elif key == 'e':
                self.vx = 0.0
                self.vy = 0.0
                self.omega = -max_w
            elif key == ' ':
                # 스페이스바: 정지
                self.vx = 0.0
                self.vy = 0.0
                self.omega = 0.0
            elif key == '\x03':
                # Ctrl+C
                sys.exit(0)

        # 스월브 기하학 계산 수행
        joint_state = self._make_swerve_message(self.vx, self.vy, self.omega)
        self.pub.publish(joint_state)

    # -----------------------------------------------------------------
    def _make_swerve_message(self, vx, vy, omega) -> JointState:
        """속도 벡터를 4개 바퀴의 조향각과 휠 속도로 변환"""
        max_wheel_v = self.get_parameter('max_wheel_vel').value
        L = self.get_parameter('wheelbase').value
        W = self.get_parameter('track_width').value
        A = W / 2.0
        B = L / 2.0

        # FL, FR, RL, RR 순서
        corners = [
            (+B, -A),  # Front Left  (x = +B, y = -A)
            (+B, +A),  # Front Right (x = +B, y = +A)
            (-B, -A),  # Rear Left   (x = -B, y = -A)
            (-B, +A)   # Rear Right  (x = -B, y = +A)
        ]

        steer_angles = []
        wheel_velocities = []

        for x, y in corners:
            v_x = vx - omega * y
            v_y = vy + omega * x

            speed = math.hypot(v_x, v_y)
            angle = math.atan2(v_y, v_x)

            # 정지 상태일 때 조향각이 꺾인 채로 유지되는 것 방지
            if speed < 0.001:
                angle = 0.0

            steer_angles.append(angle)
            wheel_velocities.append(min(speed, max_wheel_v))

        msg = JointState()
        msg.name = [
            'front_left_steer_joint', 'front_right_steer_joint', 
            'rear_left_steer_joint', 'rear_right_steer_joint',
            'front_left_wheel_joint', 'front_right_wheel_joint', 
            'rear_left_wheel_joint', 'rear_right_wheel_joint'
        ]
        msg.position = steer_angles
        msg.velocity = wheel_velocities

        return msg


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()