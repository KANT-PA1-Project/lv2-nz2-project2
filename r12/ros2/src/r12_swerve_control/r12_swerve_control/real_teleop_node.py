# =============================================================
# 패키지 : r12_swerve_control
# 파일   : real_teleop_node.py
# 역할   : 키보드 텔레옵. 차체 속도(Twist)만 발행한다. 역기구학은 하지 않는다.
#          IK·±90° fold·단위변환은 swerve_controller_node 담당(실물·시뮬 공통).
#
# [출력]  /cmd_vel (geometry_msgs/Twist): linear.x(전후), linear.y(좌+), angular.z(yaw, CCW+)
#         /r12/swerve_clear_faults (std_msgs/Empty): R 키 — 래치 고장 해제
# [파이프라인] teleop(/cmd_vel) → swerve_controller(IK) → /r12/swerve_joint_commands
#              → 실물: opencr_bridge / 시뮬: gz_command_relay → ros_gz_bridge → gz
# [UI]   키 입력마다 화면을 고정 패널로 다시 그린다(안내가 스크롤로 사라지지 않게).
# =============================================================
import sys
import select
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

CLEAR_HOME = '\033[2J\033[H'   # 화면 지우고 커서 좌상단
RULE = '━' * 40


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('real_teleop')
        self.declare_parameter('rate', 50.0)
        self.declare_parameter('max_linear_vel', 0.898)  # m/s, 하드웨어 최대(65rpm×4×0.033) = controller 와 동일
        self.declare_parameter('max_angular_vel', 5.1)   # rad/s, 하드웨어 최대(controller yaw 상한과 동일)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.clear_pub = self.create_publisher(Empty, '/r12/swerve_clear_faults', 10)
        self.last_clear_s = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        self.max_v = self.get_parameter('max_linear_vel').value
        self.max_w = self.get_parameter('max_angular_vel').value

        # cbreak 한 번만(매 틱 토글하면 틱 사이 눌린 키가 Enter 전까지 안 읽힘). 복원은 main.
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        rate = self.get_parameter('rate').value
        self.create_timer(1.0 / rate, self._tick)
        self._draw()

    def _get_key(self):
        rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
        return sys.stdin.read(1) if rlist else ''

    def _state_label(self):
        if self.vx > 0:    return '전진 ▲'
        if self.vx < 0:    return '후진 ▼'
        if self.vy > 0:    return '좌 횡이동 ◀'
        if self.vy < 0:    return '우 횡이동 ▶'
        if self.omega > 0: return '좌회전 ↺'
        if self.omega < 0: return '우회전 ↻'
        return '정지 ■'

    def _draw(self):
        print(CLEAR_HOME, end='')
        print(RULE)
        print('  R30 SWERVE TELEOP   (/cmd_vel → swerve_controller)')
        print(RULE)
        print('  이동    W / S  전·후        A / D  좌·우 횡이동')
        print('  회전    Q / E  좌·우         Space  정지')
        print('  속도    + / -  선속 상한     [ / ]  회전 상한')
        print('  기타    R  고장 해제         Ctrl-C  종료')
        print('─' * 40)
        print(f'  선속 상한 : {self.max_v:5.2f} m/s')
        print(f'  회전 상한 : {self.max_w:5.2f} rad/s')
        print(f'  명령      : vx={self.vx:+.2f}  vy={self.vy:+.2f}  w={self.omega:+.2f}')
        print(f'  상태      : {self._state_label()}')
        print(RULE, flush=True)

    def _tick(self):
        key = self._get_key()
        if key:
            k = key.lower()
            if k == 'w':
                self.vx, self.vy, self.omega = self.max_v, 0.0, 0.0
            elif k == 's':
                self.vx, self.vy, self.omega = -self.max_v, 0.0, 0.0
            elif k == 'a':
                self.vx, self.vy, self.omega = 0.0, self.max_v, 0.0
            elif k == 'd':
                self.vx, self.vy, self.omega = 0.0, -self.max_v, 0.0
            elif k == 'q':
                self.vx, self.vy, self.omega = 0.0, 0.0, self.max_w
            elif k == 'e':
                self.vx, self.vy, self.omega = 0.0, 0.0, -self.max_w
            elif key in ('+', '='):
                self.max_v = min(round(self.max_v + 0.05, 2), 2.0)
            elif key == '-':
                self.max_v = max(round(self.max_v - 0.05, 2), 0.0)
            elif key == ']':
                self.max_w = min(round(self.max_w + 0.05, 2), 3.0)
            elif key == '[':
                self.max_w = max(round(self.max_w - 0.05, 2), 0.0)
            elif key == ' ':
                self.vx, self.vy, self.omega = 0.0, 0.0, 0.0
            elif k == 'r':
                self.vx, self.vy, self.omega = 0.0, 0.0, 0.0
                now = time.monotonic()
                if now - self.last_clear_s >= 1.0:
                    self.last_clear_s = now
                    self.clear_pub.publish(Empty())
            elif key == '\x03':
                raise KeyboardInterrupt
            self._draw()

        msg = Twist()
        msg.linear.x = self.vx
        msg.linear.y = self.vy
        msg.angular.z = self.omega
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 터미널 모드부터 되돌리고, 정리 중 신호가 또 와도 조용히 끝낸다.
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        print()
        # KeyboardInterrupt 는 Exception 이 아니라 BaseException — 정리 중 또 와도 잡아 조용히 끝낸다.
        try:
            node.destroy_node()
        except BaseException:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except BaseException:
            pass


if __name__ == '__main__':
    main()
