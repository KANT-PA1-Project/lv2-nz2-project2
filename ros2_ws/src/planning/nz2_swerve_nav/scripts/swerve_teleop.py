#!/usr/bin/env python3
# nz2_swerve_nav/scripts/swerve_teleop.py — 스워브 전용 키보드 조종 (TwistStamped, 20 Hz 연속 발행)
# 개발 PC: ros2 run nz2_swerve_nav swerve_teleop.py   (시뮬이면 --ros-args -p use_sim_time:=true)
#
# 키 (base_footprint 기준, 정면 = 팔 쪽)
#   w/s : 앞/뒤        a/d : 왼쪽/오른쪽 게걸음
#   q/e : 대각 앞왼/앞오 (통로에서는 쓰지 말 것: 폭 0.48 m)
#   j/l : 제자리 왼/오 회전 (넓은 곳에서만: 회전 지름 0.50 m)
#   + / - : 속도 단계 올림/내림      space 또는 k : 즉시 정지      Ctrl-C : 정지 후 종료
# 안전: 마지막 키 입력 후 hold_time(0.8 s) 지나면 자동 정지 → 키를 누르고 있어야 계속 움직임
# 속도 상한 max_speed 0.90 m/s = 실물 최대(65 rpm × 4, 바퀴 66 mm → 0.898 m/s). 실제 속도는 뒤단 제한이 먼저 걸림:
#   r12 linear_speed_m_s(시뮬 덮어쓰기 0.2) → Gazebo URDF 바퀴 한계 10 rad/s(0.33 m/s)
# Ctrl-C: rclpy 기본 SIGINT 처리기를 끄고 직접 처리 → 정지 명령 3회 발행 후 터미널 설정 복구
import select, sys, termios, tty, time
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

KEYS = {'w': (1, 0, 0), 's': (-1, 0, 0), 'a': (0, 1, 0), 'd': (0, -1, 0),
        'q': (0.7071, 0.7071, 0), 'e': (0.7071, -0.7071, 0), 'j': (0, 0, 1), 'l': (0, 0, -1)}

class Teleop(Node):
    def __init__(self):
        super().__init__('swerve_teleop')
        self.lin = self.declare_parameter('speed', 0.15).value      # [m/s]
        self.ang = self.declare_parameter('turn', 0.6).value        # [rad/s]
        self.max_lin = self.declare_parameter('max_speed', 0.90).value
        self.hold = self.declare_parameter('hold_time', 0.8).value  # [s]
        self.frame = self.declare_parameter('frame_id', 'base_footprint').value
        self.pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.dir = (0, 0, 0); self.last_key = 0.0

    def send(self):
        m = TwistStamped(); m.header.stamp = self.get_clock().now().to_msg(); m.header.frame_id = self.frame
        if time.monotonic() - self.last_key > self.hold: self.dir = (0, 0, 0)
        m.twist.linear.x = self.dir[0] * self.lin; m.twist.linear.y = self.dir[1] * self.lin
        m.twist.angular.z = self.dir[2] * self.ang
        self.pub.publish(m)

def main():
    # rclpy 기본 SIGINT 처리기는 Ctrl-C 때 컨텍스트를 먼저 닫음 → finally 의 publish 가 예외 → 터미널 복구가 안 됨
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO); n = Teleop()
    old = termios.tcgetattr(sys.stdin)
    print('w/s 앞뒤 | a/d 좌우 게걸음 | j/l 제자리 회전 | q/e 대각 | +/- 속도 | space 정지 | 키를 누르고 있어야 계속 이동')
    try:
        tty.setcbreak(sys.stdin.fileno())
        next_t = time.monotonic()
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.01)[0]:
                c = sys.stdin.read(1).lower()
                if c in KEYS: n.dir = KEYS[c]; n.last_key = time.monotonic()
                elif c in (' ', 'k'): n.dir = (0, 0, 0)
                elif c in ('+', '='): n.lin = min(n.max_lin, n.lin + 0.05); print(f'속도 {n.lin:.2f} m/s')
                elif c in ('-', '_'): n.lin = max(0.05, n.lin - 0.05); print(f'속도 {n.lin:.2f} m/s')
            if time.monotonic() >= next_t:
                n.send(); next_t += 0.05
            rclpy.spin_once(n, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)   # 터미널 복구를 가장 먼저
        n.dir = (0, 0, 0); n.last_key = 0.0
        try:
            for _ in range(3): n.send(); time.sleep(0.02)
        except Exception as e:  # 컨텍스트가 이미 닫혔어도 종료는 진행 (어댑터가 0.25 s 뒤 0 발행)
            print(f'정지 명령 발행 실패: {e}')
        n.destroy_node(); rclpy.try_shutdown()

if __name__ == '__main__':
    main()
