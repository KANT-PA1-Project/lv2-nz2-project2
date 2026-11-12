# =============================================================
# 패키지 : r12_swerve_control
# 파일   : opencr_bridge_node.py
# 역할   : r12 계약이 비워 둔 하드웨어 전송(No hardware transport)을 채운다.
#          /r12/swerve_joint_commands(JointState) ↔ 좌·우 OpenCR 2보드 시리얼.
#
# [입력]  /r12/swerve_joint_commands (sensor_msgs/JointState): 조향 position(rad)·휠 velocity(rad/s).
#         이름 lh1/rh1/lh2/rh2 ×_steer/_wheel_joint = FL FR RL RR.
# [출력]  각 보드로 'C <en> <w...> <s...>' (휠 rad/s, 조향 rad, 보드 cfg order 순) 50 Hz.
#         보드 상태 JSON({"mod":[{c,sp,wv}]}) → /r12/joint_states (sensor_msgs/JointState) 재발행.
# [규약]  보드 모듈코드→조인트 prefix : FL→lh1 FR→rh1 RL→lh2 RR→rh2. order·side 는 부팅 '?' cfg 에서 읽는다.
#         명령이 timeout_s 넘게 끊기면 en=0·휠 0 (안전 정지). 조향각은 마지막 값을 유지한다.
# [의존]  pyserial (package.xml: python3-serial). 펌웨어 프로토콜은 r12/firmware/opencr_swerve_*.
# =============================================================
import json

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import JointState

CODE_TO_PREFIX = {'FL': 'lh1', 'FR': 'rh1', 'RL': 'lh2', 'RR': 'rh2'}
PREFIX = ['lh1', 'rh1', 'lh2', 'rh2']
STEER = {p + '_steer_joint': p for p in PREFIX}   # 조인트명 → prefix
WHEEL = {p + '_wheel_joint': p for p in PREFIX}


class Board:
    def __init__(self, port, baud, logger):
        self.port = port
        self.log = logger
        self.ser = serial.Serial(port, baud, timeout=0)   # 논블로킹 읽기
        self.buf = b''
        self.order = None                                 # cfg 가 오기 전엔 명령 안 보냄
        self.side = None
        self.ser.write(b'?\n')                            # cfg 요청

    def read_lines(self):
        """도착한 완결 줄들을 파싱한 dict 로."""
        try:
            self.buf += self.ser.read(4096)
        except serial.SerialException as e:
            self.log.warn(f'{self.port} read 실패: {e}')
            return
        while b'\n' in self.buf:
            line, self.buf = self.buf.split(b'\n', 1)
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except ValueError:
                continue                                  # 부분·비JSON 줄 무시
            if 'cfg' in obj and 'order' in obj:
                self.order = obj['order']
                self.side = obj.get('side')
                self.log.info(f'{self.port}: {self.side} order={self.order}')
            yield obj

    def send_c(self, en, wheel_by_prefix, steer_by_prefix):
        if not self.order:
            return
        w = [wheel_by_prefix.get(CODE_TO_PREFIX[c], 0.0) for c in self.order]
        s = [steer_by_prefix.get(CODE_TO_PREFIX[c], 0.0) for c in self.order]
        frame = 'C %d %s %s\n' % (en, ' '.join('%.5f' % x for x in w),
                                  ' '.join('%.5f' % x for x in s))
        try:
            self.ser.write(frame.encode())
        except serial.SerialException as e:
            self.log.warn(f'{self.port} write 실패: {e}')


class OpenCRBridgeNode(Node):
    def __init__(self):
        super().__init__('r12_opencr_bridge')
        d = self.declare_parameter
        baud = d('host_baud', 115200).value
        self.hz = d('bridge_hz', 50.0).value
        self.timeout_s = d('cmd_timeout_s', 0.25).value
        left = d('left_port', '/dev/ttyACM0').value
        right = d('right_port', '/dev/ttyACM1').value

        self.boards = [Board(left, baud, self.get_logger()),
                       Board(right, baud, self.get_logger())]
        self.steer = {}                 # prefix → 최근 조향각(rad), 유지
        self.wheel = {}                 # prefix → 최근 휠 각속도(rad/s)
        self.cmd_time = None
        self.fb_steer = {}              # prefix → 피드백 조향각
        self.fb_wheel = {}             # prefix → 피드백 휠속도

        self.create_subscription(JointState, '/r12/swerve_joint_commands', self._on_cmd, 10)
        self.fb_pub = self.create_publisher(JointState, '/r12/joint_states', 10)
        self.create_timer(1.0 / self.hz, self._tick)
        self.get_logger().info(f'r12_opencr_bridge: {left} + {right} @ {baud}')

    def _on_cmd(self, msg):
        # 조향은 position, 휠은 velocity 배열에서 (학습 노드 규약: 두 필드가 분리 발행될 수 있음)
        for i, name in enumerate(msg.name):
            if name in STEER and i < len(msg.position):
                self.steer[STEER[name]] = msg.position[i]
            if name in WHEEL and i < len(msg.velocity):
                self.wheel[WHEEL[name]] = msg.velocity[i]
        self.cmd_time = self.get_clock().now().nanoseconds * 1e-9

    def _stale(self):
        if self.cmd_time is None:
            return True
        return self.get_clock().now().nanoseconds * 1e-9 - self.cmd_time > self.timeout_s

    def _tick(self):
        stale = self._stale()
        en = 0 if stale else 1
        wheel = {p: 0.0 for p in PREFIX} if stale else self.wheel
        for b in self.boards:
            for obj in b.read_lines():
                for mod in obj.get('mod', []):
                    pre = CODE_TO_PREFIX.get(mod.get('c'))
                    if pre is None:
                        continue
                    if 'sp' in mod:
                        self.fb_steer[pre] = mod['sp']
                    if 'wv' in mod:
                        self.fb_wheel[pre] = mod['wv']
            b.send_c(en, wheel, self.steer)
        self._publish_feedback()

    def _publish_feedback(self):
        if not self.fb_steer and not self.fb_wheel:
            return
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        names, pos, vel = [], [], []
        for p in PREFIX:                            # 조향 조인트: position 만
            names.append(p + '_steer_joint')
            pos.append(float(self.fb_steer.get(p, 0.0)))
            vel.append(0.0)
        for p in PREFIX:                            # 휠 조인트: velocity 만
            names.append(p + '_wheel_joint')
            pos.append(0.0)
            vel.append(float(self.fb_wheel.get(p, 0.0)))
        m.name, m.position, m.velocity = names, pos, vel
        self.fb_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = OpenCRBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for b in node.boards:                       # 종료 시 안전 정지 1프레임
            b.send_c(0, {p: 0.0 for p in PREFIX}, node.steer)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
