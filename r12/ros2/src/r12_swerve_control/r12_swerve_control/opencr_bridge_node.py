# =============================================================
# 패키지 : r12_swerve_control
# 파일   : opencr_bridge_node.py
# 역할   : r12 계약이 비워 둔 하드웨어 전송(No hardware transport)을 채운다.
#          /r12/swerve_joint_commands(JointState) ↔ 좌·우 OpenCR 2보드 시리얼.
#
# [보드 찾기] USB 허브에 스월브 L/R + arm 세 보드가 붙으면 /dev/ttyACM 번호는 꽂힌 순서를
#          따른다. 그래서 포트를 고정하지 않고, 후보 포트마다 핸드셰이크(I)를 보내
#          각 보드가 말하는 role(swerve_left/swerve_right/arm)로 포트를 배정한다.
#          보드가 빠지거나 다시 꽂히면 discovery_interval_s 마다 재탐색한다.
#          자세한 프로토콜·순수 함수는 opencr_discovery.py.
#
# [입력]  /r12/swerve_joint_commands (sensor_msgs/JointState): 조향 position(rad)·휠 velocity(rad/s).
#         이름 {front_left,front_right,rear_left,rear_right} ×_steer/_wheel_joint = FL FR RL RR.
# [출력]  각 보드로 'C <en> <w...> <s...>' (휠 rad/s, 조향 rad, 보드 cfg order 순) 50 Hz.
#         보드 상태 JSON({"mod":[{c,sp,wv}]}) → /r12/joint_states (sensor_msgs/JointState) 재발행.
# [규약]  보드 모듈코드→조인트 prefix : FL→front_left FR→front_right RL→rear_left
#         RR→rear_right. order·side 는 부팅 '?' cfg 에서 읽는다.
#         명령이 timeout_s 넘게 끊기면 en=0·휠 0 (안전 정지). 조향각은 마지막 값을 유지한다.
#         설정한 role 이 모두 없거나, 보드가 고장(flt)·워치독(wd)을 보고하면 en=0 으로
#         이 브리지에 붙은 모든 보드를 함께 세운다(한쪽만 서면 차체가 도는 것을 막는다).
# [의존]  pyserial (package.xml: python3-serial). 펌웨어 프로토콜은 r12/firmware/opencr_swerve_*.
# =============================================================
import json
import time

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import JointState

from r12_swerve_control.opencr_discovery import (DEFAULT_PATTERNS, assign_roles,
                                                 identify_ports)

CODE_TO_PREFIX = {'FL': 'front_left', 'FR': 'front_right',
                  'RL': 'rear_left', 'RR': 'rear_right'}   # r21_description 조인트명
PREFIX = ['front_left', 'front_right', 'rear_left', 'rear_right']
STEER = {p + '_steer_joint': p for p in PREFIX}   # 조인트명 → prefix
WHEEL = {p + '_wheel_joint': p for p in PREFIX}


class Board:
    def __init__(self, role, port, baud, logger, info):
        self.role = role                                  # 핸드셰이크가 알려준 role
        self.port = port
        self.log = logger
        self.ser = serial.Serial(port, baud, timeout=0)   # 논블로킹 읽기
        self.buf = b''
        self.order = info.get('order')                    # cfg 가 오기 전엔 명령 안 보냄
        self.side = info.get('side')
        self.fw = info.get('fw')
        self.ok = True
        self.have_status = False
        self.faults = 0
        self.wd = False
        if not self.order:                                # hello 에 order 가 없던 펌웨어
            self.ser.write(b'?\n')
        self.log.info(f'{role}: {port} ({self.fw or "?"}) order={self.order}')

    def read_lines(self):
        """도착한 완결 줄들을 파싱한 dict 로. 읽기 실패하면 ok=False."""
        if not self.ok:
            return
        try:
            self.buf += self.ser.read(4096)
        except (serial.SerialException, OSError) as e:
            self.ok = False
            self.log.warning(f'{self.port} read 실패: {e}')
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
            if 'cfg' in obj:
                self.order = obj.get('order', self.order)
                self.side = obj.get('side', self.side)
            if 'flt' in obj or 'mod' in obj:              # 50 Hz 상태 레코드
                self.have_status = True
                self.faults = int(obj.get('flt', 0))
                self.wd = bool(obj.get('wd', 0))
            yield obj

    def send_c(self, en, wheel_by_prefix, steer_by_prefix):
        if not self.ok or not self.order:
            return
        w = [wheel_by_prefix.get(CODE_TO_PREFIX[c], 0.0) for c in self.order]
        s = [steer_by_prefix.get(CODE_TO_PREFIX[c], 0.0) for c in self.order]
        frame = 'C %d %s %s\n' % (en, ' '.join('%.5f' % x for x in w),
                                  ' '.join('%.5f' % x for x in s))
        try:
            self.ser.write(frame.encode())
        except (serial.SerialException, OSError) as e:
            self.ok = False
            self.log.warning(f'{self.port} write 실패: {e}')

    def close(self):
        try:
            self.ser.close()
        except (serial.SerialException, OSError):
            pass


class OpenCRBridgeNode(Node):
    def __init__(self):
        super().__init__('r12_opencr_bridge')
        d = self.declare_parameter
        self.baud = d('host_baud', 115200).value
        self.hz = d('bridge_hz', 50.0).value
        self.timeout_s = d('cmd_timeout_s', 0.25).value
        # 포트 번호 대신 role 로 찾는다. arm 보드는 자기 브리지가 찾도록 여기 넣지 않는다.
        self.roles = list(d('board_roles', ['swerve_left', 'swerve_right']).value)
        self.port_globs = list(d('port_globs', list(DEFAULT_PATTERNS)).value)
        self.explicit_ports = list(d('ports', []).value)   # 지정하면 탐색 대신 이것만
        self.scan_interval = d('discovery_interval_s', 2.0).value
        self.discovery_timeout = d('discovery_timeout_s', 0.6).value
        self.require_all = d('require_all_boards', True).value
        self.stop_on_fault = d('stop_on_fault', True).value

        self.boards = {}                # role → Board
        self.steer = {}                 # prefix → 최근 조향각(rad), 유지
        self.wheel = {}                 # prefix → 최근 휠 각속도(rad/s)
        self.cmd_time = None
        self.fb_steer = {}              # prefix → 피드백 조향각
        self.fb_wheel = {}              # prefix → 피드백 휠속도
        self.last_scan = -1e9

        self.create_subscription(JointState, '/r12/swerve_joint_commands', self._on_cmd, 10)
        self.fb_pub = self.create_publisher(JointState, '/r12/joint_states', 10)
        self._scan()
        self.create_timer(1.0 / self.hz, self._tick)
        self.get_logger().info(
            f'r12_opencr_bridge: roles={self.roles} '
            f'ports={self.explicit_ports or self.port_globs}')

    # ---------------- 보드 탐색 / 재연결 ----------------
    def _patterns(self):
        return self.explicit_ports or self.port_globs

    def _scan(self):
        self.last_scan = time.monotonic()
        if not any(r not in self.boards for r in self.roles):
            return
        infos = identify_ports(self._patterns(), self.baud, self.discovery_timeout,
                               skip_ports=[b.port for b in self.boards.values()])
        existing = {role: (b.port, None) for role, b in self.boards.items()}
        assigned, dups = assign_roles(infos, self.roles, existing)
        for role, port in dups:
            self.get_logger().error(
                f'중복 role {role!r} ({port}): 두 보드가 같은 role 을 보고한다. '
                f'각 OpenCR 의 BOARD_ROLE 과 업로드한 스케치를 확인할 것.')
        for role, (port, info) in assigned.items():
            if role in self.boards:
                continue
            try:
                self.boards[role] = Board(role, port, self.baud, self.get_logger(), info)
            except (serial.SerialException, OSError) as e:
                self.get_logger().warning(f'{role}: {port} 열기 실패: {e}')
        missing = [r for r in self.roles if r not in self.boards]
        if missing:
            self.get_logger().warning(
                f'OpenCR role 대기 중 {missing} (현재 {list(self.boards)}) 패턴={self._patterns()}')

    def _drop(self, role):
        b = self.boards.pop(role, None)
        if b is None:
            return
        self.get_logger().warning(f'{role} ({b.port}) 연결 끊김 — 재탐색한다')
        # 이 보드가 맡던 모듈의 마지막 명령을 버려, 재연결 때 옛 조향각으로 튀지 않게 한다.
        for code in (b.order or []):
            prefix = CODE_TO_PREFIX.get(code)
            if prefix:
                self.steer.pop(prefix, None)
                self.wheel.pop(prefix, None)
        b.close()

    # ---------------- 수신 ----------------
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

    def _ready(self):
        """명령을 내려도 되는 상태인가. role/모듈이 겹치면 거부한다."""
        if self.require_all and any(r not in self.boards for r in self.roles):
            return False
        if not self.boards:
            return False
        codes = []
        for b in self.boards.values():
            if not b.order:
                return False
            codes.extend(b.order)
        if len(codes) != len(set(codes)):                 # 같은 모듈을 두 보드가 맡음
            return False
        if len(self.boards) == len(self.roles) and len(self.roles) > 1:
            if set(codes) != set(CODE_TO_PREFIX):         # FL FR RL RR 이 모두 있는가
                return False
        return True

    def _unsafe(self):
        if not self.stop_on_fault:
            return False
        return any(b.have_status and (b.faults or b.wd) for b in self.boards.values())

    def _tick(self):
        now = time.monotonic()
        for role, b in list(self.boards.items()):
            for obj in b.read_lines():
                for mod in obj.get('mod', []):
                    prefix = CODE_TO_PREFIX.get(mod.get('c'))
                    if prefix is None:
                        continue
                    if 'sp' in mod:
                        self.fb_steer[prefix] = mod['sp']
                    if 'wv' in mod:
                        self.fb_wheel[prefix] = mod['wv']
            if not b.ok:
                self._drop(role)
        if len(self.boards) < len(self.roles) and now - self.last_scan >= self.scan_interval:
            self._scan()

        stale = self._stale() or not self._ready() or self._unsafe()
        en = 0 if stale else 1
        wheel = {p: 0.0 for p in PREFIX} if stale else self.wheel
        for b in self.boards.values():
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
        for b in node.boards.values():              # 종료 시 안전 정지 1프레임
            b.send_c(0, {p: 0.0 for p in PREFIX}, node.steer)
            b.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
