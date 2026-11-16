"""OpenCR 브리지 노드 통합 테스트 — 실물 보드 없이 pty 두 대로.

핸드셰이크(I) → role 배정 → C 프레임 en/값 → 고장·명령 끊김 차단까지 확인한다.
포트는 파라미터 `ports` 로 직접 지정하고, 보드는 pty 응답기로 흉내 낸다.
"""
import contextlib
import json
import os
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import rclpy  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Empty  # noqa: E402

from r12_swerve_control.opencr_bridge_node import OpenCRBridgeNode  # noqa: E402

ALL_JOINTS = ['front_left_steer_joint', 'front_right_steer_joint',
              'rear_left_steer_joint', 'rear_right_steer_joint',
              'front_left_wheel_joint', 'front_right_wheel_joint',
              'rear_left_wheel_joint', 'rear_right_wheel_joint']


class FakeBoard:
    """role 을 흉내 내는 OpenCR 한 대. I/? 에 JSON 한 줄로 답한다."""

    def __init__(self, role, order, answer=True):
        self.role = role
        self.order = order
        self.answer = answer
        self.master, self.slave = os.openpty()
        self.port = os.ttyname(self.slave)
        self.lines = []
        self.lock = threading.Lock()
        self.running = True
        self.thread = threading.Thread(target=self._serve, daemon=True)
        self.thread.start()

    def _reply(self, obj):
        os.write(self.master, (json.dumps(obj) + '\n').encode())

    def _serve(self):
        while self.running:
            try:
                data = os.read(self.master, 4096)
            except OSError:
                return
            if not data:
                return
            for raw in data.split(b'\n'):
                line = raw.strip()
                if not line:
                    continue
                with self.lock:
                    self.lines.append(line.decode())
                if not self.answer:
                    continue
                if line.startswith(b'I'):
                    self._reply({'hello': 1, 'proto': 1, 'board': self.role,
                                 'side': self.role.replace('swerve_', ''),
                                 'fw': 'opencr_' + self.role + ' 0.2.2',
                                 'order': self.order})
                elif line.startswith(b'?'):
                    self._reply({'cfg': 1, 'board': self.role,
                                 'order': self.order})

    def push(self, obj):
        self._reply(obj)

    def last_c(self):
        with self.lock:
            frames = [line for line in self.lines if line.startswith('C ')]
        return frames[-1].split()[1:] if frames else None

    def has_e(self):
        with self.lock:
            return any(line == 'E' for line in self.lines)

    def close(self):
        self.running = False
        for fd in (self.slave, self.master):
            try:
                os.close(fd)
            except OSError:
                pass


class BridgeNodeWithOverrides(OpenCRBridgeNode):
    """파라미터 오버라이드를 주입하는 얇은 래퍼."""

    OVERRIDES = {}

    def declare_parameter(self, name, value=None, descriptor=None, **kwargs):
        if name in self.OVERRIDES:
            return Parameter(name, value=self.OVERRIDES[name])
        return super().declare_parameter(name, value, descriptor, **kwargs)


def spin_for(node, seconds, pub=None, msg=None):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if pub is not None:
            pub.publish(msg)      # 실제 텔레옵처럼 계속 흘려보낸다
        rclpy.spin_once(node, timeout_sec=0.01)


def wait_boards(node, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.01)
        if len(node.boards) == len(node.roles):
            return True
    return False


def command_msg():
    msg = JointState()
    msg.name = list(ALL_JOINTS)
    msg.position = [0.1, 0.2, 0.3, 0.4, 0.0, 0.0, 0.0, 0.0]
    msg.velocity = [0.0, 0.0, 0.0, 0.0, 1.1, 2.2, 3.3, 4.4]
    return msg


@contextlib.contextmanager
def running_node(left, right, timeout_s=0.5):
    BridgeNodeWithOverrides.OVERRIDES = {
        'board_roles': ['swerve_left', 'swerve_right'],
        'ports': [left.port, right.port],
        'discovery_timeout_s': timeout_s,
        'bridge_hz': 50.0,
        'cmd_timeout_s': 0.25,
    }
    rclpy.init()
    node = BridgeNodeWithOverrides()
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_handshake_frames_and_safety_gating():
    left = FakeBoard('swerve_left', ['FL', 'RL'])
    right = FakeBoard('swerve_right', ['FR', 'RR'])
    try:
        with running_node(left, right) as node:
            assert wait_boards(node)
            assert {r: b.order for r, b in node.boards.items()} == {
                'swerve_left': ['FL', 'RL'],
                'swerve_right': ['FR', 'RR']}
            assert node._ready()

            pub = node.create_publisher(JointState, '/r12/swerve_joint_commands', 10)
            spin_for(node, 0.3)
            msg = command_msg()
            spin_for(node, 0.3, pub, msg)

            # 보드별 order 대로 값이 실린다. left=FL,RL / right=FR,RR
            assert left.last_c() == ['1', '1.10000', '3.30000', '0.10000', '0.30000']
            assert right.last_c() == ['1', '2.20000', '4.40000', '0.20000', '0.40000']

            # 한 보드라도 flt 를 보고하면 모든 보드가 en=0
            left.push({'flt': 1, 'wd': 0, 'mod': []})
            spin_for(node, 0.3, pub, msg)
            assert left.last_c()[0] == '0'
            assert right.last_c()[0] == '0'

            left.push({'flt': 0, 'wd': 0, 'mod': []})
            spin_for(node, 0.3, pub, msg)
            assert left.last_c()[0] == '1'

            # 명령이 timeout 넘게 끊기면 en=0, 조향각은 유지
            spin_for(node, 0.5)
            assert left.last_c()[0] == '0'
            assert left.last_c()[3:] == ['0.10000', '0.30000']
    finally:
        left.close()
        right.close()


def test_clear_faults_sends_e_to_every_board():
    """텔레옵 R 키 → /r12/swerve_clear_faults → 양 보드에 E."""
    left = FakeBoard('swerve_left', ['FL', 'RL'])
    right = FakeBoard('swerve_right', ['FR', 'RR'])
    try:
        with running_node(left, right) as node:
            assert wait_boards(node)
            pub = node.create_publisher(Empty, '/r12/swerve_clear_faults', 10)

            end = time.monotonic() + 1.5
            while time.monotonic() < end and not (left.has_e() and right.has_e()):
                spin_for(node, 0.05, pub, Empty())

            assert left.has_e(), '왼쪽 보드에 E 가 가지 않았다'
            assert right.has_e(), '오른쪽 보드에 E 가 가지 않았다'
    finally:
        left.close()
        right.close()


def test_missing_board_keeps_commands_disabled():
    left = FakeBoard('swerve_left', ['FL', 'RL'])
    silent = FakeBoard('swerve_right', ['FR', 'RR'], answer=False)
    try:
        with running_node(left, silent) as node:
            assert wait_boards(node, timeout=1.5) is False
            assert set(node.boards) == {'swerve_left'}
            assert not node._ready()
            pub = node.create_publisher(JointState, '/r12/swerve_joint_commands', 10)
            spin_for(node, 0.2)
            spin_for(node, 0.3, pub, command_msg())
            assert left.last_c()[0] == '0'
            assert left.last_c()[1:3] == ['0.00000', '0.00000']
    finally:
        left.close()
        silent.close()
