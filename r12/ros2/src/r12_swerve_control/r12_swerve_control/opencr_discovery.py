# =============================================================
# 패키지 : r12_swerve_control
# 파일   : opencr_discovery.py
# 역할   : USB 허브에 붙은 OpenCR 보드를 포트 번호가 아니라 핸드셰이크로 찾는다.
#          세 보드(스월브 L/R + arm)가 꽂힌 순서대로 /dev/ttyACM0..2 가 되므로
#          "왼쪽은 ACM0" 같은 가정을 버리고 보드가 스스로 말하는 role 을 쓴다.
#
# [프로토콜] 한 줄 = JSON 1개, USB CDC 115200.
#   호스트 → 보드 : I\n
#   보드 → 호스트 : {"hello":1,"proto":1,"board":"swerve_left","side":"left",
#                    "kind":"swerve","fw":"opencr_swerve_left 0.2.2","order":["FL","RL"]}
#   구 펌웨어(I 없음)는 ?\n 의 {"cfg":1,...,"side":"left"} 로 대신 식별한다.
#   role 은 board 값. 없으면 side(left/right) → swerve_left/right, 그다음 fw 이름 순.
#
# [설계] identify() 는 포트를 잠깐 열고 닫는 순수 함수다. ROS 를 import 하지 않아
#        pty 로 단위 테스트할 수 있다. 실제 포트 선택·중복 role 경고는 브리지가 한다.
# =============================================================
import glob
import json
import os
import time

import serial

PROTO_VERSION = 1
DEFAULT_PATTERNS = ('/dev/ttyACM*', '/dev/serial/by-id/*')
ROLE_ALIASES = {'left': 'swerve_left', 'right': 'swerve_right'}


def role_from_info(info):
    """핸드셰이크/cfg dict 에서 보드 role 을 고른다. 모르면 None."""
    role = info.get('board')
    if role:
        return role
    side = info.get('side')
    if side in ROLE_ALIASES:
        return ROLE_ALIASES[side]
    fw = info.get('fw', '')
    for name in ('swerve_left', 'swerve_right', 'arm'):
        if fw.startswith('opencr_' + name):
            return name
    return None


def candidate_ports(patterns):
    """glob 패턴들을 실제 tty 경로로. by-id 심볼릭 링크는 realpath 로 중복 제거."""
    seen = set()
    ports = []
    for pattern in patterns:
        for path in sorted(glob.glob(pattern)):
            real = os.path.realpath(path)
            if real in seen or not os.path.exists(real):
                continue
            seen.add(real)
            ports.append(real)
    return ports


def identify(port, baud=115200, timeout_s=0.6):
    """포트 하나에 I 를 되풀이해 보내 보드 정보를 받는다. 응답 없으면 None.

    포트를 열면 보드가 리셋되는 경우가 있어 I 를 한 번만 보내면 부팅 중에 놓친다.
    그래서 timeout_s 동안 I 를 0.2 s 간격으로 다시 보내고, 마지막 40%에는 ? 도 섞는다.
    새 펌웨어는 hello 로 즉답하고, 구 펌웨어는 ? 의 cfg(side/fw/order 포함)로 답한다.
    """
    try:
        # exclusive: 다른 브리지/터미널이 이미 연 포트는 건드리지 않는다.
        # (스월브 브리지와 팔 브리지가 같은 허브를 동시에 훑을 때 응답을 뺏지 않게)
        ser = serial.Serial(port, baud, timeout=0.02, write_timeout=0.2,
                            exclusive=True)
    except (serial.SerialException, OSError):
        return None
    try:
        ser.reset_input_buffer()
        start = time.monotonic()
        deadline = start + timeout_s
        next_send = start
        fallback_at = start + timeout_s * 0.6
        buf = b''
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_send:
                try:
                    ser.write(b'?\n' if now >= fallback_at else b'I\n')
                except (serial.SerialException, OSError):
                    return None
                next_send = now + 0.2
            try:
                chunk = ser.read(4096)
            except (serial.SerialException, OSError):
                return None
            if not chunk:
                time.sleep(0.005)
                continue
            buf += chunk
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                try:
                    obj = json.loads(line.strip())
                except ValueError:
                    continue
                if isinstance(obj, dict) and (obj.get('hello') or 'cfg' in obj):
                    return obj
        return None
    finally:
        try:
            ser.close()
        except (serial.SerialException, OSError):
            pass


def identify_ports(patterns, baud=115200, timeout_s=0.6, skip_ports=(),
                   identify_fn=identify):
    """아직 쓰지 않는 후보 포트들을 훑어 {port: info} 를 돌려준다."""
    skip = {os.path.realpath(p) for p in skip_ports}
    found = {}
    for port in candidate_ports(patterns):
        if port in skip:
            continue
        info = identify_fn(port, baud, timeout_s)
        if info is not None:
            found[port] = info
    return found


def assign_roles(infos, roles, assigned=None):
    """{port: info} 를 {role: (port, info)} 로. 이미 있는 role 은 duplicates 로.

    역할이 목록에 없는 보드(예: 스월브 브리지가 본 arm 보드)는 조용히 무시한다.
    반환: (boards, duplicates). duplicates 는 [(role, port)] — 같은 role 보드가 둘 이상.
    """
    boards = dict(assigned or {})
    duplicates = []
    for port, info in infos.items():
        role = role_from_info(info)
        if role not in roles:
            continue
        if role in boards:
            duplicates.append((role, port))
            continue
        boards[role] = (port, info)
    return boards, duplicates
