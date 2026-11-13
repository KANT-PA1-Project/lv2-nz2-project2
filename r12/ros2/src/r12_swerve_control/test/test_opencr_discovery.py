"""opencr_discovery 순수/pty 테스트 — USB 허브에서 보드를 role 로 찾는지.

실물 보드 없이 pty 쌍으로 'I' 핸드셰이크와 구 펌웨어 '?' 폴백을 흉내 낸다.
"""
import contextlib
import json
import os
import sys
import threading

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'r12_swerve_control'))
from opencr_discovery import (assign_roles, identify, identify_ports,
                              role_from_info)

ORDERS = {'swerve_left': ['FL', 'RL'], 'swerve_right': ['FR', 'RR'], 'arm': ['J1']}
SIDES = {'swerve_left': 'left', 'swerve_right': 'right'}


@contextlib.contextmanager
def pty_board(role, support_i=True, ignore_i=0):
    """role 을 흉내 내는 OpenCR 한 대.

    support_i=False 면 구 펌웨어(? 만 응답). ignore_i=N 이면 처음 N 번의 I 를
    무시한다(포트를 열 때 보드가 리셋되어 부팅 중인 상황).
    """
    master, slave = os.openpty()
    name = os.ttyname(slave)
    stop = threading.Event()
    seen_i = [0]

    def reply(obj):
        try:
            os.write(master, (json.dumps(obj) + '\n').encode())
        except OSError:
            stop.set()

    def serve():
        while not stop.is_set():
            try:
                data = os.read(master, 1024)
            except OSError:
                return
            if not data:
                return
            for line in data.split(b'\n'):
                line = line.strip()
                if line.startswith(b'I') and support_i:
                    seen_i[0] += 1
                    if seen_i[0] <= ignore_i:
                        continue
                    reply({'hello': 1, 'proto': 1, 'board': role,
                           'side': SIDES.get(role), 'kind': role.split('_')[0],
                           'fw': 'opencr_' + role + ' 0.2.2', 'order': ORDERS[role]})
                elif line.startswith(b'?'):
                    reply({'cfg': 1, 'board': role, 'side': SIDES.get(role),
                           'fw': 'opencr_' + role + ' 0.2.1', 'order': ORDERS[role]})

    thread = threading.Thread(target=serve, daemon=True)
    thread.start()
    try:
        yield name
    finally:
        stop.set()
        for fd in (slave, master):
            try:
                os.close(fd)
            except OSError:
                pass


def test_role_prefers_board_field():
    assert role_from_info({'board': 'arm', 'side': 'left', 'fw': 'x'}) == 'arm'


def test_role_falls_back_to_side():
    assert role_from_info({'side': 'left'}) == 'swerve_left'
    assert role_from_info({'side': 'right'}) == 'swerve_right'


def test_role_falls_back_to_firmware_name():
    assert role_from_info({'fw': 'opencr_swerve_right 0.2.1'}) == 'swerve_right'


def test_role_unknown():
    assert role_from_info({'fw': 'something_else'}) is None


def test_identify_new_firmware():
    with pty_board('swerve_left') as port:
        info = identify(port, timeout_s=0.5)
    assert info['hello'] == 1
    assert info['board'] == 'swerve_left'
    assert info['order'] == ['FL', 'RL']


def test_identify_retries_when_board_boots_late():
    # 포트를 열 때 보드가 리셋되면 첫 I 를 놓친다. 두 번째 I 에 답해도 찾아야 한다.
    with pty_board('swerve_left', ignore_i=2) as port:
        info = identify(port, timeout_s=0.8)
    assert info['hello'] == 1
    assert info['board'] == 'swerve_left'


def test_identify_old_firmware_uses_question_mark():
    with pty_board('swerve_right', support_i=False) as port:
        info = identify(port, timeout_s=0.6)
    assert info['cfg'] == 1
    assert role_from_info(info) == 'swerve_right'


def test_identify_ports_ignores_unrelated_board():
    with pty_board('swerve_left') as left, \
            pty_board('swerve_right') as right, \
            pty_board('arm') as arm:
        infos = identify_ports([left, right, arm], timeout_s=0.5)
        boards, dups = assign_roles(infos, ['swerve_left', 'swerve_right'])
    assert set(boards) == {'swerve_left', 'swerve_right'}
    assert role_from_info(boards['swerve_left'][1]) == 'swerve_left'
    assert role_from_info(boards['swerve_right'][1]) == 'swerve_right'
    assert not dups


def test_duplicate_role_is_reported():
    infos = {'/dev/null': {'board': 'swerve_left'},
             '/dev/zero': {'board': 'swerve_left'}}
    boards, dups = assign_roles(infos, ['swerve_left', 'swerve_right'])
    assert len(boards) == 1
    assert len(dups) == 1 and dups[0][0] == 'swerve_left'
