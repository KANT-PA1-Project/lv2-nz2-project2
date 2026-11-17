import math
import pytest

from nz2_mission.core.planner import SiteInfo

# [가정] 좌표: 발제 v5 그림 기준. 접근 자세는 경기장 공개 기간 실측 후 교체.
BOXES = {0: (1468, 950), 1: (3670, 3152), 2: (2936, 5354), 3: (734, 5354), 4: (2202, 5354), 5: (3670, 3886)}
LOCS = {"L1": (3670, 950), "L2": (2202, 3700), "L3": (3670, 5354)}


def approach(c, yaw, d=450.0):
    return (c[0] - d * math.cos(yaw), c[1] - d * math.sin(yaw), yaw)


@pytest.fixture
def boxes():
    yaws = {0: 0.0, 1: 0.0, 2: math.pi / 2, 3: math.pi / 2, 4: math.pi / 2, 5: 0.0}
    return {b: SiteInfo(c, approach(c, yaws[b])) for b, c in BOXES.items()}


@pytest.fixture
def locs():
    yaws = {"L1": 0.0, "L2": -math.pi / 2, "L3": math.pi / 2}
    return {l: SiteInfo(c, approach(c, yaws[l])) for l, c in LOCS.items()}
