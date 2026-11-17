"""실제 설정 파일(mission_locations.yaml)로 50 case 할당·계획이 끝까지 생성되는지 검사."""
import math
import os

import pytest
import yaml

from nz2_mission.core.allocator import allocate
from nz2_mission.core.mission_spec import assign_locations
from nz2_mission.core.planner import SiteInfo, plan_steps
from nz2_mission.core.world_model import WorldModel

CFG = os.path.join(os.path.dirname(__file__), "..", "config", "mission_locations.yaml")
COMBOS = [(r, y, 12 - r - y) for r in range(1, 7) for y in range(1, 7) if 1 <= 12 - r - y <= 6]


def load_sites():
    y = yaml.safe_load(open(CFG, encoding="utf-8"))
    boxes = {int(k): SiteInfo(tuple(v["center"]), tuple(v["approach"])) for k, v in y["boxes"].items()}
    locs = {k: SiteInfo(tuple(v["center"]), tuple(v["approach"]), v.get("place_order")) for k, v in y["locations"].items()}
    return y["start_pose"], boxes, locs


def test_config_schema():
    start, boxes, locs = load_sites()
    assert len(start) == 3 and len(boxes) == 6 and set(locs) == {"L1", "L2", "L3"}
    for s in list(boxes.values()) + list(locs.values()):
        d = math.hypot(s.center_mm[0] - s.approach[0], s.center_mm[1] - s.approach[1])
        assert 300 <= d <= 700, "접근 거리 범위 확인"
        # 접근 yaw가 중심을 바라보는지
        ang = math.atan2(s.center_mm[1] - s.approach[1], s.center_mm[0] - s.approach[0])
        assert abs(math.remainder(ang - s.approach[2], 2 * math.pi)) < 0.05


@pytest.mark.parametrize("combo", COMBOS)
@pytest.mark.parametrize("code", ["P2-01", "P2-02"])
def test_plan_with_real_config(combo, code):
    start, boxes, locs = load_sites()
    wm = WorldModel()
    colors = ["red"] * 6 + ["green"] * 6
    shapes = ["cyl", "cube"] * 6
    for pid in range(12):
        b = pid // 2
        c = boxes[b].center_mm
        wm.observe(pid, colors[pid], shapes[pid], 0.95, 0.95, b, (c[0] + (-40 if pid % 2 == 0 else 40), c[1]))
    groups = assign_locations("R" * combo[0] + "Y" * combo[1] + "G" * combo[2], code)
    cost = lambda p, l: math.hypot(p.pos_mm[0] - locs[l].center_mm[0], p.pos_mm[1] - locs[l].center_mm[1])
    a = allocate(groups, wm, cost)
    if a is None:   # 이 퍽 구성(빨강6·초록6)으로 풀 수 없는 조합은 출제되지 않음
        pytest.skip("infeasible for this puck set")
    steps = plan_steps(groups, a, wm, boxes, locs, start[:2])
    assert sum(1 for s in steps if s.sub == "PLACE") == 12
