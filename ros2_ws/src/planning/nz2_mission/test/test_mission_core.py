"""판단 코어 테스트: 50 case 배정, 주사위 좌표, 할당, 계획, 체크포인트, 재진입."""
import math
import random

import pytest

from nz2_mission.core import checkpoint
from nz2_mission.core.allocator import allocate, allocate_partial, diagnose
from nz2_mission.core.dice_layout import PIPS, cell_offset, place_order
from nz2_mission.core.mission_spec import SpecError, assign_locations, order_groups, validate_sequence
from nz2_mission.core.planner import plan_steps
from nz2_mission.core.states import Phase, resume_entry
from nz2_mission.core.world_model import PuckStatus, WorldModel
from conftest import BOXES

ALL_COMBOS = [(r, y, 12 - r - y) for r in range(1, 7) for y in range(1, 7) if 1 <= 12 - r - y <= 6]


def seq_from(r, y, g):
    return "R" * r + "Y" * y + "G" * g


# ------------------------------------------------------------------ 미션 사양
def test_combo_count_is_25():
    assert len(ALL_COMBOS) == 25


def test_q1_ascending():
    a = assign_locations("RGRRGRGGYGYG", "P2-01")
    assert [(x.location, x.group, x.count) for x in a] == [("L1", "Y", 2), ("L2", "R", 4), ("L3", "G", 6)]


def test_q1_descending():
    a = assign_locations("RGRRGRGGYGYG", "P2-04")
    assert [(x.group, x.count) for x in a] == [("G", 6), ("R", 4), ("Y", 2)]


@pytest.mark.parametrize("combo", ALL_COMBOS)
@pytest.mark.parametrize("code", ["P2-01", "P2-02", "P2-03", "P2-04"])
def test_all_50_cases_sorted_and_stable(combo, code):
    a = assign_locations(seq_from(*combo), code)
    counts = [x.count for x in a]
    asc = code in ("P2-01", "P2-03")
    assert counts == sorted(counts, reverse=not asc)
    idx = {"R": 0, "Y": 1, "G": 2}
    for i in range(2):  # 동률이면 R→Y→G
        if counts[i] == counts[i + 1]:
            assert idx[a[i].group] < idx[a[i + 1].group]
    assert sum(counts) == 12


def test_tie_rule_desc():
    assert order_groups({"R": 3, "Y": 3, "G": 6}, "desc") == ["G", "R", "Y"]
    assert order_groups({"R": 3, "Y": 3, "G": 6}, "asc") == ["R", "Y", "G"]


@pytest.mark.parametrize("bad", ["RGRRGRGGYGY", "RRRRRRRGGGGY", "XGRRGRGGYGYG"])
def test_invalid_sequences(bad):
    with pytest.raises(SpecError):
        validate_sequence(bad)


def test_unregistered_barcode():
    with pytest.raises(SpecError):
        assign_locations("RGRRGRGGYGYG", "P9-99")


# ------------------------------------------------------------------ 주사위
def test_pip_offsets():
    assert [cell_offset(*c) for c in PIPS[2]] == [(-60, 60), (60, -60)]
    assert len(PIPS[6]) == 6 and all(c[0] in (0, 2) for c in PIPS[6])


def test_place_order_far_first():
    # +Y 방향을 보며 접근(남쪽에서 접근) → 먼 줄 = 위쪽 행(row 0)
    order = place_order(6, math.pi / 2)
    assert [c[1] for c in order] == [0, 0, 1, 1, 2, 2]


# ------------------------------------------------------------------ 할당
def make_world(colors, shapes, conf=0.95):
    wm = WorldModel()
    for pid in range(12):
        b = pid // 2
        wm.observe(pid, colors[pid], shapes[pid], conf, conf, b,
                   (BOXES[b][0] + (-40 if pid % 2 == 0 else 40), BOXES[b][1]))
    return wm


def cost_fn(locs):
    return lambda p, l: math.hypot(p.pos_mm[0] - locs[l].center_mm[0], p.pos_mm[1] - locs[l].center_mm[1])


def brute_feasible(groups, wm):
    """퍽 12개를 모두 쓰므로 Y의 빨강 수 yr가 정해지면 R·G 수가 맞아야 한다."""
    pucks = list(wm.pucks.values())
    reds = [p for p in pucks if p.color == "red"]
    greens = [p for p in pucks if p.color == "green"]
    g = {x.group: x.count for x in groups}
    for shape in ("cyl", "cube"):
        for yr in range(g["Y"] + 1):
            yg = g["Y"] - yr
            if len(reds) - yr == g["R"] and len(greens) - yg == g["G"]:
                if sum(1 for p in reds if p.shape == shape) >= yr and sum(1 for p in greens if p.shape == shape) >= yg:
                    return True
    return False


def test_q1_alloc_y_takes_same_shape(locs):
    colors = ["red"] * 6 + ["green"] * 6
    shapes = ["cyl", "cube", "cube", "cube", "cube", "cyl"] + ["cyl", "cube"] * 3
    wm = make_world(colors, shapes)
    groups = assign_locations("RGRRGRGGYGYG", "P2-01")
    a = allocate(groups, wm, cost_fn(locs))
    assert a is not None
    y = [pid for pid, l in a.by_pid.items() if l == "L1"]
    assert len(y) == 2 and len({wm.pucks[p].shape for p in y}) == 1
    assert sum(1 for l in a.by_pid.values() if l == "L2") == 4
    assert all(wm.pucks[p].color == "red" for p, l in a.by_pid.items() if l == "L2")
    assert len(a.by_pid) == 12


@pytest.mark.parametrize("seed", range(40))
def test_random_worlds_match_bruteforce(seed, locs):
    rng = random.Random(seed)
    combo = rng.choice(ALL_COMBOS)
    groups = assign_locations(seq_from(*combo), rng.choice(["P2-01", "P2-02"]))
    n_red = rng.randint(3, 9)
    colors = ["red"] * n_red + ["green"] * (12 - n_red)
    rng.shuffle(colors)
    shapes = [rng.choice(["cyl", "cube"]) for _ in range(12)]
    wm = make_world(colors, shapes)
    a = allocate(groups, wm, cost_fn(locs))
    assert (a is not None) == brute_feasible(groups, wm)
    if a is None:
        d = diagnose(groups, wm)
        assert d.reasons


def test_partial_allocation(locs):
    colors = ["red"] * 6 + ["green"] * 6
    shapes = ["cyl"] * 12
    wm = make_world(colors, shapes)
    wm.mark_lost(0)  # 빨강 하나 손실
    groups = assign_locations("RGRRGRGGYGYG", "P2-01")  # Y2 R4 G6 → 빨강 6 필요
    assert allocate(groups, wm, cost_fn(locs)) is None
    p = allocate_partial(groups, wm, cost_fn(locs))
    assert p is not None and len(p.locations) == 2 and p.skipped


def test_unknown_attr_blocks_and_diagnoses(locs):
    colors = ["red"] * 6 + ["green"] * 6
    shapes = ["cyl", "cube"] * 6
    wm = make_world(colors, shapes)
    wm.pucks[3].shape = None
    groups = assign_locations("RGRRGRGGYGYG", "P2-01")
    d = diagnose(groups, wm)
    assert 3 in d.suspect_pids


def test_validate_missing_and_duplicate():
    wm = WorldModel()
    wm.observe(0, "red", "cyl", 0.9, 0.9, 0, (1428, 950))
    wm.observe(0, "red", "cyl", 0.9, 0.9, 3, (734, 5354))   # 같은 ID가 먼 곳에서
    kinds = {i.kind for i in wm.validate()}
    assert "missing" in kinds and "duplicate" in kinds


# ------------------------------------------------------------------ 계획
@pytest.mark.parametrize("combo", ALL_COMBOS)
def test_plan_all_combos_capacity_and_coverage(combo, boxes, locs):
    colors = ["red"] * 6 + ["green"] * 6
    shapes = ["cyl", "cube"] * 6
    wm = make_world(colors, shapes)
    for code in ("P2-01", "P2-02"):
        groups = assign_locations(seq_from(*combo), code)
        a = allocate(groups, wm, cost_fn(locs))
        if a is None:
            continue
        steps = plan_steps(groups, a, wm, boxes, locs, (734, 950))
        load = 0
        placed = set()
        for i, s in enumerate(steps):
            if s.sub == "LOAD":
                load += 1
                assert load <= wm.tray_capacity
            if s.sub == "UNLOAD":
                load -= 1
            if s.sub.startswith("MOVE"):
                assert steps[i - 1].sub == "TRAVEL_POSE"   # 이동자세 가드
            if s.sub == "PLACE":
                key = (s.args["loc"], tuple(s.args["cell"]))
                assert key not in placed
                placed.add(key)
        assert len(placed) == 12


def test_plan_trip_count_q1(boxes, locs):
    colors = ["red"] * 6 + ["green"] * 6
    shapes = ["cyl", "cube"] * 6
    wm = make_world(colors, shapes)
    groups = assign_locations("RGRRGRGGYGYG", "P2-01")
    steps = plan_steps(groups, allocate(groups, wm, cost_fn(locs)), wm, boxes, locs, (734, 950))
    # trip 수 = 첫 PICK 1회 + (놓기 확인 뒤 다시 PICK이 시작된 횟수)
    subs = [s.sub for s in steps if s.sub in ("PICK", "CHECK_PLACE")]
    trips = 1 + sum(1 for a, b in zip(subs, subs[1:]) if a == "CHECK_PLACE" and b == "PICK")
    assert 3 <= trips <= 5   # 12개 / 4칸 = 최소 3 trip


def test_plan_resumes_with_tray_contents(boxes, locs):
    colors = ["red"] * 6 + ["green"] * 6
    shapes = ["cyl", "cube"] * 6
    wm = make_world(colors, shapes)
    groups = assign_locations("RGRRGRGGYGYG", "P2-01")
    a = allocate(groups, wm, cost_fn(locs))
    pid = next(p for p, l in a.by_pid.items() if l == "L3")
    wm.mark_picked(pid)
    wm.mark_loaded(pid, 0)
    steps = plan_steps(groups, a, wm, boxes, locs, (2000, 2000))
    assert any(s.sub == "UNLOAD" and s.args["pid"] == pid for s in steps)
    assert not any(s.sub == "PICK" and s.args["pid"] == pid for s in steps)


# ------------------------------------------------------------------ 체크포인트·재진입
def test_checkpoint_roundtrip(tmp_path):
    wm = make_world(["red"] * 6 + ["green"] * 6, ["cyl"] * 12)
    wm.mark_picked(1)
    wm.mark_loaded(1, 2)
    wm.mark_placed(2, "L1", (0, 0))
    path = str(tmp_path / "cp.json")
    checkpoint.save_atomic(path, {"world": wm.to_dict()})
    wm2 = WorldModel.from_dict(checkpoint.load(path)["world"])
    assert wm2.tray[2] == 1 and wm2.pucks[2].placed_at == ("L1", (0, 0))
    assert wm2.pucks[1].status == PuckStatus.IN_TRAY


def test_corrupt_checkpoint_returns_none(tmp_path):
    p = tmp_path / "cp.json"
    p.write_text("{broken")
    assert checkpoint.load(str(p)) is None


@pytest.mark.parametrize("phase,expected", [
    ("OBSERVE", Phase.OBSERVE), ("EXECUTE", Phase.PLAN), ("SURVEY", Phase.VALIDATE),
    ("DONE", Phase.DONE), ("HOLD", Phase.RETURN), ("VERIFY", Phase.VERIFY)])
def test_resume_entry(phase, expected):
    assert resume_entry(phase, False, False) == expected


def test_resume_observe_with_confirmed_signal():
    assert resume_entry("OBSERVE", True, True) == Phase.DECODE
