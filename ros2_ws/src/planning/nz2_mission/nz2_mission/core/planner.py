"""할당 결과 → 실행 단계(Step) 목록. 트레이 용량 k 기반 운반 계획.

- 한 trip: 박스들을 돌며 줍기·싣기(최대 빈 칸 수) → 장소들을 돌며 꺼내기·놓기·확인
- trip 구성: 주 목적지 L(이미 싣고 있는 퍽의 L 우선, 없으면 남은 퍽이 가장 많은 L)의 퍽을
  가까운 박스부터 채우고, 남는 칸은 다른 L 퍽 중 가까운 것으로 채운다.
- 방문 순서: 박스·장소 각각 전수 순열(≤4!, ≤3!)로 최단.
- 장소 내 놓기 순서: dice_layout.place_order (먼 점 → 가까운 점), trip이 나뉘어도 순서 유지.
- 모든 이동(move) 앞에 travel_pose(이동자세) 확인 단계를 넣는다 → FSM 가드.
"""
import math
from dataclasses import asdict, dataclass, field
from itertools import permutations
from typing import Dict, List, Optional, Sequence, Tuple

from .allocator import Allocation
from .dice_layout import place_order
from .mission_spec import GroupAssignment
from .world_model import PuckStatus, WorldModel

Pose = Tuple[float, float, float]   # x_mm, y_mm, yaw_rad


@dataclass
class Step:
    sub: str                       # 하위 상태 이름 (states.ExecSub 값)
    args: Dict = field(default_factory=dict)

    def to_dict(self) -> Dict:
        return asdict(self)


@dataclass
class SiteInfo:
    center_mm: Tuple[float, float]
    approach: Pose
    place_order_override: Optional[List[List[int]]] = None


def _d(a: Sequence[float], b: Sequence[float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _best_order(start: Sequence[float], keys: List, pos: Dict) -> List:
    if len(keys) <= 1:
        return list(keys)
    best, best_len = None, math.inf
    for perm in permutations(keys):
        cur, L = start, 0.0
        for k in perm:
            L += _d(cur, pos[k])
            cur = pos[k]
        if L < best_len:
            best, best_len = list(perm), L
    return best


def remaining_cells(groups: Sequence[GroupAssignment], wm: WorldModel,
                    locs: Dict[str, SiteInfo]) -> Dict[str, List[Tuple[int, int]]]:
    placed = wm.placed()
    out = {}
    for g in groups:
        taken = {tuple(c) for _, c in placed.get(g.location, [])}
        order = place_order(g.count, locs[g.location].approach[2], locs[g.location].place_order_override)
        out[g.location] = [c for c in order if c not in taken]
    return out


def plan_steps(groups: Sequence[GroupAssignment], alloc: Allocation, wm: WorldModel,
               boxes: Dict[int, SiteInfo], locs: Dict[str, SiteInfo],
               robot_mm: Sequence[float]) -> List[Step]:
    steps: List[Step] = []
    cells = remaining_cells(groups, wm, locs)
    slots = list(wm.tray)                       # 시뮬레이션용 사본
    carry: Dict[int, int] = {pid: i for i, pid in enumerate(slots) if pid is not None}
    pos = tuple(robot_mm[:2])

    # 그리퍼에 퍽이 있으면 먼저 트레이에 싣는다
    if wm.gripper is not None:
        free = next(i for i, v in enumerate(slots) if v is None)
        steps.append(Step("LOAD", {"pid": wm.gripper, "slot": free}))
        slots[free] = wm.gripper
        carry[wm.gripper] = free

    to_pick = [pid for pid, loc in alloc.by_pid.items()
               if wm.pucks[pid].status == PuckStatus.IN_BOX]
    carried_alloc = [pid for pid in carry if pid in alloc.by_pid]
    # 할당에 없는 퍽을 싣고 있으면(재계획 후) 계획에서 제외 → FSM이 경고 로그

    def box_of(pid):
        return wm.pucks[pid].box

    guard = 0
    while to_pick or carried_alloc:
        guard += 1
        if guard > 50:
            raise RuntimeError("계획 생성 무한루프 방지")
        free_n = sum(1 for v in slots if v is None)
        # ---- 주 목적지 선택
        if carried_alloc:
            primary = alloc.by_pid[carried_alloc[0]]
        else:
            cnt: Dict[str, int] = {}
            for pid in to_pick:
                cnt[alloc.by_pid[pid]] = cnt.get(alloc.by_pid[pid], 0) + 1
            primary = max(cnt, key=lambda l: (cnt[l], -_d(pos, locs[l].center_mm)))
        # ---- 이번 trip에 실을 퍽
        pool = sorted([p for p in to_pick if alloc.by_pid[p] == primary],
                      key=lambda p: _d(boxes[box_of(p)].center_mm, locs[primary].center_mm))
        pool += sorted([p for p in to_pick if alloc.by_pid[p] != primary],
                       key=lambda p: _d(boxes[box_of(p)].center_mm, locs[primary].center_mm))
        chosen = pool[:free_n]
        # ---- 박스 방문
        bset = sorted({box_of(p) for p in chosen})
        bpos = {b: boxes[b].approach[:2] for b in bset}
        for b in _best_order(pos, bset, bpos):
            steps.append(Step("TRAVEL_POSE"))
            steps.append(Step("MOVE_TO_PICK", {"box": b, "pose": list(boxes[b].approach)}))
            steps.append(Step("ALIGN_PICK", {"box": b}))
            for pid in [p for p in chosen if box_of(p) == b]:
                slot = next(i for i, v in enumerate(slots) if v is None)
                steps.append(Step("PICK", {"pid": pid, "box": b}))
                steps.append(Step("LOAD", {"pid": pid, "slot": slot}))
                slots[slot] = pid
                carry[pid] = slot
            pos = bpos[b]
        to_pick = [p for p in to_pick if p not in chosen]
        # ---- 장소 방문 (싣고 있는 할당 퍽 전부)
        onboard = [pid for pid in carry if pid in alloc.by_pid]
        lset = sorted({alloc.by_pid[p] for p in onboard})
        lpos = {l: locs[l].approach[:2] for l in lset}
        for l in _best_order(pos, lset, lpos):
            steps.append(Step("TRAVEL_POSE"))
            steps.append(Step("MOVE_TO_PLACE", {"loc": l, "pose": list(locs[l].approach)}))
            steps.append(Step("ALIGN_PLACE", {"loc": l}))
            for pid in [p for p in onboard if alloc.by_pid[p] == l]:
                cell = cells[l].pop(0)
                steps.append(Step("UNLOAD", {"pid": pid, "slot": carry[pid]}))
                steps.append(Step("PLACE", {"pid": pid, "loc": l, "cell": list(cell)}))
                steps.append(Step("CHECK_PLACE", {"pid": pid, "loc": l, "cell": list(cell)}))
                slots[carry.pop(pid)] = None
            pos = lpos[l]
        carried_alloc = [pid for pid in carry if pid in alloc.by_pid]
    return steps
