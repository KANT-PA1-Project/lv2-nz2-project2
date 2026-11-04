"""퍽 → 장소 전체 할당 (이동 전 확정, 발제 요구).

제약
- R 그룹: 빨강 n개, G 그룹: 초록 n개, Y 그룹: 같은 모양 n개(색 무관)
- 한 퍽은 한 장소에만. 이미 놓인 퍽은 고정 조건(재계획 시).
방법
- Y 후보 조합을 전수 열거(퍽 ≤12 → 최대 2×C(12,6)=1848가지) 후,
  남은 퍽에서 R·G는 비용이 가장 작은 것부터 고른다(Y가 정해지면 R·G 선택은 서로 독립이라 최적).
- 해가 없으면 None + 진단(어느 조건이 막았는지, 의심 퍽)을 돌려준다 → FSM이 SURVEY 재탐색.
- 부분 할당: 완성 가능한 장소를 최대한 많이(그다음 비용 최소) 고른다.
"""
from dataclasses import dataclass, field
from itertools import combinations
from typing import Callable, Dict, List, Optional, Sequence

from .mission_spec import GroupAssignment
from .world_model import Puck, WorldModel, USABLE_SOURCES

CostFn = Callable[[Puck, str], float]


@dataclass
class Allocation:
    by_pid: Dict[int, str] = field(default_factory=dict)       # pid → L
    locations: List[str] = field(default_factory=list)         # 이번 할당이 완성시키는 장소
    skipped: List[str] = field(default_factory=list)           # 부분 할당에서 포기한 장소
    cost: float = 0.0

    def to_dict(self) -> Dict:
        return {"by_pid": {str(k): v for k, v in self.by_pid.items()}, "locations": self.locations,
                "skipped": self.skipped, "cost": self.cost}

    @staticmethod
    def from_dict(d: Dict) -> "Allocation":
        return Allocation({int(k): v for k, v in d["by_pid"].items()}, d["locations"], d["skipped"], d["cost"])


@dataclass
class Diagnosis:
    reasons: List[str]
    suspect_pids: List[int]


def _candidates(wm: WorldModel) -> List[Puck]:
    return [p for p in wm.pucks.values()
            if p.status in USABLE_SOURCES and p.color is not None and p.shape is not None]


def _solve(groups: Sequence[GroupAssignment], placed: Dict[str, List[Puck]],
           cands: List[Puck], cost: CostFn) -> Optional[Allocation]:
    need = {g.location: g.count - len(placed.get(g.location, [])) for g in groups}
    if any(v < 0 for v in need.values()):
        return None
    yg = next((g for g in groups if g.same_shape), None)
    shapes: List[Optional[str]] = [None]
    if yg is not None:
        fixed = {p.shape for p in placed.get(yg.location, [])}
        if len(fixed) > 1:
            return None
        shapes = list(fixed) if fixed else ["cyl", "cube"]
    best: Optional[Allocation] = None
    for shape in shapes:
        y_choices = [()]
        if yg is not None:
            pool = [p for p in cands if p.shape == shape]
            if len(pool) < need[yg.location]:
                continue
            y_choices = combinations(pool, need[yg.location])
        for ysel in y_choices:
            used = {p.pid for p in ysel}
            alloc = Allocation(by_pid={p.pid: yg.location for p in ysel} if yg else {})
            total = sum(cost(p, yg.location) for p in ysel) if yg else 0.0
            ok = True
            for g in groups:
                if g.same_shape:
                    continue
                pool = sorted((p for p in cands if p.pid not in used and p.color == g.color),
                              key=lambda p: cost(p, g.location))
                n = need[g.location]
                if len(pool) < n:
                    ok = False
                    break
                for p in pool[:n]:
                    alloc.by_pid[p.pid] = g.location
                    used.add(p.pid)
                    total += cost(p, g.location)
            if ok and (best is None or total < best.cost):
                alloc.cost = total
                alloc.locations = [g.location for g in groups]
                best = alloc
    return best


def allocate(groups: Sequence[GroupAssignment], wm: WorldModel, cost: CostFn) -> Optional[Allocation]:
    placed: Dict[str, List[Puck]] = {}
    for loc, items in wm.placed().items():
        placed[loc] = [wm.pucks[pid] for pid, _ in items]
    return _solve(groups, placed, _candidates(wm), cost)


def allocate_partial(groups: Sequence[GroupAssignment], wm: WorldModel, cost: CostFn) -> Optional[Allocation]:
    """전체 해가 없을 때: 완성 가능한 장소 수 최대 → 비용 최소."""
    placed: Dict[str, List[Puck]] = {}
    for loc, items in wm.placed().items():
        placed[loc] = [wm.pucks[pid] for pid, _ in items]
    cands = _candidates(wm)
    all_locs = [g.location for g in groups]
    for k in range(len(groups), 0, -1):
        best = None
        for sub in combinations(groups, k):
            a = _solve(sub, placed, cands, cost)
            if a is not None and (best is None or a.cost < best.cost):
                best = a
        if best is not None:
            best.skipped = [l for l in all_locs if l not in best.locations]
            return best
    return None


def diagnose(groups: Sequence[GroupAssignment], wm: WorldModel, n_suspects: int = 3) -> Diagnosis:
    cands = _candidates(wm)
    reasons = []
    reds = sum(1 for p in cands if p.color == "red")
    greens = sum(1 for p in cands if p.color == "green")
    need_r = sum(g.count for g in groups if g.color == "red")
    need_g = sum(g.count for g in groups if g.color == "green")
    if reds < need_r:
        reasons.append(f"빨강 부족 {reds}/{need_r}")
    if greens < need_g:
        reasons.append(f"초록 부족 {greens}/{need_g}")
    yg = next((g for g in groups if g.same_shape), None)
    if yg:
        for shape in ("cyl", "cube"):
            spare_r = max(0, reds - need_r)
            spare_g = max(0, greens - need_g)
            n_shape = sum(1 for p in cands if p.shape == shape)
            reasons.append(f"{shape} {n_shape}개(여유 빨강 {spare_r}, 초록 {spare_g}) / Y {yg.count}")
    unknown = [p.pid for p in wm.pucks.values() if p.status in USABLE_SOURCES and (p.color is None or p.shape is None)]
    if unknown:
        reasons.append(f"속성 미확정 {unknown}")
    suspects = unknown + [pid for pid in wm.low_conf_pids(n_suspects) if pid not in unknown]
    return Diagnosis(reasons, suspects[:max(n_suspects, len(unknown))])
