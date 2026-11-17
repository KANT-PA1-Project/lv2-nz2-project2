"""퍽 12개 상태표 (단일 진실 원천).

규칙: 인지 노드는 관측(observe)만 보낸다. 상태 변경(mark_*)은 mission_fsm의 전이에서만 한다.
"""
from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple


class PuckStatus(str, Enum):
    UNKNOWN = "UNKNOWN"        # 아직 못 봄
    IN_BOX = "IN_BOX"
    IN_GRIPPER = "IN_GRIPPER"
    IN_TRAY = "IN_TRAY"
    PLACED = "PLACED"
    DROPPED = "DROPPED"        # 떨어짐(회수 가능 후보)
    LOST = "LOST"              # 회수 실패·도달 불가
    UNUSABLE = "UNUSABLE"      # 파지 반복 실패 등


# 할당 가능한 퍽: 박스·트레이·그리퍼에 있는 것. DROPPED는 RECOVER 성공 후(IN_GRIPPER)에만 다시 후보가 된다.
USABLE_SOURCES = (PuckStatus.IN_BOX, PuckStatus.IN_TRAY, PuckStatus.IN_GRIPPER)


@dataclass
class Puck:
    pid: int
    color: Optional[str] = None      # 'red' / 'green'
    shape: Optional[str] = None      # 'cyl' / 'cube'
    conf_color: float = 0.0
    conf_shape: float = 0.0
    box: Optional[int] = None        # 시작 박스 번호(0~5)
    pos_mm: Optional[Tuple[float, float]] = None
    status: PuckStatus = PuckStatus.UNKNOWN
    tray_slot: Optional[int] = None
    placed_at: Optional[Tuple[str, Tuple[int, int]]] = None   # (L, (col,row))
    n_obs: int = 0
    conflict: str = ""               # 관측 충돌 사유(중복 ID 의심 등)

    def to_dict(self) -> Dict:
        d = asdict(self)
        d["status"] = self.status.value
        return d

    @staticmethod
    def from_dict(d: Dict) -> "Puck":
        d = dict(d)
        d["status"] = PuckStatus(d["status"])
        if d.get("pos_mm") is not None:
            d["pos_mm"] = tuple(d["pos_mm"])
        if d.get("placed_at") is not None:
            d["placed_at"] = (d["placed_at"][0], tuple(d["placed_at"][1]))
        return Puck(**d)


@dataclass
class Issue:
    kind: str           # missing / duplicate / unknown_attr / low_conf / count
    pid: Optional[int]
    detail: str
    box: Optional[int] = None


@dataclass
class WorldModel:
    expected_ids: List[int] = field(default_factory=lambda: list(range(12)))
    tray_capacity: int = 4
    min_conf: float = 0.7
    dup_dist_mm: float = 150.0
    pucks: Dict[int, Puck] = field(default_factory=dict)
    tray: List[Optional[int]] = field(default_factory=list)
    gripper: Optional[int] = None
    unexpected_ids: List[int] = field(default_factory=list)

    def __post_init__(self):
        if not self.tray:
            self.tray = [None] * self.tray_capacity

    # ------------------------------------------------------------ 관측
    def observe(self, pid: int, color: Optional[str], shape: Optional[str],
                conf_color: float, conf_shape: float, box: Optional[int],
                pos_mm: Optional[Tuple[float, float]]) -> None:
        if pid not in self.expected_ids:
            if pid not in self.unexpected_ids:
                self.unexpected_ids.append(pid)
            return
        p = self.pucks.get(pid)
        if p is None:
            p = Puck(pid)
            self.pucks[pid] = p
        if p.status not in (PuckStatus.UNKNOWN, PuckStatus.IN_BOX):
            return  # 운반 중·배치된 퍽은 SURVEY 관측으로 덮어쓰지 않는다
        if p.pos_mm is not None and pos_mm is not None and p.n_obs > 0:
            d = ((p.pos_mm[0] - pos_mm[0]) ** 2 + (p.pos_mm[1] - pos_mm[1]) ** 2) ** 0.5
            if d > self.dup_dist_mm:
                p.conflict = f"duplicate_id_far_{d:.0f}mm"
        # 신뢰도가 더 높은 관측으로 속성 갱신
        if color is not None and conf_color >= p.conf_color:
            p.color, p.conf_color = color, conf_color
        if shape is not None and conf_shape >= p.conf_shape:
            p.shape, p.conf_shape = shape, conf_shape
        if pos_mm is not None:
            p.pos_mm = tuple(pos_mm)
        if box is not None:
            p.box = box
        p.n_obs += 1
        p.status = PuckStatus.IN_BOX

    def clear_conflict(self, pid: int) -> None:
        if pid in self.pucks:
            self.pucks[pid].conflict = ""

    def validate(self) -> List[Issue]:
        issues: List[Issue] = []
        for pid in self.expected_ids:
            p = self.pucks.get(pid)
            if p is None or p.status == PuckStatus.UNKNOWN:
                issues.append(Issue("missing", pid, "관측되지 않음"))
                continue
            if p.conflict:
                issues.append(Issue("duplicate", pid, p.conflict, p.box))
            if p.color is None or p.shape is None:
                issues.append(Issue("unknown_attr", pid, f"color={p.color} shape={p.shape}", p.box))
            elif min(p.conf_color, p.conf_shape) < self.min_conf:
                issues.append(Issue("low_conf", pid, f"conf={p.conf_color:.2f}/{p.conf_shape:.2f}", p.box))
        for pid in self.unexpected_ids:
            issues.append(Issue("duplicate", pid, "목록에 없는 ID 관측"))
        return issues

    def boxes_to_resurvey(self, issues: List[Issue], all_boxes: List[int]) -> List[int]:
        """재탐색 대상 박스. 위치를 모르는 퍽(missing)이 있으면 전체."""
        if any(i.kind == "missing" for i in issues):
            return list(all_boxes)
        boxes = sorted({i.box for i in issues if i.box is not None})
        return boxes or list(all_boxes)

    def low_conf_pids(self, n: int) -> List[int]:
        cand = [p for p in self.pucks.values() if p.status == PuckStatus.IN_BOX]
        cand.sort(key=lambda p: min(p.conf_color, p.conf_shape))
        return [p.pid for p in cand[:n]]

    # ------------------------------------------------------------ 상태 전이 (FSM 전용)
    def free_slot(self) -> Optional[int]:
        for i, v in enumerate(self.tray):
            if v is None:
                return i
        return None

    def mark_picked(self, pid: int) -> None:
        p = self.pucks[pid]
        if p.tray_slot is not None:
            self.tray[p.tray_slot] = None
            p.tray_slot = None
        p.status, self.gripper = PuckStatus.IN_GRIPPER, pid

    def mark_loaded(self, pid: int, slot: int) -> None:
        if self.tray[slot] is not None:
            raise RuntimeError(f"트레이 {slot}칸 이미 사용 중")
        p = self.pucks[pid]
        p.status, p.tray_slot = PuckStatus.IN_TRAY, slot
        self.tray[slot] = pid
        if self.gripper == pid:
            self.gripper = None

    def mark_placed(self, pid: int, loc: str, cell: Tuple[int, int]) -> None:
        p = self.pucks[pid]
        if p.tray_slot is not None:
            self.tray[p.tray_slot] = None
        p.status, p.tray_slot, p.placed_at = PuckStatus.PLACED, None, (loc, tuple(cell))
        if self.gripper == pid:
            self.gripper = None

    def mark_dropped(self, pid: int, pos_mm: Optional[Tuple[float, float]] = None) -> None:
        p = self.pucks[pid]
        if p.tray_slot is not None:
            self.tray[p.tray_slot] = None
            p.tray_slot = None
        if self.gripper == pid:
            self.gripper = None
        p.status = PuckStatus.DROPPED
        if pos_mm is not None:
            p.pos_mm = tuple(pos_mm)

    def mark_lost(self, pid: int) -> None:
        self.mark_dropped(pid)
        self.pucks[pid].status = PuckStatus.LOST

    def mark_unusable(self, pid: int) -> None:
        self.mark_dropped(pid)
        self.pucks[pid].status = PuckStatus.UNUSABLE

    def mark_unplaced(self, pid: int) -> None:
        """VERIFY 보정 등으로 배치에서 빠진 퍽을 다시 집은 경우."""
        p = self.pucks[pid]
        p.placed_at = None
        p.status, self.gripper = PuckStatus.IN_GRIPPER, pid

    # ------------------------------------------------------------ 조회
    def placed(self) -> Dict[str, List[Tuple[int, Tuple[int, int]]]]:
        out: Dict[str, List[Tuple[int, Tuple[int, int]]]] = {}
        for p in self.pucks.values():
            if p.status == PuckStatus.PLACED and p.placed_at:
                out.setdefault(p.placed_at[0], []).append((p.pid, p.placed_at[1]))
        return out

    def carried(self) -> List[int]:
        ids = [pid for pid in self.tray if pid is not None]
        if self.gripper is not None:
            ids.append(self.gripper)
        return ids

    # ------------------------------------------------------------ 직렬화
    def to_dict(self) -> Dict:
        return {"expected_ids": self.expected_ids, "tray_capacity": self.tray_capacity,
                "min_conf": self.min_conf, "dup_dist_mm": self.dup_dist_mm,
                "pucks": {str(k): v.to_dict() for k, v in self.pucks.items()},
                "tray": self.tray, "gripper": self.gripper, "unexpected_ids": self.unexpected_ids}

    @staticmethod
    def from_dict(d: Dict) -> "WorldModel":
        wm = WorldModel(d["expected_ids"], d["tray_capacity"], d["min_conf"], d["dup_dist_mm"])
        wm.pucks = {int(k): Puck.from_dict(v) for k, v in d["pucks"].items()}
        wm.tray, wm.gripper, wm.unexpected_ids = d["tray"], d["gripper"], d["unexpected_ids"]
        return wm
