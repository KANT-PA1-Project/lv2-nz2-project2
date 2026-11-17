"""주사위 눈 → 장소 중심 기준 격자 좌표(mm), 놓기 순서.

- 3×3 격자: 장소 중심 기준 X,Y 각 -60·0·+60 mm. 지도 위쪽 = +Y. 회전 대체 불가.
- 2·3은 좌상→우하 대각선, 6은 좌우 두 열 (발제 그림 그대로).
- 놓기 순서: 접근 방향 기준 먼 점 → 가까운 점 (이미 놓인 퍽과 손가락 간섭 방지).
"""
import math
from typing import Dict, List, Optional, Sequence, Tuple

PITCH_MM = 60.0
# (열, 행), 행 0 = 위(+Y), 열 0 = 왼쪽(-X)
PIPS: Dict[int, List[Tuple[int, int]]] = {
    1: [(1, 1)],
    2: [(0, 0), (2, 2)],
    3: [(0, 0), (1, 1), (2, 2)],
    4: [(0, 0), (2, 0), (0, 2), (2, 2)],
    5: [(0, 0), (2, 0), (1, 1), (0, 2), (2, 2)],
    6: [(0, 0), (2, 0), (0, 1), (2, 1), (0, 2), (2, 2)],
}


def cell_offset(col: int, row: int, pitch: float = PITCH_MM) -> Tuple[float, float]:
    return ((col - 1) * pitch, (1 - row) * pitch)


def pip_cells(n: int) -> List[Tuple[int, int]]:
    if n not in PIPS:
        raise ValueError(f"주사위 눈은 1~6: {n}")
    return list(PIPS[n])


def place_order(n: int, approach_yaw_rad: float,
                override: Optional[Sequence[Sequence[int]]] = None) -> List[Tuple[int, int]]:
    """approach_yaw: 장소에 접근했을 때 로봇이 바라보는 방향(지도 기준, rad).
    override가 있으면(arena 설정의 place_order) 그 순서에서 해당 눈의 칸만 고른다."""
    cells = pip_cells(n)
    if override:
        ordered = [tuple(c) for c in override if tuple(c) in cells]
        missing = [c for c in cells if c not in ordered]
        return ordered + missing
    fx, fy = math.cos(approach_yaw_rad), math.sin(approach_yaw_rad)
    lx, ly = -fy, fx  # 로봇 기준 왼쪽

    def key(c):
        dx, dy = cell_offset(*c)
        return (-(dx * fx + dy * fy), -(dx * lx + dy * ly))  # 먼 줄 먼저, 같은 줄은 왼쪽 먼저

    return sorted(cells, key=key)


def cell_world_mm(center_mm: Sequence[float], cell: Tuple[int, int]) -> Tuple[float, float]:
    dx, dy = cell_offset(*cell)
    return (center_mm[0] + dx, center_mm[1] + dy)
