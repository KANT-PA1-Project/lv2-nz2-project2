"""신호 문자열 + 바코드 → L1·L2·L3 배정 (발제 v5 §02).

- R=빨강 그룹, G=초록 그룹, Y=모양 통일 그룹(색 무관, 한 모양)
- 바코드: P2-01/P2-03 오름차순(적은 횟수부터), P2-02/P2-04 내림차순
- 동률이면 R → Y → G 순서 유지(안정 정렬). 정렬 대상은 그룹 횟수(퍽 ID·장소 번호 아님)
- 바코드 의미 표는 설정 파일(mission.yaml barcode_map)에서 읽는다. 아래는 기본값.
"""
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional

COLORS = ("R", "Y", "G")
LOCATIONS = ("L1", "L2", "L3")
DEFAULT_BARCODE_MAP = {"P2-01": "asc", "P2-03": "asc", "P2-02": "desc", "P2-04": "desc"}

# 그룹 → (요구 색, 모양 통일 여부). 퍽 색은 'red'/'green' 2색.
GROUP_RULE = {"R": ("red", False), "G": ("green", False), "Y": (None, True)}


@dataclass(frozen=True)
class GroupAssignment:
    location: str      # L1/L2/L3
    group: str         # R/Y/G
    count: int         # 주사위 눈 수 = 필요한 퍽 수
    color: Optional[str]
    same_shape: bool

    def to_dict(self) -> Dict:
        return asdict(self)


class SpecError(ValueError):
    pass


def count_colors(seq: str) -> Dict[str, int]:
    return {c: seq.count(c) for c in COLORS}


def validate_sequence(seq: str, total: int = 12, cmin: int = 1, cmax: int = 6) -> Dict[str, int]:
    if len(seq) != total or any(ch not in COLORS for ch in seq):
        raise SpecError(f"신호 문자열 형식 오류: {seq!r}")
    c = count_colors(seq)
    if not all(cmin <= v <= cmax for v in c.values()):
        raise SpecError(f"횟수 범위 위반: {c}")
    return c


def barcode_direction(code: str, barcode_map: Optional[Dict[str, str]] = None) -> str:
    m = barcode_map or DEFAULT_BARCODE_MAP
    if code not in m:
        raise SpecError(f"미등록 바코드: {code!r}")
    d = m[code]
    if d not in ("asc", "desc"):
        raise SpecError(f"바코드 의미 오류: {code} → {d}")
    return d


def order_groups(counts: Dict[str, int], direction: str) -> List[str]:
    """안정 정렬: 동률이면 R→Y→G 유지 (오름·내림 모두)."""
    idx = {c: i for i, c in enumerate(COLORS)}
    sign = 1 if direction == "asc" else -1
    return sorted(COLORS, key=lambda c: (sign * counts[c], idx[c]))


def assign_locations(seq: str, barcode: str, barcode_map: Optional[Dict[str, str]] = None) -> List[GroupAssignment]:
    counts = validate_sequence(seq)
    direction = barcode_direction(barcode, barcode_map)
    out = []
    for loc, g in zip(LOCATIONS, order_groups(counts, direction)):
        color, same_shape = GROUP_RULE[g]
        out.append(GroupAssignment(loc, g, counts[g], color, same_shape))
    return out
