"""테스트·리허설용 무한루프 신호 시뮬레이터 (ROS 불필요).

실제 카메라 대신 '촬영 시각 + 램프 3개 밝기' 샘플열을 만든다.
불규칙 프레임레이트, 프레임 드롭, 점등 놓침, 번짐(옆 램프 밝아짐)을 주입할 수 있다.
"""
import random
from dataclasses import dataclass
from typing import Iterator, List, Optional, Set, Tuple

LAMP_INDEX = {"R": 0, "Y": 1, "G": 2}


@dataclass
class SimConfig:
    sequence: str = "RGRRGRGGYGYG"
    on_s: float = 1.0
    off_s: float = 1.0
    gap_s: float = 3.0              # 12번째 뒤 소등 총 길이
    start_offset_s: float = 0.0     # 관측 시작이 루프의 어디인지(중간 진입)
    duration_s: float = 140.0
    fps: float = 30.0
    fps_jitter: float = 0.0         # 0~1, 샘플 간격 무작위 변동 비율
    drop_prob: float = 0.0          # 샘플 드롭 확률
    on_level: float = 220.0
    off_level: float = 25.0
    bloom: float = 0.0              # 켜진 램프 이웃 램프에 더해지는 밝기
    noise: float = 3.0
    missed_blinks: Optional[Set[Tuple[int, int]]] = None  # {(루프번호, 점멸번호)} 점등 안 됨
    seed: int = 1

    @property
    def period(self) -> float:
        n = len(self.sequence)
        return n * self.on_s + (n - 1) * self.off_s + self.gap_s


def lit_color_at(cfg: SimConfig, t: float) -> Optional[str]:
    """시각 t(루프 기준)에 켜진 색. 없으면 None."""
    loop = int(t // cfg.period)
    x = t - loop * cfg.period
    step = cfg.on_s + cfg.off_s
    k = int(x // step)
    if k >= len(cfg.sequence):
        return None
    if x - k * step < cfg.on_s:
        if cfg.missed_blinks and (loop, k) in cfg.missed_blinks:
            return None
        return cfg.sequence[k]
    return None


def samples(cfg: SimConfig) -> Iterator[Tuple[float, List[float]]]:
    rng = random.Random(cfg.seed)
    t = 0.0
    dt0 = 1.0 / cfg.fps
    while t < cfg.duration_s:
        if rng.random() >= cfg.drop_prob:
            lit = lit_color_at(cfg, t + cfg.start_offset_s)
            vals = [cfg.off_level] * 3
            if lit is not None:
                i = LAMP_INDEX[lit]
                vals[i] = cfg.on_level
                for j in (i - 1, i + 1):
                    if 0 <= j < 3:
                        vals[j] += cfg.bloom
            vals = [max(0.0, v + rng.uniform(-cfg.noise, cfg.noise)) for v in vals]
            yield t, vals
        t += dt0 * (1.0 + rng.uniform(-cfg.fps_jitter, cfg.fps_jitter))


def all_valid_sequences_sample(n: int, seed: int = 0) -> List[str]:
    """규칙(각 1~6, 합 12)을 만족하는 무작위 순서 문자열 n개."""
    rng = random.Random(seed)
    out = []
    combos = [(r, y, 12 - r - y) for r in range(1, 7) for y in range(1, 7) if 1 <= 12 - r - y <= 6]
    for i in range(n):
        r, y, g = combos[i % len(combos)]
        letters = list("R" * r + "Y" * y + "G" * g)
        rng.shuffle(letters)
        out.append("".join(letters))
    return out
