"""램프 3개 ROI 밝기 → 샘플 라벨(R/Y/G/OFF/AMBIG) 판정.

설계 근거 (docs/mission_fsm_design.md §3):
- 색은 위치로 확정한다(위=R, 가운데=Y, 아래=G, lamp_order 파라미터).
- 기준 밝기(꺼짐)는 램프별 이동 최솟값으로 추정한다. 신호 루프에서 모든 램프는
  2초마다 최소 1초, 프레임 경계에서 3초 꺼지므로 4초 창의 최솟값 = 꺼짐 밝기.
- 번짐(옆 램프로 빛이 샘) 대응: 1위 비율 / 2위 비율 ≥ lit_ratio 일 때만 점등 인정.
"""
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, List, Sequence, Tuple

LABEL_OFF = "OFF"
LABEL_AMBIG = "AMBIG"


@dataclass
class LampClassifierParams:
    lamp_order: Tuple[str, str, str] = ("R", "Y", "G")  # 위→아래
    on_ratio_thresh: float = 2.0   # 기준(꺼짐) 대비 이 배수 이상이면 '켜짐 후보'
    lit_ratio: float = 2.0         # 1위/2위 비율 하한 (번짐 방지)
    baseline_window_s: float = 4.0
    baseline_floor: float = 5.0    # 0 나눗셈·잡음 방지용 최소 기준 밝기


@dataclass
class LampClassifier:
    params: LampClassifierParams = field(default_factory=LampClassifierParams)
    _hist: List[Deque[Tuple[float, float]]] = field(default_factory=lambda: [deque(), deque(), deque()])

    def reset(self) -> None:
        self._hist = [deque(), deque(), deque()]

    def _baseline(self, i: int, t: float, value: float) -> float:
        h = self._hist[i]
        h.append((t, value))
        while h and t - h[0][0] > self.params.baseline_window_s:
            h.popleft()
        return max(self.params.baseline_floor, min(v for _, v in h))

    def classify(self, t: float, brightness: Sequence[float]) -> str:
        """brightness: 위→아래 순서 램프 3개의 평균 밝기(0~255)."""
        if len(brightness) != 3:
            raise ValueError("brightness는 램프 3개 값이어야 한다")
        ratios = [brightness[i] / self._baseline(i, t, float(brightness[i])) for i in range(3)]
        order = sorted(range(3), key=lambda i: ratios[i], reverse=True)
        top, second = ratios[order[0]], ratios[order[1]]
        if top < self.params.on_ratio_thresh:
            return LABEL_OFF
        if second > 0 and top / second < self.params.lit_ratio:
            return LABEL_AMBIG
        return self.params.lamp_order[order[0]]
