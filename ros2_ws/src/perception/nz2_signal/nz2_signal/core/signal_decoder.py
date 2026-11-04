"""무한루프 신호등의 프레임 동기화(SYNC) · 수집(CAPTURE) · 투표(VOTE).

신호 구조(발제 v5 + 운영 확인 2026-10):
- 점멸 1회 = 1초 켜짐 + 1초 소등, 연속 같은 색도 소등으로 구분
- 12번째 점멸 뒤 3초 소등(1+3 아님, 총 3초) → 다음 프레임 시작
- 1주기 = 12 + 11 + 3 = 26초, 신호등은 경기 전부터 무한 반복

판정 원칙:
- 시간은 반드시 촬영 시각(header.stamp)으로 넣는다. 처리 시각 금지.
- 경계 시각 = 라벨이 바뀐 두 샘플의 중간.
- 점등 1개를 놓치면 소등이 정확히 3초가 되어 경계와 구분 불가 → 프레임 점멸 수=12,
  프레임 길이 23±1초 검사로 걸러지고 다음 진짜 경계에서 자동 재동기화된다.
- 의심스러운 것은 버린다(프레임 무효). 자리별 다수결로 문자열을 합성하지 않는다.
"""
from collections import Counter
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from .lamp_classifier import LABEL_AMBIG, LABEL_OFF

COLORS = ("R", "Y", "G")

STAGE_SYNC = "SYNC"
STAGE_CAPTURE = "CAPTURE"
STAGE_CONFIRMED = "CONFIRMED"
STAGE_FAILED = "FAILED"


@dataclass
class SignalParams:
    on_min_s: float = 0.6
    on_max_s: float = 1.4
    off_min_s: float = 0.4          # 이보다 짧은 소등 = 깜빡임 잡음 → 프레임 무효
    intra_off_max_s: float = 1.5
    gap_min_s: float = 2.5
    gap_max_s: float = 3.5
    frame_span_s: float = 23.0      # 첫 점등 시작 ~ 12번째 점등 끝
    frame_span_tol_s: float = 1.0
    blinks_per_frame: int = 12
    count_min: int = 1
    count_max: int = 6
    k_agree: int = 3
    min_margin: int = 2
    max_frames: int = 8             # 유효·무효 합계. 초과 시 FAILED
    max_sample_gap_s: float = 0.3   # 이 이상 샘플 공백이 전환을 걸치면 프레임 무효


@dataclass
class FrameRecord:
    t_start: float
    t_end: float
    sequence: str
    valid: bool
    reason: str = ""
    blink_times: List[List[float]] = field(default_factory=list)  # [시작, 끝]

    def to_dict(self) -> Dict:
        return {"t_start": round(self.t_start, 3), "t_end": round(self.t_end, 3),
                "sequence": self.sequence, "valid": self.valid, "reason": self.reason,
                "blink_times": [[round(a, 3), round(b, 3)] for a, b in self.blink_times]}


def count_colors(seq: str) -> Dict[str, int]:
    return {c: seq.count(c) for c in COLORS}


def counts_valid(seq: str, p: SignalParams) -> bool:
    if len(seq) != p.blinks_per_frame or any(ch not in COLORS for ch in seq):
        return False
    c = count_colors(seq)
    return all(p.count_min <= v <= p.count_max for v in c.values()) and sum(c.values()) == p.blinks_per_frame


class SignalDecoder:
    """샘플을 시간순으로 feed() 한다. stage가 CONFIRMED/FAILED가 되면 종료."""

    def __init__(self, params: Optional[SignalParams] = None):
        self.p = params or SignalParams()
        self.reset()

    # ------------------------------------------------------------------ 상태
    def reset(self) -> None:
        self.stage = STAGE_SYNC
        self.frames: List[FrameRecord] = []
        self.result: Optional[str] = None
        self.fail_reason = ""
        self._prev_t: Optional[float] = None
        self._run_label: Optional[str] = None
        self._run_start: Optional[float] = None
        self._run_unreliable = False   # 현재 구간의 시작 경계가 공백에 걸침
        self._cur: List[str] = []
        self._cur_times: List[List[float]] = []
        self._cur_start: Optional[float] = None
        self._cur_invalid = ""

    @property
    def done(self) -> bool:
        return self.stage in (STAGE_CONFIRMED, STAGE_FAILED)

    def summary(self) -> Dict:
        valid = [f.sequence for f in self.frames if f.valid]
        return {"stage": self.stage, "result": self.result, "fail_reason": self.fail_reason,
                "frames_total": len(self.frames), "frames_valid": len(valid),
                "votes": dict(Counter(valid))}

    # ------------------------------------------------------------------ 입력
    def feed(self, t: float, label: str) -> None:
        if self.done:
            return
        if label not in COLORS and label not in (LABEL_OFF, LABEL_AMBIG):
            raise ValueError(f"알 수 없는 라벨: {label}")
        if self._prev_t is None:
            self._prev_t, self._run_label, self._run_start = t, label, t
            return
        if t <= self._prev_t:
            return  # 역순·중복 타임스탬프 무시
        gap = t - self._prev_t
        if label != self._run_label:
            edge = 0.5 * (self._prev_t + t)
            unreliable_edge = gap > self.p.max_sample_gap_s
            self._close_run(self._run_label, self._run_start, edge, self._run_unreliable or unreliable_edge)
            self._run_label, self._run_start, self._run_unreliable = label, edge, unreliable_edge
        self._prev_t = t

    # ------------------------------------------------------------------ 내부
    def _invalidate(self, reason: str) -> None:
        if self.stage == STAGE_CAPTURE and not self._cur_invalid:
            self._cur_invalid = reason

    def _close_run(self, label: str, start: float, end: float, unreliable: bool) -> None:
        d = end - start
        if label == LABEL_AMBIG:
            self._invalidate("ambiguous_sample")
            return
        if label in COLORS:
            if self.stage != STAGE_CAPTURE:
                return
            if unreliable:
                self._invalidate("sample_gap_at_edge")
            if not (self.p.on_min_s <= d <= self.p.on_max_s):
                self._invalidate(f"on_duration_{d:.2f}")
            if self._cur_start is None:
                self._cur_start = start
            self._cur.append(label)
            self._cur_times.append([start, end])
            if len(self._cur) > self.p.blinks_per_frame:
                self._invalidate("too_many_blinks")
            return
        # ---- OFF 구간
        if unreliable and self.stage == STAGE_CAPTURE:
            self._invalidate("sample_gap_at_edge")
        if d < self.p.off_min_s:
            self._invalidate(f"off_too_short_{d:.2f}")
        elif d <= self.p.intra_off_max_s:
            pass  # 프레임 내부 간격
        elif self.p.gap_min_s <= d <= self.p.gap_max_s:
            self._on_boundary(end, unreliable)
        elif d > self.p.gap_max_s:
            # 점등을 놓쳤거나 신호가 멈춤 → 현재 프레임 무효, 다음 정상 경계까지 재동기화
            if self.stage == STAGE_CAPTURE:
                self._invalidate(f"off_too_long_{d:.2f}")
                self._finish_frame(start)
            self.stage = STAGE_SYNC if not self.done else self.stage
            self._clear_cur()
        else:
            self._invalidate(f"off_ambiguous_{d:.2f}")

    def _on_boundary(self, t_boundary: float, unreliable: bool) -> None:
        if self.stage == STAGE_CAPTURE:
            self._finish_frame(t_boundary)
        if self.done:
            return
        self.stage = STAGE_CAPTURE
        self._clear_cur()
        if unreliable:
            self._cur_invalid = "sample_gap_at_frame_start"

    def _clear_cur(self) -> None:
        self._cur, self._cur_times, self._cur_start, self._cur_invalid = [], [], None, ""

    def _finish_frame(self, t_end: float) -> None:
        seq = "".join(self._cur)
        reason = self._cur_invalid
        if not reason:
            if len(seq) != self.p.blinks_per_frame:
                reason = f"blink_count_{len(seq)}"
            else:
                span = self._cur_times[-1][1] - self._cur_times[0][0]
                if abs(span - self.p.frame_span_s) > self.p.frame_span_tol_s:
                    reason = f"frame_span_{span:.2f}"
                elif not counts_valid(seq, self.p):
                    reason = "count_rule"
        start = self._cur_start if self._cur_start is not None else t_end
        self.frames.append(FrameRecord(start, t_end, seq, not reason, reason, list(self._cur_times)))
        self._vote()

    def _vote(self) -> None:
        votes = Counter(f.sequence for f in self.frames if f.valid)
        if votes:
            ranked = votes.most_common()
            top_seq, top = ranked[0]
            second = ranked[1][1] if len(ranked) > 1 else 0
            if top >= self.p.k_agree and top - second >= self.p.min_margin:
                self.stage, self.result = STAGE_CONFIRMED, top_seq
                return
        if len(self.frames) >= self.p.max_frames:
            self.stage, self.fail_reason = STAGE_FAILED, "max_frames_without_consensus"
