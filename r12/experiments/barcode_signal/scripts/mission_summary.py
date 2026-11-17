"""Pre-survey mission requirements; no puck allocation or robot commands."""
from collections import Counter, deque
import math
import time
import os
import json
from pathlib import Path
import sys

R12_SRC = Path(os.environ.get("R12_SRC", "~/git/lv2-nz2-project2/r12/src")).expanduser()
if not (R12_SRC / "arm_practice/mission.py").is_file():
    raise ImportError(f"R12_SRC must contain arm_practice/mission.py: {R12_SRC}")
sys.path.insert(0, str(R12_SRC))
from arm_practice.mission import SignalRecorder

BARCODE_WINDOW_S = 5.0
BARCODE_MIN_MATCHES = 3

BARCODE_CONFIG = Path(__file__).resolve().parents[1] / "configs/barcodes.json"
_rows = json.loads(BARCODE_CONFIG.read_text(encoding="utf-8"))["barcodes"]
BARCODE_RULES = {row["barcode"]: dict(row) for row in _rows}
if len(_rows) != 4 or set(BARCODE_RULES) != {"P2-01", "P2-02", "P2-03", "P2-04"}:
    raise ValueError("Barcode config must contain P2-01..04 exactly once")
for _row in BARCODE_RULES.values():
    if set(_row) != {"barcode", "sortOrder", "sortLabel"}:
        raise ValueError("Barcode fields must be barcode/sortOrder/sortLabel")
    if _row["sortOrder"] not in ("ASC", "DESC") or _row["sortLabel"] != {
            "ASC": "오름차순", "DESC": "내림차순"}[_row["sortOrder"]]:
        raise ValueError("Invalid barcode sortOrder/sortLabel")
# Compatibility view; the JSON above is the source of the sort definitions.
BARCODES = {code: row["sortOrder"] == "DESC" for code, row in BARCODE_RULES.items()}


def barcode_metadata(code):
    if code is None:
        return dict(barcodes=[], sortOrder=None, sortLabel=None)
    row = dict(BARCODE_RULES[code])
    return dict(barcodes=[row], sortOrder=row["sortOrder"], sortLabel=row["sortLabel"])



def requirements(sequence, barcode):
    """Mission PDF v5 p3: stable R,Y,G ties, count order, no inventory needed."""
    sequence = list(sequence)
    if len(sequence) != 12 or any(c not in ("R", "Y", "G") for c in sequence):
        raise ValueError("Exactly 12 known R/Y/G flashes required")
    counts = {c: sequence.count(c) for c in "RYG"}
    if any(not 1 <= n <= 6 for n in counts.values()):
        raise ValueError("Each signal count must be 1..6")
    if barcode not in BARCODES:
        raise ValueError("Unregistered barcode")
    descending = BARCODES[barcode]
    groups = sorted("RYG", key=lambda c: -counts[c] if descending else counts[c])
    constraints = {
        "R": {"color": "R", "shape": "any"},
        "Y": {"color": "any", "shape": "one_common_shape_pending_inventory"},
        "G": {"color": "G", "shape": "any"},
    }
    return dict(barcode=barcode, **barcode_metadata(barcode),
                direction="descending" if descending else "ascending",
                sequence=sequence, counts=counts, tie_order=list("RYG"),
                locations=[dict(location=f"L{i+1}", group=c, count=counts[c],
                                constraint=constraints[c]) for i, c in enumerate(groups)],
                puck_allocation="pending_inventory")


class MissionSession:
    """Explicit arm before first flash; every error remains latched until reset.

    Feed the same seconds clock for arm and observe. Barcode reads are decoded
    observations, never command-line assumed codes. Three matching decode frames
    within five seconds latch a code. Registered conflicts defer confirmation;
    missing/unregistered reads do not erase evidence or fail the session.
    """
    def __init__(self):
        self.recorder = SignalRecorder()
        self.barcode = None
        self.candidate = None
        self.matches = 0
        self.barcode_votes = deque()
        self.last_barcode_time = None
        self.armed = False
        self.error = None

    def fail(self, reason):
        if self.error is None:
            self.error = str(reason)

    def observe_barcodes(self, reads, time_s=None):
        if self.error:
            return
        now = time.monotonic() if time_s is None else float(time_s)
        if not math.isfinite(now):
            raise ValueError("Non-finite barcode timestamp")
        # One camera frame contributes at most one vote per registered code.
        if self.last_barcode_time is not None and now <= self.last_barcode_time:
            return
        self.last_barcode_time = now
        while self.barcode_votes and now-self.barcode_votes[0][0] > BARCODE_WINDOW_S:
            self.barcode_votes.popleft()
        texts = {r["text"] for r in reads if r["text"] in BARCODES}
        for code in sorted(texts):
            self.barcode_votes.append((now, code))
        counts = Counter(code for _, code in self.barcode_votes)
        if self.barcode is not None and any(code != self.barcode for code in texts):
            self.barcode = None
        if len(counts) > 1:
            # Do not choose a majority when two allowed codes are in the window.
            self.barcode = None
            self.candidate, self.matches = None, 0
        elif counts:
            self.candidate, self.matches = next(iter(counts.items()))
            if self.matches >= BARCODE_MIN_MATCHES:
                self.barcode = self.candidate
        else:
            self.candidate, self.matches = None, 0
        # A confirmed code survives missing reads after its evidence expires.

    def arm(self, time_s, color, *, confirmed_off, fixed_settings):
        if self.error or self.armed:
            return False
        if not confirmed_off or not fixed_settings or color != "OFF":
            return False
        try:
            self.recorder.observe(time_s, color)
        except ValueError as exc:
            self.fail(exc)
            return False
        self.armed = True
        return True

    def observe(self, time_s, color):
        if self.armed and not self.error and not self.recorder.complete:
            try:
                self.recorder.observe(time_s, color)
            except ValueError as exc:
                self.fail(exc)
        return self.snapshot()

    def snapshot(self):
        seq = list(self.recorder.sequence)
        ready = self.armed and self.recorder.complete and self.barcode is not None and not self.error
        result = dict(status="error" if self.error else "ready" if ready else
                      "waiting_barcode" if self.recorder.complete else
                      "recording" if self.armed else "waiting_arm",
                      error=self.error, barcode=self.barcode,
                      direction=None if self.barcode is None else
                      "descending" if BARCODES[self.barcode] else "ascending",
                      sequence=seq, counts={c: Counter(seq)[c] for c in "RYG"},
                      signal_complete=self.recorder.complete,
                      locations=[], puck_allocation="pending_inventory",
                      scope="perception_requirements_only_not_puck_assignment_or_motion")
        result.update(barcode_metadata(self.barcode))
        if ready:
            result.update(requirements(seq, self.barcode))
        return result
