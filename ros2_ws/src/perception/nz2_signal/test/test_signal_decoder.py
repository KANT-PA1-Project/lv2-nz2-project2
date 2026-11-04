"""신호 판정 코어 테스트: 중간 진입, 불규칙 fps, 드롭, 점등 놓침, 번짐."""
import pytest

from nz2_signal.core.lamp_classifier import LampClassifier
from nz2_signal.core.signal_decoder import (STAGE_CONFIRMED, STAGE_FAILED, SignalDecoder,
                                            SignalParams, count_colors)
from nz2_signal.core.simulator import SimConfig, all_valid_sequences_sample, samples


def run(cfg: SimConfig, params: SignalParams = None):
    clf = LampClassifier()
    dec = SignalDecoder(params)
    for t, vals in samples(cfg):
        dec.feed(t, clf.classify(t, vals))
        if dec.done:
            break
    return dec


def test_q1_basic_confirm():
    dec = run(SimConfig(sequence="RGRRGRGGYGYG"))
    assert dec.stage == STAGE_CONFIRMED
    assert dec.result == "RGRRGRGGYGYG"
    assert count_colors(dec.result) == {"R": 4, "Y": 2, "G": 6}


@pytest.mark.parametrize("offset", [0.0, 3.3, 11.7, 22.9, 24.5, 25.9])
def test_mid_loop_entry(offset):
    """경기 시작 시점이 루프 어디든 동기화 후 확정되어야 한다."""
    dec = run(SimConfig(start_offset_s=offset))
    assert dec.stage == STAGE_CONFIRMED and dec.result == "RGRRGRGGYGYG"


@pytest.mark.parametrize("seq", all_valid_sequences_sample(25, seed=7))
def test_all_count_combinations(seq):
    dec = run(SimConfig(sequence=seq, start_offset_s=7.0))
    assert dec.result == seq


def test_irregular_fps_and_drops():
    dec = run(SimConfig(fps=12.0, fps_jitter=0.5, drop_prob=0.1, seed=3))
    assert dec.stage == STAGE_CONFIRMED and dec.result == "RGRRGRGGYGYG"


def test_missed_blink_invalidates_only_that_frame():
    """점등 1개 누락 → 3초 소등으로 보여도 해당 프레임만 버리고 확정."""
    cfg = SimConfig(start_offset_s=0.5, missed_blinks={(1, 5)}, duration_s=200)
    dec = run(cfg)
    assert dec.stage == STAGE_CONFIRMED and dec.result == "RGRRGRGGYGYG"
    assert any(not f.valid for f in dec.frames)


def test_weak_bloom_ok():
    dec = run(SimConfig(bloom=40.0))
    assert dec.result == "RGRRGRGGYGYG"


def test_heavy_bloom_never_wrong():
    """심한 번짐: 틀린 문자열로 확정하면 안 된다(확정 실패는 허용)."""
    dec = run(SimConfig(bloom=150.0, duration_s=260), SignalParams(max_frames=8))
    assert dec.result in (None, "RGRRGRGGYGYG")
    if dec.result is None:
        assert dec.stage == STAGE_FAILED


def test_too_slow_camera_does_not_confirm_wrong():
    """2fps 이하: 경계 공백으로 무효가 되어야지 오답을 내면 안 된다."""
    dec = run(SimConfig(fps=2.0, duration_s=260))
    assert dec.result in (None, "RGRRGRGGYGYG")


def test_signal_stopped_fails():
    """신호가 꺼져 있으면 확정되지 않는다(SYNC에서 대기)."""
    cfg = SimConfig(sequence="RGRRGRGGYGYG", duration_s=60)
    clf, dec = LampClassifier(), SignalDecoder()
    for t, vals in samples(cfg):
        dec.feed(t, clf.classify(t, [25.0, 25.0, 25.0]))
    assert dec.result is None


def test_frame_log_contains_blink_times():
    dec = run(SimConfig())
    v = [f for f in dec.frames if f.valid][0]
    assert len(v.blink_times) == 12
    assert v.to_dict()["sequence"] == "RGRRGRGGYGYG"
