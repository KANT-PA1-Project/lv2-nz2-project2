import pytest
from arm_practice.native_rgbd.settling_window import SettlingWindow

Z = [0.] * 5


def test_230_good_ticks_do_not_certify_quarter_second():
    w = SettlingWindow(dwell_s=.25)
    for i in range(230):
        result = w.update(i, Z, Z)
    assert not result['settled_interval']
    for i in range(230, 240):
        result = w.update(i, Z, Z)
    assert result['settled_interval']
    assert not result['motion_authorized']


@pytest.mark.parametrize('bad', [.02, .033795, float('nan'), float('inf'), True])
def test_zero_fd_cannot_hide_native_speed_and_resets_dwell(bad):
    w = SettlingWindow()
    for i in range(479):
        w.update(i, Z, Z)
    assert w.update(479, [bad, 0., 0., 0., 0.], Z)['continuous_steps'] == 0
    assert not w.update(480, Z, Z)['settled_interval']


@pytest.mark.parametrize('step', [10, 12, -1, True])
def test_duplicate_missing_or_invalid_tick_resets(step):
    w = SettlingWindow()
    w.update(10, Z, Z)
    assert w.update(step, Z, Z)['continuous_steps'] == 0


def test_fd_also_required():
    w = SettlingWindow()
    assert w.update(0, Z, [.03, 0., 0., 0., 0.])['continuous_steps'] == 0
