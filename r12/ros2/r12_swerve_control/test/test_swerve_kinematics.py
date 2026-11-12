"""swerve_kinematics 순수 테스트 — ROS 없이 수식만. ±90° 제약 IK + fold/역전 검증."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'r12_swerve_control'))
from swerve_kinematics import HALF_PI, inverse_kinematics, optimize_angle

# FL FR RL RR (±0.124 m)
POS = [(0.124, 0.124), (0.124, -0.124), (-0.124, 0.124), (-0.124, -0.124)]


def test_forward_straight():
    out = inverse_kinematics(0.2, 0.0, 0.0, POS, 0.0)
    assert all(abs(a) < 1e-9 for a, _ in out)
    assert all(abs(s - 0.2) < 1e-9 for _, s in out)


def test_strafe_ninety_deg():
    out = inverse_kinematics(0.0, 0.2, 0.0, POS, 0.0)
    assert all(abs(a - HALF_PI) < 1e-9 for a, _ in out)
    assert all(abs(s - 0.2) < 1e-9 for _, s in out)


def test_max_speed_scaling():
    out = inverse_kinematics(1.0, 0.0, 0.0, POS, 0.3)
    assert all(abs(s - 0.3) < 1e-9 for _, s in out)   # 1.0 요청이 0.3 으로 축소


def test_optimize_forward():
    assert optimize_angle(0.0, 0.0) == (0.0, 1.0)


def test_optimize_strafe():
    a, sign = optimize_angle(HALF_PI, 0.0)
    assert abs(a - HALF_PI) < 1e-9 and sign == 1.0


def test_optimize_reverse_is_sign_flip_not_180():
    # 후진 heading(π)은 조향 180° 뒤집기가 아니라 바퀴 반전으로 처리 → 조향 0, 부호 -1.
    a, sign = optimize_angle(math.pi, 0.0)
    assert abs(a) < 1e-9 and sign == -1.0


def test_optimize_hysteresis_holds_at_90():
    # +90°에 있을 때 91.7°(경계+hyst 안) 요청해도 -90°로 뒤집지 않고 +90° 유지.
    a, sign = optimize_angle(1.6, HALF_PI)
    assert abs(a - HALF_PI) < 1e-9 and sign == 1.0


def test_optimize_clips_to_limit():
    a, _ = optimize_angle(1.6, HALF_PI)    # 1.6 > π/2 → π/2 로 클립
    assert a <= HALF_PI + 1e-9
