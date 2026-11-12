# =============================================================
# 패키지 : nz2_control
# 파일   : swerve_kinematics.py
# 레이어 : control
# 역할   : (라이브러리) 제약 ±90° 스월브 역기구학. 차체 속도 → 4모듈 (조향각, 구동속도).
#
# [입력]  vx, vy [m/s], omega [rad/s], module_positions [(x,y)…] FL FR RL RR, max_speed [m/s]
# [출력]  [(steer_angle_rad, drive_velocity_mps)…] 4개, FL FR RL RR
# [설계]  조향 ±90° 제약(단일회전 Position 모드). 범위를 넘는 heading 은 180° 돌리고 구동 반전
#         (케이블 꼬임 방지·경계 연속성). fold/부호 결정은 현재각이 필요해 optimize_angle 이 맡고,
#         상태(직전각)는 노드가 들고 있다. 순수 함수 — ROS 없이 테스트된다.
# =============================================================
import math

HALF_PI = math.pi / 2


def _wrap(a):
    """(-pi, pi] 로 정규화."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def inverse_kinematics(vx, vy, omega, module_positions, max_speed):
    """차체 속도를 모듈 4개의 (원시 조향각 rad, 구동속도 m/s)로.

    조향각은 fold 전 원시 heading(atan2), 구동속도는 양수 크기. fold·구동 부호는 optimize_angle.
    max_speed>0 이고 가장 빠른 모듈이 넘으면 4모듈을 같은 비율로 축소(방향 유지).
    """
    raw, speeds = [], []
    for (x, y) in module_positions:
        mvx = vx - omega * y
        mvy = vy + omega * x
        speed = math.hypot(mvx, mvy)
        raw.append(math.atan2(mvy, mvx) if speed > 1e-9 else 0.0)
        speeds.append(speed)
    top = max(speeds)
    if max_speed > 0.0 and top > max_speed:
        k = max_speed / top
        speeds = [s * k for s in speeds]
    return list(zip(raw, speeds))


def optimize_angle(target_angle, current_angle, limit=HALF_PI, hysteresis=0.17):
    """조향을 ±limit 안으로 fold 하고, 뒤집었으면 구동 부호(-1)를 돌려준다.

    {target, target±180°} 중 |각| <= limit+hysteresis 인 후보에서 current_angle 에 가장 가까운 쪽
    선택(히스테리시스로 ±90° 경계 노이즈마다 180° 뒤집히는 것 방지). 최종 각은 ±limit 로 클립.
    반환: (steer_angle_rad, drive_sign). drive_sign 을 inverse_kinematics 속도에 곱한다.
    """
    a1 = _wrap(target_angle)
    a2 = _wrap(a1 + math.pi)
    lim = limit + hysteresis
    cands = []
    if abs(a1) <= lim:
        cands.append((a1, 1.0))
    if abs(a2) <= lim:
        cands.append((a2, -1.0))
    if not cands:                                  # 이론상 없음(둘은 180° 간격)
        return max(-limit, min(limit, a1)), 1.0
    a, sign = min(cands, key=lambda c: abs(c[0] - current_angle))
    return max(-limit, min(limit, a)), sign
