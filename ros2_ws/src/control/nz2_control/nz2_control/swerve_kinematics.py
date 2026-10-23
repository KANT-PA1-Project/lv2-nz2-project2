# =============================================================
# 패키지 : nz2_control
# 파일   : swerve_kinematics.py
# 레이어 : control
# 역할   : (라이브러리) 스월브 역기구학: 차체 속도(vx, vy, ω) → 4모듈 (조향각, 구동속도)
#
# [입력]  차체 속도 vx, vy [m/s], ω [rad/s]
#         모듈 장착 위치 (config/swerve_params.yaml)
# [출력]  모듈 4개의 (steer_angle_rad, drive_velocity_mps) — FL, FR, RL, RR 순서
# [해야 할 일 / TODO]
#   - 각 모듈 속도 벡터 = (vx - ω·y_i, vy + ω·x_i) 로 계산
#   - 벡터를 각도(atan2)와 크기로 변환
#   - 조향 범위 ±90°: 범위를 넘으면 각도를 180° 돌리고 구동 방향을 반대로 (케이블 꼬임 방지)
#   - 가장 빠른 바퀴가 최대 속도를 넘으면 전체를 같은 비율로 줄이기
# [ROS2 메모] 역기구학 = 원하는 움직임(차체 속도)에서 각 관절(바퀴) 명령을 거꾸로 계산하는 것
# =============================================================


def inverse_kinematics(vx, vy, omega, module_positions, max_speed):
    """차체 속도를 모듈 4개의 (조향각, 구동속도)로 바꾼다.

    module_positions: [(x, y), ...] 차체 중심 기준 모듈 위치 [m], FL·FR·RL·RR 순서
    max_speed: 바퀴 최대 선속도 [m/s]
    반환: [(steer_angle_rad, drive_velocity_mps), ...] 4개
    """
    raise NotImplementedError('TODO(제어): 스월브 역기구학')


def optimize_angle(target_angle, current_angle):
    """조향 범위(±90°) 안으로 각도를 맞추고, 뒤집었으면 구동 부호(-1)를 함께 돌려준다."""
    raise NotImplementedError('TODO(제어): 각도 wrap / 역전 최적화')
