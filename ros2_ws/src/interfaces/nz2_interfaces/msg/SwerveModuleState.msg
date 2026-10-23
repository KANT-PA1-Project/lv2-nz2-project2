# =============================================================
# 패키지 : nz2_interfaces
# 파일   : msg/SwerveModuleState.msg
# 레이어 : interface
# 역할   : 스월브 모듈 1개의 목표(또는 현재) 상태
#
# [쓰는 곳] SwerveCommand.msg 안의 배열 원소
# =============================================================

# 조향 각도 [rad]. 차체 정면 = 0, 반시계 방향 +
float32 steer_angle_rad

# 바퀴 구동 속도 [m/s]. 앞으로 굴러가면 +
float32 drive_velocity_mps
