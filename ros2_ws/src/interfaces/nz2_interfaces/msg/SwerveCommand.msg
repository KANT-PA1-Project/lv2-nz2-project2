# =============================================================
# 패키지 : nz2_interfaces
# 파일   : msg/SwerveCommand.msg
# 레이어 : interface
# 역할   : 스월브 4모듈 전체 명령
#
# [보내는 쪽] nz2_control (swerve_controller_node)
# [받는 쪽]   nz2_control (opencr_bridge_node) → OpenCR #1
# =============================================================

# 모듈 순서 고정: [0]=FL(앞왼쪽), [1]=FR(앞오른쪽), [2]=RL(뒤왼쪽), [3]=RR(뒤오른쪽)
SwerveModuleState[4] modules
