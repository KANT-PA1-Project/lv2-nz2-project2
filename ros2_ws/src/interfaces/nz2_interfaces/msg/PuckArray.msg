# =============================================================
# 패키지 : nz2_interfaces
# 파일   : msg/PuckArray.msg
# 레이어 : interface
# 역할   : 한 프레임에서 인식된 퍽 목록
#
# [토픽]     /nz2/pucks
# [보내는 쪽] nz2_perception / [받는 쪽] nz2_planning
# =============================================================

# 인식 시각과 기준 좌표계
std_msgs/Header header

# 인식된 퍽들 (없으면 빈 배열)
Puck[] pucks
