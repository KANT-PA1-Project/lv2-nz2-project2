# =============================================================
# 패키지 : nz2_interfaces
# 파일   : msg/PuckArray.msg
# 레이어 : interface
# 역할   : 한 번에 확정된 퍽 목록
#
# [토픽]     /perception/puck_tracks
# [보내는 쪽] nz2_perception (puck_registry) / [받는 쪽] nz2_planning, perception_master
# =============================================================

# 인식 시각과 기준 좌표계 (안의 Puck.header 와 같음)
std_msgs/Header header

# 확정된 퍽들 (없으면 빈 배열)
Puck[] pucks
