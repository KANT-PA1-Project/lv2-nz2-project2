# =============================================================
# 패키지 : nz2_interfaces
# 파일   : msg/Puck.msg
# 레이어 : interface
# 역할   : 인식·확정된 퍽 1개의 정보 (모양·색·자세·3D 위치·태그 ID)
#
# [보내는 쪽] nz2_perception (puck_registry → /perception/puck_tracks, PuckArray 안)
# [받는 쪽]   nz2_planning (mission_manager_node), perception_master(요약)
# [ROS2 메모] msg = 토픽에 실리는 데이터 구조. 상수(대문자)는 값 비교용, 필드(소문자)가 실제 데이터
# =============================================================

# 기준 좌표계·시각. frame_id = map (SLAM 켜짐) / odom (없을 때) / 카메라 광학 프레임 (tf 없을 때)
std_msgs/Header header

# 색 상수 — 대회 퍽 2색 (YOLO 9클래스와 일치)
uint8 COLOR_UNKNOWN=0
uint8 COLOR_RED=1
uint8 COLOR_GREEN=2
uint8 color

# 모양 상수 — 원통(반지름 30mm·높이 60mm) / 직육면체(30×30×60mm)
uint8 SHAPE_UNKNOWN=0
uint8 SHAPE_CYLINDER=1
uint8 SHAPE_BOX=2
uint8 shape

# 자세 상수 — YOLO 클래스 _up/_down + depth 로 잰 높이로 보정
uint8 POSTURE_UNKNOWN=0
uint8 POSTURE_UP=1      # 세움
uint8 POSTURE_DOWN=2    # 누움
uint8 posture

# 퍽 중심 3D 위치 (m), header.frame_id 기준
geometry_msgs/Point position

# 접근(파지) 방향 (rad). 아직 계산 안 하면 NaN
float32 yaw

# YOLO 검출 신뢰도 0.0 ~ 1.0
float32 confidence

# 프레임 사이 추적 번호 (같은 퍽이면 같은 값). 추적 안 하면 -1
int32 track_id

# 퍽 AprilTag ID (0~11). 아직 확정 못 했으면 -1 (3번 이상·70% 이상 같은 ID 로 읽혀야 확정)
int32 tag_id -1
