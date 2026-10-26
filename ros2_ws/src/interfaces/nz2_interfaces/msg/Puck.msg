# =============================================================
# 패키지 : nz2_interfaces
# 파일   : msg/Puck.msg
# 레이어 : interface
# 역할   : 인식된 퍽 1개의 정보
#
# [보내는 쪽] nz2_perception (puck_detector_node, puck_pose_estimator_node)
# [받는 쪽]   nz2_planning (mission_manager_node)
# [ROS2 메모] msg = 토픽에 실리는 데이터 구조
# =============================================================

# 모양 상수
uint8 SHAPE_CYLINDER=0
uint8 SHAPE_BOX=1

# 자세 상수
uint8 ORIENTATION_STANDING=0
uint8 ORIENTATION_LYING=1

# 퍽 색: "red" / "green"  (2색 확정 — 파랑 제외, YOLO 9클래스와 일치)
string color

# 퍽 위치·자세 (header.frame_id에 기준 좌표계를 적는다)
geometry_msgs/PoseStamped pose

# 모양: SHAPE_CYLINDER(원통, 반지름 30mm·높이 60mm) / SHAPE_BOX(직육면체 30×30×60mm)
uint8 shape

# 자세: ORIENTATION_STANDING(세움) / ORIENTATION_LYING(누움)
uint8 orientation

# 인식 신뢰도 0.0 ~ 1.0
float32 confidence
