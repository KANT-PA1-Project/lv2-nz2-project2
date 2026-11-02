# =============================================================
# 패키지 : nz2_interfaces
# 파일   : msg/PerceptionState.msg
# 레이어 : interface
# 역할   : 인지 결과 한눈에 보기 (미션 FSM·플래너는 이것 + /perception/summary 만 보면 됨)
#
# [토픽]     /perception/state  (10 Hz)
# [보내는 쪽] nz2_perception (perception_master) / [받는 쪽] nz2_planning (mission_manager_node)
# [참고] 신호 점멸 기록·주사위 판정·퍽 등록표는 /perception/summary (std_msgs/String JSON) 에 있음
# =============================================================

std_msgs/Header header

# 인지 모드 상수 — SetPerceptionMode 로 바꿈 (카메라 = 그리퍼 D435 1대)
uint8 MODE_IDLE=0
uint8 MODE_NAVIGATE=1   # 주행: 신호등·바코드
uint8 MODE_APPROACH=2   # 상자 접근: 장소 태그
uint8 MODE_PICK=3       # 파지: 퍽 위치·ID
uint8 MODE_PLACE=4      # 놓기: 주사위 판정
uint8 mode

# 최신 신호등 상태
TrafficLight traffic_light

# 최신 확정 퍽 목록
Puck[] pucks

# 최신 바코드 목록
Barcode[] barcodes

# 그리퍼 카메라가 0.5 s 안에 영상(camera_info)을 보냈는지 (꺼둔 모드에서는 true)
bool camera_ok

# 위치 추정이 믿을 만한지 [미정: 카토그래퍼 tf 기준으로 판정 예정, 지금은 false]
bool localization_ok

# 카메라 프레임 수 / YOLO 처리 수 (초당, 근사)
float32 camera_fps
float32 yolo_fps
