# =============================================================
# 패키지 : nz2_interfaces
# 파일   : msg/TrafficLight.msg
# 레이어 : interface
# 역할   : 신호등(경광등 3단: 위부터 빨/노/초) 인식 결과
#
# [토픽]     /perception/traffic_light
# [보내는 쪽] nz2_perception (traffic_light_filter) / [받는 쪽] signal_recorder, perception_master, nz2_planning
# [미션 v5] 12회 점멸 순서를 기록해야 해서 "꺼짐(OFF)"과 "안 보임(UNKNOWN)"을 구분한다
# =============================================================

std_msgs/Header header

# 상태 상수
uint8 STATE_UNKNOWN=0   # 안 보임 / 판정 불가 (꺼짐이 아님! 점멸 기록에서 꺼짐으로 세면 안 됨)
uint8 STATE_RED=1
uint8 STATE_YELLOW=2
uint8 STATE_GREEN=3
uint8 STATE_OFF=4       # 경광등은 보이는데 세 칸 모두 꺼짐

# 확정 상태 (같은 raw_state 가 confirm_frames 프레임 연속일 때 바뀜) — 출발/정지 판단은 이 값
uint8 state

# 신뢰도 0.0 ~ 1.0
float32 confidence

# 같은 raw_state 가 연속된 프레임 수
uint8 confirmed_frames

# 이번 프레임에 경광등이 보였는지 (박스 + 같은 시각 영상)
bool visible

# 이번 프레임 판정 (확정 전, STATE_* 값). 점멸 기록기(signal_recorder)는 이 값을 씀
uint8 raw_state
