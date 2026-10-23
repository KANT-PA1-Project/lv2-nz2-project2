# =============================================================
# 패키지 : nz2_interfaces
# 파일   : msg/TrafficLight.msg
# 레이어 : interface
# 역할   : 신호등 인식 결과 (출발/정지 판단에 쓴다)
#
# [토픽]     /nz2/traffic_light
# [보내는 쪽] nz2_perception (traffic_light_node) / [받는 쪽] nz2_planning
# [가정] 세부 대회 규칙 미공개. 색과 동작의 대응은 규칙 확정 후 갱신
# =============================================================

# 신호 색: "red" / "yellow" / "green" (모르면 "unknown")
string state

# 인식 신뢰도 0.0 ~ 1.0
float32 confidence
