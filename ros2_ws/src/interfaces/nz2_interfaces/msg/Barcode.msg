# =============================================================
# 패키지 : nz2_interfaces
# 파일   : msg/Barcode.msg
# 레이어 : interface
# 역할   : 읽은 바코드 1개 (미션 v5: Code 128 "P2-01" ~ "P2-04")
#
# [보내는 쪽] nz2_perception (barcode_reader, ZBar) — BarcodeArray 안에 담김
# =============================================================

std_msgs/Header header

# 디코드된 문자열 (예: "P2-01")
string data

# 심볼 종류 (예: "CODE128", "QRCODE")
string format

# 추정 위치 (m). 안 쓰면 NaN
geometry_msgs/Point position

# 신뢰도 0.0 ~ 1.0. 같은 문자열을 confirm_reads 번 연속 읽으면 1.0 (확정)
float32 confidence
