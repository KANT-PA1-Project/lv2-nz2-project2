# =============================================================
# 패키지 : nz2_interfaces
# 파일   : msg/BarcodeArray.msg
# 레이어 : interface
# 역할   : 한 프레임에서 읽은 바코드 목록
#
# [토픽]     /perception/barcodes
# [보내는 쪽] nz2_perception (barcode_reader) / [받는 쪽] perception_master, nz2_planning
# =============================================================

std_msgs/Header header

# 읽은 바코드들 (없으면 빈 배열)
Barcode[] codes
