cmake_minimum_required(VERSION 3.10)
project(nz2_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 인지 범위 메시지/서비스 → 빌드 시 C++/Python 코드 생성
# [2026-11-02] 팀 레포 nz2_interfaces(#21)와 같은 정의로 맞춤 (ShelfPose 삭제). 팀 레포에는 스월브·액션 정의가 더 있음
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Puck.msg"
  "msg/PuckArray.msg"
  "msg/TrafficLight.msg"
  "msg/Barcode.msg"
  "msg/BarcodeArray.msg"
  "msg/PerceptionState.msg"
  "srv/SetPerceptionMode.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
