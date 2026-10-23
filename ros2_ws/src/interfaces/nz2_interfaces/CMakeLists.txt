# =============================================================
# 패키지 : nz2_interfaces
# 파일   : CMakeLists.txt
# 레이어 : interface
# 역할   : msg/srv/action 파일을 등록해 Python·C++ 타입을 생성한다
#
# [해야 할 일 / TODO]
#   - 새 msg/srv/action 파일을 만들면 아래 rosidl_generate_interfaces 목록에 추가
#   - 표준 타입(geometry_msgs/Twist, sensor_msgs/Image, vision_msgs 등)으로 되면 새로 만들지 않는다
# [ROS2 메모] rosidl_generate_interfaces = 정의 파일 → 코드 자동 생성
# =============================================================
cmake_minimum_required(VERSION 3.8)
project(nz2_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Puck.msg"
  "msg/PuckArray.msg"
  "msg/TrafficLight.msg"
  "msg/SwerveModuleState.msg"
  "msg/SwerveCommand.msg"
  "srv/SetRobotMode.srv"
  "action/ExecuteMission.action"
  "action/PickPlace.action"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
