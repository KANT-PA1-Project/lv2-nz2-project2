cmake_minimum_required(VERSION 3.10)
project(nz2_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(nz2_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# ---------------- YOLO 엔진 (2026-10-24 추가) ----------------
#   TensorRT (Jetson 본선): NvInfer.h + libnvinfer + CUDA 를 찾으면 자동으로 켜짐 (-DNZ2_USE_TENSORRT=OFF 로 끔)
#   OpenCV DNN (.onnx, 노트북 CPU): 항상 빌드
option(NZ2_USE_TENSORRT "TensorRT 백엔드 빌드 (찾으면 자동)" ON)
set(NZ2_HAVE_TRT OFF)
if(NZ2_USE_TENSORRT)
  if(NOT DEFINED CUDAToolkit_ROOT AND EXISTS /usr/local/cuda)
    set(CUDAToolkit_ROOT /usr/local/cuda)   # Jetson(JetPack) 기본 위치
  endif()
  find_package(CUDAToolkit QUIET)
  find_path(TENSORRT_INCLUDE_DIR NvInfer.h
    HINTS ENV TENSORRT_ROOT /usr/local/TensorRT
    PATHS /usr/include/aarch64-linux-gnu /usr/include/x86_64-linux-gnu /usr/include
    PATH_SUFFIXES include)
  find_library(TENSORRT_NVINFER_LIB nvinfer
    HINTS ENV TENSORRT_ROOT /usr/local/TensorRT
    PATHS /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu
    PATH_SUFFIXES lib lib64)
  if(CUDAToolkit_FOUND AND TENSORRT_INCLUDE_DIR AND TENSORRT_NVINFER_LIB)
    set(NZ2_HAVE_TRT ON)
    message(STATUS "perception: TensorRT 백엔드 사용 (${TENSORRT_NVINFER_LIB})")
  else()
    message(WARNING "perception: TensorRT/CUDA 를 못 찾음 → OpenCV DNN(.onnx) 백엔드만 빌드합니다")
  endif()
endif()
set(YOLO_SRCS src/yolo_engine.cpp src/yolo_engine_opencv.cpp)
if(NZ2_HAVE_TRT)
  list(APPEND YOLO_SRCS src/yolo_engine_tensorrt.cpp)
endif()
add_library(yolo_engine STATIC ${YOLO_SRCS})
target_include_directories(yolo_engine PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(yolo_engine PUBLIC ${OpenCV_LIBS})
set_target_properties(yolo_engine PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(NZ2_HAVE_TRT)
  target_compile_definitions(yolo_engine PUBLIC NZ2_WITH_TENSORRT=1)
  target_include_directories(yolo_engine PRIVATE ${TENSORRT_INCLUDE_DIR})
  target_link_libraries(yolo_engine PUBLIC ${TENSORRT_NVINFER_LIB} CUDA::cudart)
endif()

# yolo_detector
add_executable(yolo_detector src/yolo_detector_node.cpp)
ament_target_dependencies(yolo_detector rclcpp sensor_msgs std_msgs vision_msgs cv_bridge OpenCV)
target_link_libraries(yolo_detector yolo_engine)

# traffic_light_filter
add_executable(traffic_light_filter src/traffic_light_filter_node.cpp)
ament_target_dependencies(traffic_light_filter rclcpp sensor_msgs vision_msgs nz2_interfaces cv_bridge OpenCV)

# barcode_reader — [2026-10-28] 미션 바코드가 Code 128(1D) → ZBar 필요 (sudo apt install libzbar-dev)
add_executable(barcode_reader src/barcode_reader_node.cpp)
ament_target_dependencies(barcode_reader rclcpp sensor_msgs nz2_interfaces cv_bridge OpenCV)
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(ZBAR QUIET zbar)
endif()
if(ZBAR_FOUND)
  target_compile_definitions(barcode_reader PRIVATE NZ2_WITH_ZBAR=1)
  target_include_directories(barcode_reader PRIVATE ${ZBAR_INCLUDE_DIRS})
  target_link_libraries(barcode_reader ${ZBAR_LINK_LIBRARIES})   # 전체 경로로 링크
  message(STATUS "perception: barcode_reader ← ZBar ${ZBAR_VERSION} (Code 128 지원)")
else()
  message(WARNING "perception: libzbar 없음 → barcode_reader 가 QR 만 읽음 (Code 128 불가). sudo apt install libzbar-dev")
endif()

# tag_reader — [2026-10-28] AprilTag tag36h11 (퍽 16 mm / 장소 48 mm) → /nz2/tags. OpenCV ArUco 사용 (Isaac ROS 불필요)
add_executable(tag_reader src/tag_reader_node.cpp src/tag_detector.cpp)
target_include_directories(tag_reader PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(tag_reader rclcpp sensor_msgs geometry_msgs vision_msgs cv_bridge
  tf2 tf2_ros tf2_geometry_msgs OpenCV)

# perception_master
add_executable(perception_master src/perception_master.cpp)
ament_target_dependencies(perception_master
  rclcpp sensor_msgs std_msgs geometry_msgs vision_msgs nz2_interfaces)   # [2026-10-30] vision_msgs: 검출 fps

set(PERCEPTION_TARGETS
  yolo_detector traffic_light_filter
  barcode_reader tag_reader perception_master)

install(TARGETS ${PERCEPTION_TARGETS} DESTINATION lib/${PROJECT_NAME})
# Python YOLO 노드(PC 테스트용, ultralytics .pt). 실행: ros2 run nz2_perception yolo_detector.py
install(PROGRAMS src/yolo_detector.py src/puck_registry.py
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
