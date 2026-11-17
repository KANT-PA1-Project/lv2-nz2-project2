cmake_minimum_required(VERSION 3.16)
project(nz2_swerve_nav CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(swerve_controller_node src/swerve_controller_node.cpp)
target_include_directories(swerve_controller_node PRIVATE include)
target_link_libraries(swerve_controller_node
  rclcpp::rclcpp tf2_ros::tf2_ros
  ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${sensor_msgs_TARGETS} ${std_msgs_TARGETS})

add_executable(scan_merger_node src/scan_merger_node.cpp)
target_link_libraries(scan_merger_node
  rclcpp::rclcpp tf2_ros::tf2_ros
  ${geometry_msgs_TARGETS} ${sensor_msgs_TARGETS})

install(TARGETS swerve_controller_node scan_merger_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/swerve_teleop.py scripts/cmd_frame_adapter.py scripts/r12_cmd_to_gz.py scripts/shift_map_origin.py scripts/save_map.sh scripts/odom_check.py scripts/nav_goal_test.py scripts/bag_to_csv.py scripts/transform_map.py scripts/save_map_carto.sh scripts/real_wheel_check.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch behavior_trees DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(test_swerve_kinematics test/test_swerve_kinematics.cpp)
  target_include_directories(test_swerve_kinematics PRIVATE include)
  add_test(NAME test_swerve_kinematics COMMAND test_swerve_kinematics)
endif()

ament_package()
