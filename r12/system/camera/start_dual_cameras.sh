#!/usr/bin/env bash
source /home/nz2/nz2_camera_ws/install/setup.bash
export AMENT_PREFIX_PATH=/home/nz2/p2_runtime_ws/deps/arm64/opt/ros/lyrical:${AMENT_PREFIX_PATH}
export LD_LIBRARY_PATH=/home/nz2/p2_runtime_ws/deps/arm64/opt/ros/lyrical/lib:${LD_LIBRARY_PATH}
export ROS_DOMAIN_ID=78
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
exec ros2 launch /home/nz2/nz2/cameras.launch.py profile:=640x480x30 depth_profile:=640x480x30
