#!/usr/bin/env bash
set -e
camera_preview_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/lyrical/setup.bash
camera_plugin_prefix="${camera_preview_dir}/runtime_deps/amd64/opt/ros/lyrical"
export AMENT_PREFIX_PATH="${camera_plugin_prefix}:${AMENT_PREFIX_PATH}"
export LD_LIBRARY_PATH="${camera_plugin_prefix}/lib:${LD_LIBRARY_PATH}"
export ROS_DOMAIN_ID=78
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
exec rviz2 -d "${camera_preview_dir}/dual_camera.rviz"
