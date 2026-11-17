#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/lyrical/setup.bash
source "$ROOT/reports/local/ros_install/setup.bash"
# Dedicated local domain for this simulated robot; do not send to hardware.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-78}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
case "${1:-demo}" in
  demo) exec ros2 launch r12_arm_moveit_config demo.launch.py ;;
  sim) exec ros2 launch r12_arm_moveit_config sim.launch.py ;;
  cumotion) exec ros2 launch isaac_ros_cumotion isaac_ros_cumotion.launch.py \
    cumotion_action_server.urdf_file_path:="$ROOT/ros2/r12_arm_moveit_config/config/r12.urdf" \
    cumotion_action_server.xrdf_file_path:="$ROOT/ros2/r12_arm_moveit_config/config/r12.xrdf" \
    cumotion_action_server.read_esdf_world:=false ;;
  *) echo 'Usage: bash scripts/r12_ros.sh [demo|sim|cumotion]' >&2; exit 2 ;;
esac
