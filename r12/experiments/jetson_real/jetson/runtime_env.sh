#!/usr/bin/env bash
# Source this file in the Jetson shell before running individual runtime nodes.
source /opt/ros/lyrical/setup.bash
source /home/nz2/nz2_camera_ws/install/setup.bash
source /home/nz2/p2_runtime_ws/install/setup.bash
export PYTHONPATH="/home/nz2/p2_runtime_ws/runtime/python${PYTHONPATH:+:${PYTHONPATH}}"
