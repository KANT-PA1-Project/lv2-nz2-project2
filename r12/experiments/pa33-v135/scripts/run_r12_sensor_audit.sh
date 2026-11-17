#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
export ROS_DOMAIN_ID="${R12_ROS_DOMAIN_ID:-94}"
mkdir -p reports/local/sensors
rm -f reports/local/sensors/completion.json
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
set +u
source /opt/ros/lyrical/setup.bash
source /home/seongmin/ros2_ws/src/turtlebot-swerve-cuvslam/scripts/robot_localization_env.sh
source /home/seongmin/ros2_ws/src/turtlebot-swerve-cuvslam/scripts/nvblox_env.sh
set -u
pids=()
cleanup(){ for p in "${pids[@]}";do kill -TERM -- "-$p" 2>/dev/null || true;done; for p in "${pids[@]}";do wait "$p" 2>/dev/null || true;done; }
trap cleanup EXIT
: > reports/local/sensors/bridge.log
setsid ros2 launch configs/sensors/cuvslam.launch.py > reports/local/sensors/cuvslam.log 2>&1 & pids+=("$!")
setsid ros2 launch configs/sensors/mapping.launch.py > reports/local/sensors/mapping.log 2>&1 & pids+=("$!")
setsid .venv-r12/bin/python scripts/r12_sensor_ros.py > reports/local/sensors/bridge.log 2>&1 & pids+=("$!")
for i in $(seq 1 30);do if rg -q SENSOR_ROS_READY reports/local/sensors/bridge.log;then break;fi;sleep 1;done
rg -q SENSOR_ROS_READY reports/local/sensors/bridge.log
# The renderer/physics process has no ROS imports; clear sourced ROS library overrides.
env -u ROS_DISTRO -u AMENT_PREFIX_PATH -u PYTHONPATH -u LD_LIBRARY_PATH OMNI_KIT_ACCEPT_EULA=YES bash scripts/r12_runtime.sh python scripts/r12_sensor_sim.py --frames "${R12_SENSOR_FRAMES:-100}" > reports/local/sensors/sim.log 2>&1

# Kit can exit with code zero after a Python script error; require the explicit completion marker.
rg -q R12_SENSOR_SIM_COMPLETE reports/local/sensors/sim.log
test -s reports/local/sensors/completion.json
