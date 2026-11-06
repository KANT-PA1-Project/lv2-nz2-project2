#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
export R12_SENSOR_RUN="${R12_SENSOR_RUN:-mission_$(date +%Y%m%d_%H%M%S)}"
export R12_USE_LATEST_PPO="${R12_USE_LATEST_PPO:-1}"
export R12_CRUISE_SPEED="${R12_CRUISE_SPEED:-.899795667}"
export R12_GUI="${R12_GUI:-1}"
export R12_MISSION="${R12_MISSION:-1}"
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_DOMAIN_ID="${R12_ROS_DOMAIN_ID:-94}"
report_dir="reports/local/${R12_SENSOR_RUN:-sensors}"
mkdir -p "$report_dir"
if test -e "$report_dir/STOP";then echo "STOP exists in $report_dir; choose a new R12_SENSOR_RUN" >&2;exit 1;fi
rm -f "$report_dir"/completion.json
.venv-r12/bin/python scripts/r12_run_manifest.py "$report_dir"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
set +u
source /opt/ros/lyrical/setup.bash
source /home/seongmin/ros2_ws/src/turtlebot-swerve-cuvslam/scripts/robot_localization_env.sh
source /home/seongmin/ros2_ws/src/turtlebot-swerve-cuvslam/scripts/nvblox_env.sh
set -u
pids=()
cleanup(){ for p in "${pids[@]}";do kill -TERM -- "-$p" 2>/dev/null || true;done; for p in "${pids[@]}";do wait "$p" 2>/dev/null || true;done; }
trap cleanup EXIT
: > "$report_dir"/bridge.log
setsid ros2 launch configs/sensors/cuvslam.launch.py > "$report_dir"/cuvslam.log 2>&1 & pids+=("$!")
setsid ros2 launch configs/sensors/mapping.launch.py > "$report_dir"/mapping.log 2>&1 & pids+=("$!")
if [[ "${R12_RVIZ:-1}" == 1 ]]; then
  setsid .venv-r12/bin/python scripts/r12_rviz_map.py > "$report_dir"/rviz_map.log 2>&1 & pids+=("$!")
  setsid rviz2 -d configs/sensors/r12_mission.rviz --ros-args -p use_sim_time:=true > "$report_dir"/rviz.log 2>&1 & pids+=("$!")
fi
setsid .venv-r12/bin/python scripts/r12_sensor_ros.py > "$report_dir"/bridge.log 2>&1 & pids+=("$!")
for i in $(seq 1 30);do if rg -q SENSOR_ROS_READY "$report_dir"/bridge.log;then break;fi;sleep 1;done
rg -q SENSOR_ROS_READY "$report_dir"/bridge.log
# The renderer/physics process has no ROS imports; clear sourced ROS library overrides.
env -u ROS_DISTRO -u AMENT_PREFIX_PATH -u PYTHONPATH -u LD_LIBRARY_PATH OMNI_KIT_ACCEPT_EULA=YES bash scripts/r12_runtime.sh python scripts/r12_sensor_sim.py --frames "${R12_SENSOR_FRAMES:-40000}" --port "${R12_SENSOR_PORT:-19612}" > "$report_dir"/sim.log 2>&1

# Kit can exit with code zero after a Python script error; require the explicit completion marker.
rg -q R12_SENSOR_SIM_COMPLETE "$report_dir"/sim.log
test -s "$report_dir"/completion.json
