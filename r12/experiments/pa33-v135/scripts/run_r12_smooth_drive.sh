#!/usr/bin/env bash
# v433-validated S1 control profile; full-maze performance remains to be measured.
set -e
if [ "$#" -lt 1 ]; then
    echo "Usage: $0 UNIQUE_RUN [run_r12_headless.py options]" >&2
    exit 2
fi
run="$1"
shift
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
overlay="${R12_ROS_OVERLAY_ROOT:-$HOME/ros2_ws/src/turtlebot-swerve-cuvslam}"
source /opt/ros/lyrical/setup.bash
source "$overlay/scripts/robot_localization_env.sh"
source "$overlay/scripts/nvblox_env.sh"
source "$HOME/.config/isaac-fleet/ros.env"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
cd "$repo"
exec "$HOME/.local/bin/isaac-fleet-run" --run-id "$run" -- \
    "$repo/.venv-r12/bin/python" "$repo/scripts/run_r12_headless.py" \
    --run "$run" --domain "$ROS_DOMAIN_ID" --port "$FLEET_SENSOR_PORT" \
    --frames 12000 --cruise-speed .6 --residual-scale 0 --wheel-kd .04 \
    --continuous-path --imu-latest-sampling --approach-radius .10 \
    --low-speed-residual-fraction .25 --goal-stop-hold --async-path-smoothing \
    --prepared-camera-rig --incremental-result-logs --rviz --rviz-after-tracking \
    --rviz-domain "$FLEET_RVIZ_DOMAIN_ID" --record-video \
    --dds-profile "$repo/configs/sensors/fleet_shm64.xml" "$@"
