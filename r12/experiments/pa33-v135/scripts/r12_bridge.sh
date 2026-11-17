#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -n "${ROS_DISTRO:-}" ]]; then
    echo 'Use a fresh terminal without a sourced ROS distribution for the Sim process.' >&2
    exit 1
fi
SIM_SITE="${R12_SIM_SITE:-$HOME/isaacsim6_venv/lib/python3.12/site-packages}"
export LD_LIBRARY_PATH="$SIM_SITE/isaacsim/exts/isaacsim.ros2.core/jazzy/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-78}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
exec bash "$ROOT/scripts/r12_runtime.sh" python "$ROOT/scripts/r12_sim_bridge.py" "$@"
