#!/usr/bin/env bash
set -euo pipefail
LAB_DIR="${LAB_DIR:-$HOME/IsaacLab-3.0-EA}"
[[ "$(git -C "$LAB_DIR" describe --tags --exact-match)" == v3.0.0-EA ]] || { echo 'Expected Lab v3.0.0-EA'; exit 1; }
cd "$LAB_DIR"
# --no-sync preserves the separately installed arm-practice entry point.
exec uv run --frozen --no-sync --extra isaacsim isaaclab "$@"
