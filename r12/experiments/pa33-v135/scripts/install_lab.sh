#!/usr/bin/env bash
set -euo pipefail
LAB_DIR="${LAB_DIR:-$HOME/IsaacLab-3.0-EA}"
command -v uv >/dev/null || { echo 'Install uv from https://docs.astral.sh/uv/getting-started/installation/'; exit 1; }
if [[ ! -e "$LAB_DIR" ]]; then
  git clone --branch v3.0.0-EA --depth 1 https://github.com/isaac-sim/IsaacLab.git "$LAB_DIR"
fi
[[ "$(git -C "$LAB_DIR" describe --tags --exact-match)" == v3.0.0-EA ]] || { echo 'Expected v3.0.0-EA'; exit 1; }
[[ -z "$(git -C "$LAB_DIR" status --porcelain)" ]] || { echo 'Lab source is modified; inspect before installing'; exit 1; }
cd "$LAB_DIR"
uv sync --frozen --extra isaacsim
uv run --frozen --no-sync --extra isaacsim python -c 'import importlib.metadata as m; print("Isaac Sim", m.version("isaacsim"))'
