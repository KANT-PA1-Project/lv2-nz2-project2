#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM_SITE="${R12_SIM_SITE:-$HOME/isaacsim6_venv/lib/python3.12/site-packages}"
USD_LIBS=("$SIM_SITE"/isaacsim/extscache/omni.usd.libs-*)
# Use Kit's USD Python bindings and native libraries together (not standalone pxr).
export PYTHONPATH="${USD_LIBS[0]}${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="${USD_LIBS[0]}/bin${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export ARM_CONFIG="${ARM_CONFIG:-$ROOT/configs/r12_robot.json}"
cd "$ROOT"
if [[ "${1:-}" == python ]]; then
    shift
    exec "$ROOT/.venv-r12/bin/python" "$@"
fi
exec "$ROOT/.venv-r12/bin/isaaclab" "$@"
