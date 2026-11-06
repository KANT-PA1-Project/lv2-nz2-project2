#!/usr/bin/env bash
# Reuse existing large Sim wheels read-only; install the EA adapter in an isolated venv.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LAB_DIR="${LAB_DIR:-$ROOT/../IsaacLab-3.0-EA}"
SIM_SITE="${R12_SIM_SITE:?Set R12_SIM_SITE to the existing Isaac Sim 6.1 site-packages}"
LAB_SITE="${R12_LAB_SITE:?Set R12_LAB_SITE to the existing Lab dependency site-packages}"
[[ "$(git -C "$LAB_DIR" describe --tags --exact-match)" == v3.0.0-EA ]]
python3 -m venv "$ROOT/.venv-r12"
"$ROOT/.venv-r12/bin/python" - "$SIM_SITE" "$LAB_SITE" <<'PY'
import site,sys
from pathlib import Path
(Path(site.getsitepackages()[0])/'runtime_base.pth').write_text('\n'.join(sys.argv[1:])+'\n')
PY
args=(-e "$ROOT")
for package in isaaclab isaaclab_assets isaaclab_tasks isaaclab_rl isaaclab_physx isaaclab_newton isaaclab_visualizers isaaclab_ov isaaclab_contrib isaaclab_experimental isaaclab_tasks_experimental isaaclab_ppisp; do
    args+=(-e "$LAB_DIR/source/$package")
done
"$ROOT/.venv-r12/bin/python" -m pip install --no-deps "${args[@]}" \
    usd-exchange==2.3.0 warp-lang==1.16.0 newton==1.5.2 rsl-rl-lib==5.4.1
"$ROOT/.venv-r12/bin/python" -m pip install 'moviepy>=1.0.3,<2.0.0.dev0' imageio-ffmpeg
"$ROOT/.venv-r12/bin/python" -m pip freeze > "$ROOT/reports/local/r12-runtime-freeze.txt"
