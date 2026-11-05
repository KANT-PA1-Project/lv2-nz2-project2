#!/usr/bin/env bash
set -euo pipefail
[[ $# -eq 2 ]] || { echo 'Usage: bash scripts/convert_urdf.sh input.urdf output.usd'; exit 2; }
INPUT="$(realpath -e "$1")"
OUTPUT="$(realpath -m "$2")"
[[ ! -e "$OUTPUT" ]] || { echo 'Output already exists. Choose another filename.'; exit 1; }
LAB_DIR="${LAB_DIR:-$HOME/IsaacLab-3.0-EA}"
[[ "$(git -C "$LAB_DIR" describe --tags --exact-match)" == v3.0.0-EA ]] || { echo 'Expected Lab v3.0.0-EA'; exit 1; }
mkdir -p "$(dirname "$OUTPUT")"
cd "$LAB_DIR"
# Keep fixed joints so the configured TCP link remains available.
uv run --frozen --no-sync --extra isaacsim python scripts/tools/convert_urdf.py "$INPUT" "$OUTPUT" --fix-base --joint-target-type position
test -s "$OUTPUT"
echo "Created: $OUTPUT"
