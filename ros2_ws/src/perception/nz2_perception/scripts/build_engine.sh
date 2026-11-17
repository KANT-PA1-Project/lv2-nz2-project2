#!/usr/bin/env bash
# Jetson 에서 실행: .pt → TensorRT FP16 엔진 (엔진은 TensorRT 버전/GPU 에 묶이므로 노트북에서 만들면 안 됨)
# 사용: bash jetson/build_engine.sh [models/current.pt] [H] [W]
#   기본 384x640: D435 848x480 을 640 폭으로 줄이면 640x362 → 384 로 패딩. 640x640 대비 픽셀 40% 감소
set -euo pipefail
PT=${1:-models/current.pt}
H=${2:-384}
W=${3:-640}
source ~/yolo_env/bin/activate

OUT="${PT%.pt}_${H}x${W}_fp16.engine"
python - "$PT" "$H" "$W" "$OUT" <<'EOF'
import sys, shutil
from ultralytics import YOLO
pt, h, w, out = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), sys.argv[4]
m = YOLO(pt)
print("classes:", m.names)
eng = m.export(format="engine", half=True, imgsz=[h, w], batch=1, dynamic=False,
               workspace=2, simplify=True, device=0)
shutil.move(eng, out)
print("ENGINE:", out)
EOF
ln -sf "$(basename "$OUT")" "$(dirname "$PT")/current.engine"
echo "→ $(dirname "$PT")/current.engine 링크 갱신"
echo "확인: python jetson/bench.py --model $(dirname "$PT")/current.engine --imgsz $H $W"
