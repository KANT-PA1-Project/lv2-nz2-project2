#!/usr/bin/env bash
# NZ2 최대 도달 집기 실험 실행기 — 어느 폴더에서 실행해도 됨 (개발 PC)
#   사용: bash ~/manipulator-isaac61-lyrical-practice/scripts/nz2_maxreach_run.sh check        # 점검 (L1, 환경 9개)
#         bash ~/manipulator-isaac61-lyrical-practice/scripts/nz2_maxreach_run.sh all          # 9곳 전체 (장소마다 저장 후 자동 종료)
#         bash ~/manipulator-isaac61-lyrical-practice/scripts/nz2_maxreach_run.sh site S2      # 한 곳만
#         bash ~/manipulator-isaac61-lyrical-practice/scripts/nz2_maxreach_run.sh map L1       # 맵 전체 보기 (환경 1개, 화면 확인용)
set -euo pipefail
LAB=~/pa-course/IsaacLab-6.1
export ARM_CONFIG=~/git/lv2_assignment/nz2/R12_NO_USB_URDF_USD/isaac/nz2_arm_noD435.json
[ -f "$ARM_CONFIG" ] || { echo "[오류] $ARM_CONFIG 없음 → nz2_disable_front_d435.py 를 먼저 실행"; exit 1; }
cd "$LAB"
run() { uv run --no-sync isaaclab zero_agent --task NZ2-MaxReach-Sweep "$@" physics=isaacsim_physx; }
case "${1:-}" in
  check) NZ2_SITE=L1 NZ2_GAPS=0.02 NZ2_SHAPES=box NZ2_TRIALS=1 run ;;
  all)   for S in L1 L3 S1 S2 S3 S4 S5 S6 LINE; do
           echo "===== $S ====="
           NZ2_SITE=$S NZ2_EXIT_WHEN_DONE=1 run || echo "[경고] $S 비정상 종료 (다음 장소 계속)"
         done
         echo "결과: $LAB/logs/nz2_maxreach/" ;;
  site)  NZ2_SITE="${2:?장소 이름 필요 (L1 L3 S1~S6 LINE)}" NZ2_EXIT_WHEN_DONE=1 run ;;
  map)   NZ2_SITE="${2:-L1}" NZ2_FULL_MAP=1 run --num_envs 1 ;;
  *)     sed -n '2,6p' "$0"; exit 2 ;;
esac
