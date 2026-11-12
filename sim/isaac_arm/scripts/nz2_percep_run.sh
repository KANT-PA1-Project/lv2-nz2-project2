#!/usr/bin/env bash
# NZ2 퍽 인식 실험: IK 집기를 돌리며 look/via/pre 순간의 640×480 손목 영상 + 정답 저장 (개발 PC)
#   → ~/manipulator-isaac61-lyrical-practice/agent/percep/<이름>/*.npz
# 실행: NZ2_PERCEP_DUMP=<이름> bash nz2_percep_run.sh [단계=3] [에피소드=64] [env=8]
#   이어서 평가: python3 ~/manipulator-isaac61-lyrical-practice/scripts/nz2_percep_eval.py ~/manipulator-isaac61-lyrical-practice/agent/percep/<이름>
# 환경 설정(URDF·차체·팔 높이)은 nz2_vispick_run.sh 의 LAB= ~ P= 줄을 그대로 읽어 씀 (중복 없이 한곳에서)
set -eo pipefail
D=$(cd "$(dirname "$0")" && pwd)
[ -n "${NZ2_PERCEP_DUMP:-}" ] || { echo "[오류] NZ2_PERCEP_DUMP=<이름> 필요"; exit 1; }
# shellcheck disable=SC1090
source <(sed -n '/^LAB=/,/^P="physics/p' "$D/nz2_vispick_run.sh")
[ -f "$ARM_CONFIG" ] || { echo "[오류] $ARM_CONFIG 없음"; exit 1; }
export NZ2_PERCEP_MAX="${NZ2_PERCEP_MAX:-64}"
cd "$LAB"
NZ2_PLAY=1 NZ2_EXPERT_MAX_EPS="${2:-64}" NZ2_STAGE="${1:-3}" uv run --no-sync isaaclab zero_agent \
  --task NZ2-VisPick-Fixed-Residual --num_envs "${3:-8}" $P
