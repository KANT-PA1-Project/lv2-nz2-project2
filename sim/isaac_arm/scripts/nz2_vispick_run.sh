#!/usr/bin/env bash
# NZ2 팔만(차체 없음) 바닥 고정 + 카메라 영상 RL 실행기 (개발 PC, 어느 폴더에서든)
#   usd               : 학습 USD → 팔만 남긴 _armonly USD + nz2_arm_armonly.json 생성 (URDF/USD 바뀔 때마다 1번)
#   geom              : 카메라 시야·가림·관측 자세 사전 계산 (Isaac 없음)
#   cam [단계]        : 환경 4개, 영상 PNG 저장 + 퍽 투영 검사 후 자동 종료
#   state-check|state : 카메라 없이 상태 학습 (16×10 점검 | 128×1500)
#   check|train [env] : 영상 학습 (8×5 점검 | 32×1500)
#   resume cam|state [체크포인트.pt] [추가반복=500] : 이어 학습 (생략 시 가장 최근 실행의 마지막 모델)
#                     이전 실행의 마지막 Curriculum/vispick_stage 값을 NZ2_STAGE_START=<값> 으로 앞에 붙임
#   play cam|state [체크포인트.pt] [단계=1]          : 재생 (증강·잡음 끔)
#   (단계 3 = mission.pdf 배치: 박스 중심 ±40 mm 퍽 2개)
#   expert-play [env=4] [단계=3] [잡음=1] : IK 집기 재생 (최종 설정, 화면으로 보기, 끝없이 반복 → Ctrl+C)
#   expert-check [단계=3] [잡음=1] : IK 단독 성공률 (카메라 없음, 64 env, 256 에피소드 후 결과 json·종료). 잡음 0 = 정답 위치
#   residual [env=32] [단계=3] [반복=600] : IK + 잔차 RL 학습 (영상, α=NZ2_RES_ALPHA 기본 0.10, 처음 50 반복 비평가 워밍업)
#   residual-resume [체크포인트.pt] [추가반복=300] : 잔차 RL 이어 학습 (워밍업 없음)
#   residual-play [체크포인트.pt] [단계=3] : 잔차 정책 재생 (증강 끔, 인식 오차는 유지)
#   tb                : TensorBoard (http://localhost:6006)
#   runs              : 저장된 실행·체크포인트 목록
set -euo pipefail
LAB=~/pa-course/IsaacLab-6.1
R12=~/git/lv2_assignment/nz2/R12_NO_USB_URDF_USD
SRC_USD="${NZ2_SRC_USD:-$R12/urdf/r12_arm_train_pos/r12_arm_train/r12_arm_train.usda}"
export ARM_CONFIG=$R12/isaac/nz2_arm_armonly.json
export NZ2_URDF="${NZ2_URDF:-$R12/urdf/r12_arm_train.urdf}"   # 해석 IK 치수를 이 URDF 에서 자동으로 읽음 (새 URDF 면 이 경로를 바꾸고 usd 다시)
export NZ2_TOPCAM_PITCH_DEG="${NZ2_TOPCAM_PITCH_DEG:-65}"
# 차체 프리셋 (11/10): NZ2_CHASSIS=r20d → 새 R20D 차체 위 실기 팔 높이·위치 + 차체 충돌 근사
#   팔 베이스 z = 2020 프로파일 윗면 0.187 + 마운트 판 0.005[가정] + OM-X 베이스→joint1 0.017[가정] = 0.209 m
#   팔 위치: 차체 중심에서 앞으로 NZ2_ARM_FWD [m] (기본 0.12 [가정], 앞 레일 0.16~0.18 과 가로대 0.06~0.08 사이)
#   차체 앞끝 = 앞 스워브 모듈 0.218 → 0.22 m (차체 중심 기준). 팔 기준으로는 0.22 − ARM_FWD
#     → 박스 앞면 거리 NZ2_CHASSIS_FRONT_X = 0.22 − ARM_FWD (+간격 10~40 mm는 cfg 에서)
#     → 차체 충돌 근사: 팔 앞쪽 r 0 ~ (0.22 − ARM_FWD + 0.005), 윗면 0.197 m (브라켓 0.192 + 볼트 머리)
#   주의: 시뮬 장면에는 차체가 없음(팔만) → 차체 충돌은 계획 근사로만 피함. TO_VIA(관절 보간)는 검사 안 함
if [ "${NZ2_CHASSIS:-}" = r20d ]; then
  FWD="${NZ2_ARM_FWD:-0.12}"
  export NZ2_ARM_Z="${NZ2_ARM_Z:-0.209}"
  export NZ2_CHASSIS_FRONT_X="${NZ2_CHASSIS_FRONT_X:-$(awk -v f="$FWD" 'BEGIN{printf "%.3f", 0.22 - f}')}"
  export NZ2_CHASSIS_KEEPOUT="${NZ2_CHASSIS_KEEPOUT:-$(awk -v f="$FWD" 'BEGIN{printf "0.0,%.3f,0.197", 0.22 - f + 0.005}')}"
  echo "[NZ2] 차체 R20D: 팔 베이스 z $NZ2_ARM_Z m, 차체 중심→팔 $FWD m, 팔→차체 앞끝(박스 기준) $NZ2_CHASSIS_FRONT_X m, 차체 충돌 근사 $NZ2_CHASSIS_KEEPOUT"
fi
export NZ2_ARM_Z="${NZ2_ARM_Z:-0.0}"       # 팔 베이스(arm_joint1) 높이 [m] = 바닥 z=0 고정 (사용자 결정). 박스 70 mm·퍽 윗면 130 mm 는 mission.pdf
export NZ2_KIN_J1_Z="$NZ2_ARM_Z"            # 전문가 IK 의 팔 베이스 높이 = 환경 팔 높이 (같아야 함, 아니면 expert 가 중단)
T=NZ2-VisPick-Fixed
P="physics=isaacsim_physx"
cd "$LAB"
need_cfg() { [ -f "$ARM_CONFIG" ] || { echo "[오류] $ARM_CONFIG 없음 → 먼저: bash $0 usd"; exit 1; }; }
task_of() { [ "$1" = state ] && echo "$T-State" || echo "$T"; }
exp_of()  { [ "$1" = state ] && echo nz2_vispick_state || echo nz2_vispick; }
latest_ckpt() {   # $1 = 실험 이름 → 반복 수가 가장 큰 체크포인트 (점검용 10회 실행이 뽑히지 않게)
  local f; f=$(ls logs/rsl_rl/"$1"/*/model_*.pt 2>/dev/null | awk -F'model_' '{split($2,a,".");print a[1]"\t"$0}' | sort -n | tail -1 | cut -f2)
  [ -n "$f" ] || { echo "[오류] logs/rsl_rl/$1 에 체크포인트 없음" >&2; exit 1; }
  echo "$f"
}
iter_of() { basename "$1" .pt | sed 's/model_//'; }
case "${1:-}" in
  usd)         ~/isaacsim/python.sh ~/manipulator-isaac61-lyrical-practice/scripts/nz2_make_armonly_usd.py "$SRC_USD" ;;
  geom)        uv run --no-sync python ~/manipulator-isaac61-lyrical-practice/scripts/nz2_vispick_geom_check.py ;;
  cam)         need_cfg; NZ2_CAM_CHECK=1 NZ2_EXIT_WHEN_DONE=1 NZ2_STAGE="${2:-1}" uv run --no-sync isaaclab zero_agent --task $T --num_envs 4 $P ;;
  state-check) need_cfg; uv run --no-sync isaaclab train --rl_library rsl_rl --task $T-State --num_envs 16 --max_iterations 10 $P ;;
  state)       need_cfg; uv run --no-sync isaaclab train --rl_library rsl_rl --task $T-State --num_envs 128 --max_iterations 1500 $P ;;
  check)       need_cfg; uv run --no-sync isaaclab train --rl_library rsl_rl --task $T --num_envs 8 --max_iterations 5 $P ;;
  train)       need_cfg; uv run --no-sync isaaclab train --rl_library rsl_rl --task $T --num_envs "${2:-32}" --max_iterations 1500 $P ;;
  resume)      need_cfg; K="${2:?cam 또는 state}"; C="${3:-$(latest_ckpt "$(exp_of "$K")")}"; N="${4:-500}"
               I=$(iter_of "$C"); echo "이어 학습: $C (반복 $I 부터 +$N)"
               NZ2_ITER_OFFSET="$I" uv run --no-sync isaaclab train --rl_library rsl_rl --task "$(task_of "$K")" \
                 --checkpoint "$C" --run_name "resume_from_$I" --max_iterations "$N" $P ;;
  play)        need_cfg; K="${2:?cam 또는 state}"; C="${3:-$(latest_ckpt "$(exp_of "$K")")}"; echo "재생: $C"
               NZ2_PLAY=1 NZ2_STAGE="${4:-1}" uv run --no-sync isaaclab play --rl_library rsl_rl --task "$(task_of "$K")" \
                 --num_envs 4 --checkpoint "$C" --viz "${NZ2_VIZ:-newton}" $P ;;
  expert-check) need_cfg; Z="${3:-1}"; [ "$Z" = 0 ] && export NZ2_POSE_BIAS_MM=0 NZ2_POSE_NOISE_MM=0
               NZ2_EXPERT_MAX_EPS=256 NZ2_STAGE="${2:-3}" uv run --no-sync isaaclab zero_agent \
                 --task $T-Residual-State --num_envs 64 $P ;;
  expert-play) need_cfg; Z="${4:-1}"; [ "$Z" = 0 ] && export NZ2_POSE_BIAS_MM=0 NZ2_POSE_NOISE_MM=0
               # 최종 설정(11/10, 018): 누운 퍽 롤 맞춤 + z 오차 2/1 mm + 근접 재관측 2 mm. 끝없이 반복 재생(Ctrl+C 종료)
               export NZ2_LIE_ROLL="${NZ2_LIE_ROLL:-1}" NZ2_REFINE_BIAS_MM="${NZ2_REFINE_BIAS_MM:-2}"
               # (11/11) 019 결정: 매 스텝 잡음 0 (잡기점 고정). 026 정지 게이트·처짐 보상 static 은 코드 기본값
               export NZ2_POSE_BIAS_Z_MM="${NZ2_POSE_BIAS_Z_MM:-2}" NZ2_POSE_NOISE_Z_MM="${NZ2_POSE_NOISE_Z_MM:-0}" NZ2_POSE_NOISE_MM="${NZ2_POSE_NOISE_MM:-0}"
               NZ2_EXPERT_MAX_EPS=0 NZ2_STAGE="${3:-3}" uv run --no-sync isaaclab zero_agent \
                 --task $T-Residual-State --num_envs "${2:-4}" $P ;;
  residual)    need_cfg; NZ2_RES_ENVS="${2:-32}" NZ2_STAGE="${3:-3}" uv run --no-sync isaaclab train --rl_library rsl_rl \
                 --task $T-Residual --num_envs "${2:-32}" --max_iterations "${4:-600}" $P ;;
  residual-resume) need_cfg; C="${2:-$(latest_ckpt nz2_vispick_residual)}"; I=$(iter_of "$C"); echo "잔차 이어 학습: $C"
               NZ2_RES_WARMUP_ITERS=0 NZ2_ITER_OFFSET="$I" NZ2_STAGE="${4:-3}" uv run --no-sync isaaclab train --rl_library rsl_rl \
                 --task $T-Residual --checkpoint "$C" --run_name "resume_from_$I" --max_iterations "${3:-300}" $P ;;
  residual-play) need_cfg; C="${2:-$(latest_ckpt nz2_vispick_residual)}"; echo "재생: $C"
               NZ2_RES_WARMUP_ITERS=0 NZ2_PLAY=1 NZ2_STAGE="${3:-3}" uv run --no-sync isaaclab play --rl_library rsl_rl \
                 --task $T-Residual --num_envs 4 --checkpoint "$C" --viz "${NZ2_VIZ:-newton}" $P ;;
  tb)          uv run --no-sync tensorboard --logdir logs/rsl_rl --port 6006 ;;
  runs)        for e in nz2_vispick nz2_vispick_state nz2_vispick_residual; do echo "== $e"; for d in $(ls -td logs/rsl_rl/$e/*/ 2>/dev/null); do
                 echo "  $d  마지막 $(ls "$d"model_*.pt 2>/dev/null | sort -t_ -k2 -n | tail -1 | xargs -r basename)"; done; done ;;
  *)           sed -n '2,18p' "$0"; exit 2 ;;
esac
