# NZ2 로봇팔 집기·적재통 시뮬레이션 (Isaac Lab 6.1)

브랜치 `arm-pick-train-SuYongChung`. 대회용 OpenMANIPULATOR-X 개조 팔(5축 + 그리퍼)이 시작 박스의 퍽을 집어
적재통 컵에 세워 넣는 동작을 Isaac Lab에서 검증한 코드와 결과다. 동작은 **IK 상태기계(스크립트 전문가)** 이며,
같은 환경에서 잔차(residual) RL을 얹을 수 있다.

## 폴더
| 경로 | 내용 |
|---|---|
| `src/arm_practice/tasks/` | 작업 정의. `__init__.py`(gym 등록), `nz2_vispick_cfg.py`(장면·리셋·종료·적재통 컵), `nz2_vispick_residual.py`(IK 상태기계 + 잔차 행동 + 통계), `nz2_arm_kin.py`(해석 FK/IK, URDF 치수 자동 읽기), `nz2_vispick_geom.py`(박스·카메라 치수), `nz2_vispick_obs.py`(카메라 관측), `nz2_percep_dump.py`(인식 실험 영상 저장) |
| `scripts/nz2_vispick_run.sh` | 실행 진입점 (`usd`, `cam`, `expert-check`, `residual`, `residual-play` …) |
| `scripts/nz2_agent_runner.sh` | 실험 대기열 실행기 (`agent/queue/*.job` → `agent/results/`) |
| `scripts/nz2_percep_run.sh`, `nz2_percep_eval.py` | 인식 실험 (보류) |
| `agent/jobs/*.job` | 실험 001~043 조건 (환경 변수) |
| `agent/results/*.json`, `*.summary.txt` | 실험 결과 (성공률, 조건별·단계별 분해) |
| `docs/` | 설계·결과·인계 문서 |

## 실행 (개발 PC)
```bash
# 1회: 팔만 남긴 USD 만들기 (URDF 가 바뀌면 다시)
bash scripts/nz2_vispick_run.sh usd
# 카메라 점검
bash scripts/nz2_vispick_run.sh cam
# 대기열 실행기 (터미널 하나에서만, GPU 8 GB 에서 Isaac 동시 실행 금지)
bash scripts/nz2_agent_runner.sh
```
job 예 (`agent/queue/NNN_이름.job`):
```
MODE=expert-check
NZ2_TO_BIN=1
NZ2_TARGET_LIE=1
NZ2_CHASSIS=r20d
NZ2_LIE_ROLL=1
STAGE=3
NOISE=1
```
경로 의존: `nz2_vispick_run.sh` 의 `LAB=~/pa-course/IsaacLab-6.1`, `R12=~/git/lv2_assignment/nz2/R12_NO_USB_URDF_USD` (URDF·USD·ARM_CONFIG). 다른 PC 에서는 이 두 줄을 바꾼다.

## 주요 환경 변수
| 변수 | 기본 | 뜻 |
|---|---|---|
| `NZ2_CHASSIS=r20d` | - | R20D 차체 위 실기 팔 높이 0.209 m + 차체 충돌 근사 |
| `NZ2_TO_BIN` | 0 | 1 = 집은 뒤 적재통 컵에 넣기 단계 이어 붙임 |
| `NZ2_TARGET_LIE` | 0 | 1 = 목표 퍽 항상 누운 직육면체 |
| `NZ2_BIN_FLOOR_MM` | 300 | 컵 바닥 높이 [가정] |
| `NZ2_BIN_GRASP_OFF_MM` | 12 | 누운 퍽을 위로 갈 끝 쪽으로 치우쳐 쥐는 거리 (후보 14) |
| `NZ2_CYL_CAN_LIE` | 0 | 1 = 원통도 눕힘 (운영진 공지 전 조건) |
| `NZ2_POSE_BIAS_MM` / `NZ2_REFINE_BIAS_MM` | 6 / −1 | 인식 오차 흉내 (편향 / pre 재관측 후 편향) |
| `NZ2_SETTLE` | 1 | 정지 확인 게이트 |

## 결과 요약 (256 에피소드)
| 실험 | 조건 | 성공 |
|---|---|---|
| 030 | 집기, 원통은 세움만, 인식 오차 (편향 6 / 재관측 2 / z 2 mm) | **99.6 %** |
| 037 | 누운 퍽 세워 적재통, 인식 오차 0 | **99.6 %** |
| 038 / 043 / 039 | 같은 조건 + 인식 오차, 쥐는 점 12 / 14 / 15 mm | 90.6 / **93.4** / 91.8 % |
| 042 | 세운 퍽 적재통 (컵 바닥 0.30 m) | 0 % (계획 단계 IK 불가) |
자세한 내용은 `docs/nz2_bin_tilt_plan.md`.

## 알려진 문제
- **컵 높이 상충**: 차체 윗면(0.192 m)에서는 누운 퍽 세워 넣기 불가, 받침 0.30 m 에서는 세운 퍽 위에서 넣기 불가 → 최종 URDF 로 결정
- **시뮬 카메라 빈 화면**: 집기 작업(Residual)에서는 손목·상단 카메라 영상이 단색. 카메라 점검(Fixed)에서는 정상. 원인 미해결 (`docs/nz2_perception_models_v2.md`)
- 치수는 URDF `r12_arm_train.urdf`(R12) 기준 [가정]. 최종 URDF 반영 전

## 문서
- `docs/nz2_handoff_2026-11-11_night.md` — 최신 인계 (결정·결과·파일·다음 할 일)
- `docs/nz2_bin_tilt_plan.md` — 누운 퍽 세워 적재통 넣기 설계·도달 검사·결과
- `docs/nz2_perception_models_v2.md` — 튜터님 인식 모델(보류)·시뮬 카메라 진단
