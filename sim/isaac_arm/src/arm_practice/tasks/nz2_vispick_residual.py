"""NZ2 바닥 고정 팔: 스크립트 IK 전문가 + 잔차(residual) RL 행동 (실행 시점에만 불러옴)

설치 위치: ~/manipulator-isaac61-lyrical-practice/src/arm_practice/tasks/nz2_vispick_residual.py
같은 폴더 필요: nz2_arm_kin, nz2_smooth_action, nz2_common, nz2_vispick_cfg, nz2_vispick_geom
주의: Isaac 실행용 모듈을 import 한다 → 설정에서는 문자열 class_type 으로만 가리킨다 (pxr 충돌 방지, 11/3 교훈).

행동 합성 (환경마다, 30 Hz)
  a = a_IK(pose_hat) + α · tanh(π(o)) · gate
    a_IK   : 스크립트 IK 상태기계가 낸 관절 속도 명령 [−1, 1] (1 = 관절 최대 속도)
    pose_hat = 퍽 정답 위치 + 인식 오차 흉내 (에피소드 고정 편향 σ 6 mm + 매 스텝 잡음 σ 2 mm, AR(1) τ 0.1 s)  [가정]
    π(o)   : 정책 출력 (영상 2대 + 단계 one-hot …). 마지막 층 0 초기화 → 시작 성능 = IK 단독 성능
    α      : 관절 최대 속도의 비율 (기본 0.10, NZ2_RES_ALPHA). 팔 5축만, 그리퍼는 IK 만 (잡은 퍽을 정책이 놓지 못하게)
    gate   : APPROACH · CLOSE · LIFT 에서만 1 (WAIT · TO_VIA · DESCEND = 이동 구간은 0 → 큰 동작 중 충돌 위험 없음)
  잔차는 '관절 오프셋'으로 적분된다: off += α·tanh(π)·v_max·dt, |off| ≤ RES_MAX_RAD → 목표각 = IK 목표 + 처짐보상 + off
    (속도만 더하면 다음 스텝 IK 가 바로 되돌려 효과가 1 스텝 3 mrad 에 그침 → 오프셋으로 유지)
    LIFT 이후(HOLD)에는 오프셋 유지, 새 에피소드에서 0
단계: 0 WAIT 0.3 s → 1 TO_VIA → 2 DESCEND via→pre → 3 APPROACH pre→grasp → 4 CLOSE 1.0 s → 5 LIFT +40 mm → 6 HOLD 1.5 s
      → 7 DONE   8 SKIP = 도달 불가 (a = 0)
안전장치
  - 시작 시 FK 자기검사 (해석 FK vs 시뮬 TCP > 20 mm 이면 중단), J1_Z ≠ ARM_Z 면 중단
  - 관절 속도 ≤ max_velocity, 목표는 관절 한계 안쪽 (부모 클래스), IK 실패 시 그 자리 정지 + SKIP
  - 잔차 오프셋 |off| ≤ 0.08 rad (팔 5축) — 정책이 망가져도 피해 상한
  - 처짐 보상 ±0.15 rad, 추종 오차 > 0.12 rad 이면 경로 진행 멈춤
  - (11/10 추가, 선택) 닫기 전 손끝 위치 게이트: FK(실제 관절) 와 잡기점 거리 < NZ2_CLOSE_GATE_MM 일 때만 CLOSE.
    최대 CLOSE_WAIT_S 기다린 뒤에는 그냥 닫는다 (멈춰서 시간초과 나지 않게). 0 = 끔(기존 동작)
  - (11/10 추가) 옆 퍽 장애물: 옆 퍽이 팔 평면(반경선) 위에 있으면 그 퍽을 (r 구간, 윗면) 장애물로 넣어 계획한다.
    → 손목(link5)·손가락 뿌리가 앞 퍽을 치거나 손끝이 뒤 퍽을 건드리는 하향각을 계획 단계에서 거른다.
    NZ2_OTHER_KEEPOUT=0 이면 끔, 여유 NZ2_OTHER_KEEP_MM(기본 3 mm). 옆 퍽 위치에도 같은 인식 편향을 넣는다.
  - (11/10 추가) 누운 퍽 손목 롤 맞춤 (NZ2_LIE_ROLL=1 기본):
    집게 축 ĵ(φ) = cosφ·ŝ + sinφ·(sinP·r̂ + cosP·ẑ)   (r̂ 반경, ŝ = ẑ×r̂, P 하향각, φ = j5 롤)
    퍽 긴 축 â = cosψ·r̂ + sinψ·ŝ 에 수직(지름 쥠)이 되는 롤: φ_d = atan2(−sinψ, sinP·cosψ)  (±90° 로 접음)
    단, φ≠0 이면 집게 축이 수평에서 τ = asin(|sinφ|·cosP) 만큼 기울어 아래 손가락이 박스로 내려감
      τ ≤ NZ2_ROLL_TILT_MAX_DEG(25°)  → φ_d (지름 쥠, 모드 diam)
      그 외 ψ > 60° (축이 거의 반경⊥) → φ = 0, 긴 쪽(≤ 67 mm)을 쥠 (모드 length, 최대 열림 75 mm)
      그 외                          → τ 한계까지만 돌림 (모드 partial)
    퍽 yaw 인식 오차: 에피소드 고정 σ NZ2_YAW_BIAS_DEG(5°). 세운 퍽은 롤 0.
    자기검사: CLOSE 진입 시 실제 손가락 두 개 위치로 집게 축을 재서 |cos(집게 축, 퍽 축)| 기록 (diam ≈ 0, length ≈ 1)
  - (11/10 추가) via 들어올리기 NZ2_VIA_UP_MM(기본 60): TO_VIA(관절 보간, 충돌 검사 없음) 목표를 via 보다 위로 올려
    손가락이 옆 퍽·박스를 쓸고 지나가지 않게 함. 올린 점이 IK 불가면 원래 via. 이후 DESCEND 직선이 그만큼 길어짐
통계: NZ2_EXPERT_MAX_EPS=N 이면 N 에피소드 뒤 성공률 출력 후 종료 (expert-check)
  (11/10 추가) 조건별 분해: 퍽 종류·자세(누움 축 방향 / 세운 사각 면 방향) × 옆 퍽 줄 방향 × 하향각 구간
              + CLOSE 진입 순간 손끝 추종 오차(z, 수평) 평균 → 실패 원인을 조건별로 본다
환경 변수: NZ2_RES_ALPHA(0.10), NZ2_POSE_BIAS_MM(6), NZ2_POSE_NOISE_MM(2), NZ2_EXPERT_MAX_EPS, NZ2_CLOSE_GATE_MM(0),
          NZ2_OTHER_KEEPOUT(1), NZ2_OTHER_KEEP_MM(3), NZ2_LIE_GRASP_UP_MM(0, nz2_vispick_cfg 에서 읽음)
"""
from __future__ import annotations

import json
import math
import os
import time
from collections.abc import Sequence

import sys

import torch

from isaaclab.utils import math as math_utils

from arm_practice.tasks import nz2_arm_kin as K
from arm_practice.tasks import nz2_common as C
from arm_practice.tasks import nz2_vispick_cfg as VC
from arm_practice.tasks import nz2_vispick_geom as G
from arm_practice.tasks import nz2_percep_dump as PD          # (11/11) 인식 실험 영상 저장 (NZ2_PERCEP_DUMP 일 때만)
from arm_practice.tasks.nz2_smooth_action import NZ2SmoothJointPositionAction

# 결과가 파일로 리다이렉트될 때도 줄 단위로 바로 쓰이게 (os._exit 전에 버퍼가 날아가지 않게)
try:
    sys.stdout.reconfigure(line_buffering=True)
except Exception:
    pass

P_WAIT, P_TO_VIA, P_DESCEND, P_APPROACH, P_CLOSE, P_LIFT, P_HOLD, P_DONE, P_SKIP = range(9)
PHASE_NAMES = ["WAIT", "TO_VIA", "DESCEND", "APPROACH", "CLOSE", "LIFT", "HOLD", "DONE", "SKIP"]
NUM_PHASES = 9
# ===== (11/11 저녁) 적재통 놓기 단계 (NZ2_TO_BIN=1 일 때만, 기존 0~8 번호·동작은 그대로) =====
#   LIFT → CARRY(높이 들어 팔 쪽으로) → ALIGN(j5 롤 → 0: 누운 퍽 축을 반경 방향에)
#   → TILT(집은 방향 그대로, 컵 위 경유점 높이에서 공구 하향 90° → 수평 0° = 퍽이 섬)
#   → ROTATE(j1 만 → 컵 방향, 세운 퍽이 컵 벽보다 높이 지나감) → BIN_DESC(컵 위에서 수직 하강) → RELEASE(엶)
#   → RETREAT(수직 위로) → VERIFY(퍽이 컵 안·세움 확인)
#   (v2 11/11: v1 은 컵 방향으로 먼저 돌린 뒤 컵 옆에서 세워 035·036 전부 TILT 중 손가락–컵 접촉 → 세운 뒤 돌리기)
TO_BIN = VC.TO_BIN
P_CARRY, P_ALIGN, P_ROTATE, P_TILT, P_BIN_DESC, P_RELEASE, P_RETREAT, P_VERIFY = range(9, 17)
if TO_BIN:
    PHASE_NAMES = PHASE_NAMES + ["CARRY", "ALIGN", "ROTATE", "TILT", "BIN_DESC", "RELEASE", "RETREAT", "VERIFY"]
    NUM_PHASES = 17
LIFT_NEXT = P_CARRY if TO_BIN else P_HOLD
RES_PHASES = (P_APPROACH, P_CLOSE, P_LIFT)

# ===== 전문가 상수 (한곳에서 바꿈) =====
WAIT_S = float(os.environ.get("NZ2_WAIT_S", "0.3"))       # 진단: 크게 주면 팔이 그동안 가만히 있음 (퍽 자체 움직임 확인)
V_DESCEND = 0.10          # via → pre 직선 속도 [m/s]
V_APPROACH = 0.05         # pre → grasp
V_LIFT = 0.05
CLOSE_S = 1.0
HOLD_S = 1.5
SAG_KI = float(os.environ.get("NZ2_SAG_KI", "3.0"))   # 처짐 보상 적분 이득 [1/s] (0 = 끔)
SAG_MAX = 0.15                                        # 처짐 보상 최대 [rad]
# (11/11) 처짐 보상 적분 조건: all = 이동 중 항상 (기존) / static = 명령 관절속도가 SAG_STATIC_RAD_S 미만일 때만
#   all 은 이동 중 PD 추종 지연(처짐이 아님)까지 적분 → 이동이 멈추거나 느려질 때 아래로 넘침(와인드업) 의심
SAG_MODE = os.environ.get("NZ2_SAG_MODE", "static")   # (11/11) 026 채택: 기본 static
SAG_STATIC_RAD_S = float(os.environ.get("NZ2_SAG_STATIC_RAD_S", "0.05"))
if SAG_MODE not in ("all", "static"):
    raise ValueError(f"NZ2_SAG_MODE 는 all 또는 static: {SAG_MODE}")
# (11/11 오후) 정지 확인 게이트: via(TO_VIA→DESCEND)·pre(DESCEND→APPROACH)에서 팔이 '멈춘 뒤' 다음 단계로
#   조건: 추종 오차 < SETTLE_ERR_RAD 그리고 측정 관절속도 < SETTLE_QD_RAD_S 가 SETTLE_HOLD_S 동안 연속
#   SETTLE_MAX_S 지나도 못 멈추면 그냥 넘어감(시간 초과 횟수 집계). NZ2_SETTLE=0 이면 기존 동작 그대로
#   (11/11) 실험 026 채택 → 기본값: 켬, 오차 0.02 rad, 최대 1.0 s, 처짐 보상 static (95.7 %, 경로 아래 평균 −3.6 mm)
SETTLE = os.environ.get("NZ2_SETTLE", "1") != "0"
SETTLE_ERR_RAD = float(os.environ.get("NZ2_SETTLE_ERR_RAD", "0.02"))
SETTLE_QD_RAD_S = float(os.environ.get("NZ2_SETTLE_QD_RAD_S", "0.05"))
SETTLE_HOLD_S = float(os.environ.get("NZ2_SETTLE_HOLD_S", "0.2"))
SETTLE_MAX_S = float(os.environ.get("NZ2_SETTLE_MAX_S", "1.0"))
# (11/11) 처짐(경로 아래로 내려감) 진단 문턱 [m]
DIP_WARN_M = 0.003
TRACK_GATE_RAD = 0.12
FK_CHECK_MM = 20.0
PITCH_MIN_DEG, PITCH_STEP_DEG = 10.0, 5.0
BOX_TOP = G.BOX[2]
# ===== 닫기 전 손끝 위치 게이트 (선택) =====
CLOSE_GATE_M = float(os.environ.get("NZ2_CLOSE_GATE_MM", "0")) / 1000.0     # 0 = 끔
CLOSE_WAIT_S = 1.0                                                           # 직선 끝난 뒤 최대 대기 [s]
# ===== 근접 재관측 (11/10) =====
# pre 자세(퍽 위 약 5~10 cm)에서 손목 카메라로 다시 보면 오차가 줄어든다 [가정] → APPROACH 시작 때 편향을 이 σ 로 새로 뽑아
# 잡기점·들기점을 고친다. 음수 = 끔. 실기에 쓰려면 pre 자세 정지 → 손목 카메라 재인식 단계를 구현해야 함
REFINE_M = float(os.environ.get("NZ2_REFINE_BIAS_MM", "-1")) / 1000.0
# ===== 옆 퍽 장애물 =====
OTHER_KEEPOUT = os.environ.get("NZ2_OTHER_KEEPOUT", "1") != "0"
OTHER_KEEP_M = float(os.environ.get("NZ2_OTHER_KEEP_MM", "3")) / 1000.0     # r 방향 여유
OTHER_LAT_M = 0.02                                                           # 반경선에서 옆으로 이 안이면 '팔 평면 위'
# ===== 누운 퍽 손목 롤 =====
LIE_ROLL = os.environ.get("NZ2_LIE_ROLL", "1") != "0"
ROLL_TILT_MAX = math.radians(float(os.environ.get("NZ2_ROLL_TILT_MAX_DEG", "25")))
LEN_PSI = math.radians(60.0)
LEN_WHEN_ROW = os.environ.get("NZ2_LEN_WHEN_ROW", "1") != "0"                # 옆 퍽이 반경 방향이면 긴 쪽 쥠                                                 # 이보다 반경에서 더 틀어지면 긴 쪽 쥠
YAW_BIAS = math.radians(float(os.environ.get("NZ2_YAW_BIAS_DEG", "5")))
MODE_NAMES = ["세움(롤 0)", "누움 diam(지름 쥠)", "누움 length(긴 쪽 쥠)", "누움 partial(일부만 돌림)", "누움 롤 끔"]
FINGER_BODIES = ["gripper_left_finger", "gripper_right_finger"]
# ===== 인식 오차 흉내 [가정: 실기 인식 오차 ±6 mm] =====
POSE_BIAS_M = float(os.environ.get("NZ2_POSE_BIAS_MM", "6")) / 1000.0     # 에피소드마다 고정, 축마다 σ
POSE_NOISE_M = float(os.environ.get("NZ2_POSE_NOISE_MM", "2")) / 1000.0   # 매 스텝 잡음 (정상상태 σ)
# z 축은 따로 줄 수 있음 (11/10): 실기에서 퍽 높이는 박스 높이 70 mm + 퍽 치수로 이미 알려져 있어
# 카메라 깊이 오차가 그대로 들어가지 않음 → NZ2_POSE_BIAS_Z_MM / NZ2_POSE_NOISE_Z_MM (기본 = xy 와 같음)
POSE_BIAS_Z_M = float(os.environ.get("NZ2_POSE_BIAS_Z_MM", str(POSE_BIAS_M * 1000))) / 1000.0
POSE_NOISE_Z_M = float(os.environ.get("NZ2_POSE_NOISE_Z_MM", str(POSE_NOISE_M * 1000))) / 1000.0
POSE_NOISE_TAU_S = 0.1                                                       # 잡음 상관 시간 (실기 인식 떨림 흉내)
# ===== 잔차 =====
RES_MAX_RAD = 0.08                            # 오프셋 최대 [rad] (팔 5축)
# ===== via 들어올리기 =====
VIA_UP_M = float(os.environ.get("NZ2_VIA_UP_MM", "60")) / 1000.0
# ===== 들어올림 높이 =====
# 성공 판정은 퍽이 40 mm(VC.LIFT_M) 들려야 함. 기존 LIFT 경로도 정확히 40 mm 라 1 mm 만 미끄러져도 판정 실패(DONE/time_out) → 여유
LIFT_UP_M = float(os.environ.get("NZ2_LIFT_MM", "60")) / 1000.0
# ===== (11/11) 적재통 놓기 상수 [가정] =====
BIN_GRASP_OFF_M = float(os.environ.get("NZ2_BIN_GRASP_OFF_MM", "12")) / 1000.0  # 누운 퍽: 세웠을 때 위로 갈 끝 쪽으로 쥐는 점 이동
#   세운 뒤 손가락(두께 ±13 mm, 퍽 축 방향) 아래 끝 = 퍽 중심 + 12 − 13 = 퍽 바닥 + 29 mm > 컵 벽 22 mm (+ 바닥 위 3 mm)
CARRY_R = float(os.environ.get("NZ2_CARRY_R_MM", "150")) / 1000.0            # 나르는 자세: 팔 베이스에서 반경 [m]
CARRY_Z = float(os.environ.get("NZ2_CARRY_Z_MM", "320")) / 1000.0            # 나르는 자세: TCP 높이 [m, 바닥 기준] (11/11 IK: 0.33 까지 가능)
BIN_VIA_UP = 0.06            # 컵 위 경유점: 놓는 점 + 60 mm
BIN_RETREAT_UP = 0.08        # 놓은 뒤 수직 위로 80 mm
V_BIN = 0.03                 # 컵 안으로 내리는 속도 [m/s]
V_RETREAT = 0.05
PUCK_GAP_M = 0.003           # 놓을 때 퍽 바닥 − 컵 바닥
RELEASE_S = 0.5
RELEASE_FRAC = float(os.environ.get("NZ2_RELEASE_FRAC", "0.63"))           # 열림 비율 (44/70 mm) [가정: 관절↔폭 선형]
VERIFY_S = 1.0
BIN_XY_TOL, BIN_Z_TOL, BIN_TILT_DEG = 0.012, 0.010, 10.0                   # 성공 판정
JMOVE_VFRAC = 0.5            # 관절 보간 이동 속도 = 최대 관절속도 × 이 비율
ALIGN_RAD_S = 1.0            # 롤 맞춤 속도
# ===== 흔들림 진단 문턱 (종료 판정보다 낮게 → 종료 전에 시작 순간을 잡음) =====
WATCH_PUSH_M, WATCH_TILT_DEG = 0.004, 6.0
# ===== 조건별 분해 이름 =====
# 집게 닫힘 방향 = 수평, 반경(팔 베이스→퍽)에 수직 (j5 롤 = 0 고정)
KIND_NAMES = ["누움·축∥반경(지름 쥠)", "누움·축⊥반경(길이 쥠)", "세움·원기둥", "세움·사각 면정렬", "세움·사각 대각"]
ROW_NAMES = ["옆퍽 앞(베이스 쪽)", "옆퍽 뒤(먼 쪽)", "옆퍽 좌우(집게 방향)"]
PITCH_BINS = ((0.0, 27.5, "하향 ≤25°"), (27.5, 42.5, "하향 30~40°"), (42.5, 91.0, "하향 ≥45°"))
COS45 = math.cos(math.radians(45.0))


class NZ2VisPickResidualAction(NZ2SmoothJointPositionAction):
    cfg: "object"

    def __init__(self, cfg: "object", env):
        super().__init__(cfg, env)
        if self.action_dim != 6:
            raise ValueError(f"관절 6개(arm_joint1~5, gripper_joint) 필요: {self._joint_names}")
        if not 0.0 <= cfg.residual_alpha <= 0.5:
            raise ValueError(f"residual_alpha 는 0~0.5: {cfg.residual_alpha}")
        n, dev = self.num_envs, self.device
        self._env_ref = env
        self._dt = env.step_dt
        self._lo = self._clip[0, :, 0].clone()
        self._hi = self._clip[0, :, 1].clone()
        self._vmax = self._max_step[0] / self._dt                                # (6,) rad/s
        self.alpha = float(cfg.residual_alpha)
        # 상태기계
        self.phase = torch.full((n,), P_SKIP, dtype=torch.long, device=dev)
        self.t_ph = torch.zeros(n, device=dev)
        self.s = torch.zeros(n, device=dev)
        self.pitch = torch.zeros(n, device=dev)
        self.wps = torch.zeros(n, 4, 3, device=dev)
        self.q_via = torch.zeros(n, 5, device=dev)
        self.q_start = torch.zeros(n, 5, device=dev)
        self.via_T = torch.ones(n, device=dev)
        self.g_cmd = torch.full((n,), float(self._hi[5]), device=dev)
        self.q_cmd = torch.zeros(n, 6, device=dev)
        self.q_int = torch.zeros(n, 5, device=dev)
        self.q_cmd_prev = torch.zeros(n, 5, device=dev)                         # 처짐 보상 static 판정용
        self.qm_prev = torch.zeros(n, 5, device=dev)                            # 정지 게이트: 측정 관절속도용
        self.settle_t = torch.zeros(n, device=dev)                              # 정지 조건 연속 유지 시간 [s]
        self.wait_t = torch.zeros(n, device=dev)                                # 경로 끝 도달 후 대기 시간 [s]
        self.n_settle = {"via": 0, "pre": 0}
        self.n_settle_to = {"via": 0, "pre": 0}
        self.settle_wait_sum = {"via": 0.0, "pre": 0.0}
        # (11/11) 내려감 진단: 손끝 z − 직전 명령 경로 z 의 최저값(음수 = 경로 아래), 손끝 하강 속도 최저값
        self.p_cmd = torch.zeros(n, 3, device=dev)
        self._last_p = torch.zeros(n, 3, device=dev)
        self.prev_tcp = torch.zeros(n, 3, device=dev)
        self.dip = torch.zeros(n, device=dev)
        self.dip_ph = torch.zeros(n, dtype=torch.long, device=dev)
        self.vz_min = torch.zeros(n, device=dev)
        self.vz_ph = torch.zeros(n, dtype=torch.long, device=dev)
        self.qint_close = torch.zeros(n, device=dev)                           # CLOSE 진입 때 |q_int| 최대 (j2~j4) [rad]
        self.by_dip: dict[str, dict] = {}
        # 인식 오차
        self.pose_bias = torch.zeros(n, 3, device=dev)
        self.pose_noise = torch.zeros(n, 3, device=dev)
        self._noise_a = math.exp(-self._dt / POSE_NOISE_TAU_S)
        self._bias_sig = torch.tensor([POSE_BIAS_M, POSE_BIAS_M, POSE_BIAS_Z_M], device=dev)
        self._noise_sig = torch.tensor([POSE_NOISE_M, POSE_NOISE_M, POSE_NOISE_Z_M], device=dev)
        # 잔차
        self.res_off = torch.zeros(n, 5, device=dev)                             # 적분된 관절 오프셋 [rad]
        self.res_cmd = torch.zeros(n, 5, device=dev)                             # 이번 스텝 tanh(π)·gate (벌점용)
        self._checked = False
        self._tcfg = None
        # 조건 기록 (에피소드마다)
        self.kind = torch.zeros(n, dtype=torch.long, device=dev)
        self.row = torch.zeros(n, dtype=torch.long, device=dev)
        self.close_err = torch.zeros(n, 2, device=dev)                           # CLOSE 진입 시 (z 오차, 수평 오차) [m]
        self.close_rec = torch.zeros(n, dtype=torch.bool, device=dev)
        # 누운 퍽 롤
        self.lie = torch.zeros(n, dtype=torch.bool, device=dev)
        self.psi = torch.zeros(n, device=dev)                                    # 퍽 긴 축의 반경 기준 각 (인식값)
        self.roll = torch.zeros(n, device=dev)
        self.roll_mode = torch.zeros(n, dtype=torch.long, device=dev)
        self.jaw_cos = torch.zeros(n, device=dev)
        self._finger_ids = None
        self.by_mode: dict[str, list[float]] = {}
        # 퍽 흔들림 진단 (11/10): 처음 흔들린 순간의 스텝·단계·손끝 위치
        self.ep_step = torch.zeros(n, dtype=torch.long, device=dev)
        self.dist_step = torch.full((n,), -1, dtype=torch.long, device=dev)
        self.dist_phase = torch.zeros(n, dtype=torch.long, device=dev)
        self.dist_tcp = torch.zeros(n, 3, device=dev)
        self.dist_tip_z = torch.zeros(n, device=dev)
        self.dist_which = torch.zeros(n, dtype=torch.long, device=dev)          # 1 목표, 2 옆 퍽
        self.dist_kind = torch.zeros(n, dtype=torch.long, device=dev)           # 흔들린 퍽: 누움*2 + 모양(0 사각, 1 원기둥)
        self.dist_log: dict[str, list] = {}
        # (11/11) 적재통
        self.bin_ok = torch.zeros(n, dtype=torch.bool, device=dev)
        self.bin_fail = torch.zeros(n, dtype=torch.bool, device=dev)
        self.place_pitch = torch.zeros(n, device=dev)
        self.bin_wps = torch.zeros(n, 3, 3, device=dev)                        # via, place, retreat (팔 좌표)
        self.q_carry = torch.zeros(n, 5, device=dev)
        self.q_rot = torch.zeros(n, 5, device=dev)                             # (v2) TILT 끝 = 집은 방향의 컵 위 경유점 자세
        self.q_bv = torch.zeros(n, 5, device=dev)
        self.jT = torch.ones(n, device=dev)                                    # 관절 보간 시간 [s]
        self.bin_center = torch.zeros(n, 3, device=dev)                        # 놓은 뒤 퍽 중심 기대값
        self.tilt_rec = torch.zeros(n, 3, device=dev)                          # TILT 끝: (|퍽축·z|, TCP z − 퍽 중심 z, 기록함)
        self.verify_err = torch.zeros(n, 3, device=dev)                        # VERIFY: (xy 오차, z 오차, 기울기°)
        self.n_bin_unreach = 0
        for k_ in ("tilt", "bin", "place"):
            self.n_settle[k_], self.n_settle_to[k_], self.settle_wait_sum[k_] = 0, 0, 0.0
        if TO_BIN:
            print(f"[NZ2-BIN] 적재통 놓기 켬: 컵 바닥 {VC.BIN_FLOOR_Z * 1000:.0f} mm, 칸 {VC.BIN_SLOTS}, "
                  f"쥐는 점 이동 {BIN_GRASP_OFF_M * 1000:.0f} mm, 나르는 자세 r {CARRY_R} z {CARRY_Z}, 목표 누운 퍽 고정 {VC.TARGET_LIE}")
        # 통계
        self.active = torch.zeros(n, dtype=torch.bool, device=dev)
        self.n_done = self.n_ok = self.n_skip = self.n_ikfail = 0
        self.max_eps = int(os.environ.get("NZ2_EXPERT_MAX_EPS", "0"))
        self.term_count: dict[str, int] = {}
        self.by_cond: dict[str, dict] = {}
        self.by_pitch: dict[str, list[int]] = {}
        print(f"[NZ2-RES] α={self.alpha} (팔 5축, 최대 속도 비율), 인식 오차 편향 σ xy {POSE_BIAS_M * 1000:.1f} / z {POSE_BIAS_Z_M * 1000:.1f} mm + 잡음 σ xy {POSE_NOISE_M * 1000:.1f} / z {POSE_NOISE_Z_M * 1000:.1f} mm, "
              f"오프셋 한계 ±{RES_MAX_RAD} rad, 잔차 단계 {[PHASE_NAMES[p] for p in RES_PHASES]}, "
              f"닫기 전 손끝 게이트 {'끔' if CLOSE_GATE_M <= 0 else f'{CLOSE_GATE_M * 1000:.1f} mm (최대 대기 {CLOSE_WAIT_S} s)'}, "
              f"옆 퍽 장애물 {'켬 (여유 %.0f mm)' % (OTHER_KEEP_M * 1000) if OTHER_KEEPOUT else '끔'}, "
              f"via 들어올림 +{VIA_UP_M * 1000:.0f} mm, 누운 퍽 잡기 높이 +{VC.LIE_GRASP_UP * 1000:.0f} mm, 누운 퍽 롤 맞춤 "
              f"{'켬 (기울기 한계 %.0f°, yaw 편향 σ %.0f°)' % (math.degrees(ROLL_TILT_MAX), math.degrees(YAW_BIAS)) if LIE_ROLL else '끔'}")
        print(f"[NZ2-RES] 정지 확인 게이트: " + (f"켬 (via·pre, 추종 오차 < {SETTLE_ERR_RAD} rad + 관절속도 < {SETTLE_QD_RAD_S} rad/s, "
              f"{SETTLE_HOLD_S} s 유지, 최대 {SETTLE_MAX_S} s)" if SETTLE else "끔 (기존: via 0.05 rad, pre 0.03 rad)"))
        print(f"[NZ2-RES] 처짐 보상: ki {SAG_KI} /s, 최대 {SAG_MAX} rad, 모드 {SAG_MODE}"
              f"{' (명령 관절속도 < %.2f rad/s 일 때만 적분)' % SAG_STATIC_RAD_S if SAG_MODE == 'static' else ' (이동 중 항상 적분)'}")

    # ------------------------------------------------------------------
    # 보조
    # ------------------------------------------------------------------
    def _tcp_cfg(self):
        if self._tcfg is None:
            self._tcfg = C.tcp_cfg()
            self._tcfg.resolve(self._env_ref.scene)
        return self._tcfg

    def _q_meas(self):
        return self._asset.data.joint_pos.torch[:, self._joint_ids]

    def _fk_self_check(self):
        env = self._env_ref
        q = self._q_meas()[:, :5]
        tcp = C.tcp_pos_w(env, self._tcp_cfg()) - env.scene.env_origins
        e = float((K.fk_tcp(q) - tcp).norm(dim=1).max()) * 1000.0
        print(f"[NZ2-RES] FK 자기검사(시뮬 vs 해석 FK): TCP 오차 최대 {e:.1f} mm (J1_Z={K.J1_Z}, ARM_Z={G.ARM_Z}, "
              f"치수 출처 {K.DIMS_SOURCE}, URDF FK 대조 {K.KIN_INFO.get('fk_vs_urdf_max_mm', '-')} mm)")
        if abs(K.J1_Z - G.ARM_Z) > 1e-6:
            raise RuntimeError(f"NZ2_KIN_J1_Z({K.J1_Z}) 와 NZ2_ARM_Z({G.ARM_Z}) 가 다름 → nz2_vispick_run.sh 로 실행 (둘 다 같은 값)")
        if e > FK_CHECK_MM:
            raise RuntimeError(f"해석 FK 와 시뮬 TCP 가 {e:.0f} mm 다름 → nz2_arm_kin 치수·부호 또는 팔 장착 위치(arm_j1_xyz) 확인")
        self._checked = True

    def phase_onehot(self) -> torch.Tensor:
        return torch.nn.functional.one_hot(self.phase, NUM_PHASES).float()

    def pose_err(self) -> torch.Tensor:
        """IK 가 쓰는 위치 − 정답 위치 [m] (비평가 전용 특권 정보)"""
        return self.pose_bias + self.pose_noise

    # ------------------------------------------------------------------
    # 조건 분류 (새 에피소드, 퍽이 놓인 뒤)
    # ------------------------------------------------------------------
    def _classify(self, ids: torch.Tensor, gp: torch.Tensor):
        """퍽 종류·자세와 옆 퍽 줄 방향을 '반경 방향'(팔 베이스→목표 퍽, 수평) 기준으로 분류.
        집게는 반경에 수직인 수평 방향으로 닫힌다 (j5 롤 0)."""
        env = self._env_ref
        dev = self.device
        st = VC.vp(env)
        pos_t, quat_t, _, _ = VC._color_pose(env, "target")
        pos_o, _, _, _ = VC._color_pose(env, "other")
        c = st.color[ids].unsqueeze(1)
        lie = st.lying[ids].gather(1, c).squeeze(1)
        shp = st.shape[ids].gather(1, c).squeeze(1)                              # 0 직육면체, 1 원기둥
        rad = gp[ids, :2]
        rad = rad / rad.norm(dim=1, keepdim=True).clamp(min=1e-6)
        q = quat_t[ids]
        k = len(ids)
        ez = math_utils.quat_apply(q, torch.tensor([0.0, 0.0, 1.0], device=dev).expand(k, 3))[:, :2]
        ex = math_utils.quat_apply(q, torch.tensor([1.0, 0.0, 0.0], device=dev).expand(k, 3))[:, :2]

        def cos_rad(v):
            v = v / v.norm(dim=1, keepdim=True).clamp(min=1e-6)
            return (v * rad).sum(dim=1).abs()

        s_hat = torch.stack([-rad[:, 1], rad[:, 0]], dim=1)                     # ŝ = ẑ × r̂ (롤 0 집게 축)
        ez_n = ez / ez.norm(dim=1, keepdim=True).clamp(min=1e-6)
        psi = torch.atan2((ez_n * s_hat).sum(dim=1), (ez_n * rad).sum(dim=1))
        self.psi[ids] = psi + torch.randn_like(psi) * YAW_BIAS
        self.lie[ids] = lie
        lie_par = cos_rad(ez) > COS45                                            # 누운 퍽 긴 축이 반경과 나란함
        ang = torch.rad2deg(torch.acos(cos_rad(ex).clamp(0.0, 1.0)))             # 세운 사각 면 각 0~90°
        aligned = torch.minimum(ang, 90.0 - ang) < 22.5
        kind = torch.where(lie, torch.where(lie_par, 0, 1),
                           torch.where(shp == 1, 2, torch.where(aligned, 3, 4)))
        self.kind[ids] = kind.long()
        o = (pos_o - pos_t)[ids, :2]
        along = (o * rad).sum(dim=1)                                             # + = 옆 퍽이 더 멂
        self.row[ids] = torch.where(cos_rad(o) > COS45, torch.where(along < 0, 0, 1), 2).long()

    def _other_keepout(self, ids: torch.Tensor, gp_hat: torch.Tensor):
        """옆 퍽을 팔 평면 장애물 (r0, r1, 윗면 z) 로. 각 값 (k,1) 텐서 → K.clearance 에서 (k,6) 과 브로드캐스트.
        반경선에서 OTHER_LAT_M 넘게 벗어난 환경은 r 구간을 멀리 보내 끔."""
        env = self._env_ref
        st = VC.vp(env)
        pos_o = VC._color_pose(env, "other")[0][ids] - env.scene.env_origins[ids] + self.pose_bias[ids]
        co = (1 - st.color[ids]).unsqueeze(1)
        lie = st.lying[ids].gather(1, co).squeeze(1)
        shp = st.shape[ids].gather(1, co).squeeze(1)
        on = st.on_box[ids].gather(1, co).squeeze(1)
        rad = gp_hat[:, :2] / gp_hat[:, :2].norm(dim=1, keepdim=True).clamp(min=1e-6)
        r_o = (pos_o[:, :2] * rad).sum(dim=1)
        lat = (pos_o[:, 0] * rad[:, 1] - pos_o[:, 1] * rad[:, 0]).abs()
        half = torch.where(lie, torch.full_like(r_o, G.PUCK_BOX[2] / 2),                       # 누움: 긴 축 반 (보수적)
                           torch.where(shp == 1, torch.full_like(r_o, G.PUCK_CYL_R),
                                       torch.full_like(r_o, G.PUCK_BOX[0] / 2 * math.sqrt(2.0))))  # 세운 사각: 대각 반
        top = torch.where(lie, torch.full_like(r_o, BOX_TOP + G.PUCK_BOX[0]), torch.full_like(r_o, BOX_TOP + G.PUCK_BOX[2]))
        use = on & (lat < OTHER_LAT_M)
        far = torch.full_like(r_o, 10.0)
        r0 = torch.where(use, r_o - half - OTHER_KEEP_M, far)
        r1 = torch.where(use, r_o + half + OTHER_KEEP_M, far)
        return ((r0.unsqueeze(1), r1.unsqueeze(1), top.unsqueeze(1)),)

    @staticmethod
    def _roll_for(psi: torch.Tensor, pitch: torch.Tensor, lie: torch.Tensor, row: torch.Tensor | None = None):
        """누운 퍽 j5 롤과 모드. 식은 모듈 설명 참조"""
        sp = torch.sin(pitch).clamp(min=1e-3)
        cp = torch.cos(pitch).clamp(min=1e-3)
        phi = torch.atan2(-torch.sin(psi), sp * torch.cos(psi))
        phi = torch.where(phi > math.pi / 2, phi - math.pi, phi)                 # 집게는 180° 대칭
        phi = torch.where(phi < -math.pi / 2, phi + math.pi, phi)
        tilt = torch.asin((phi.sin().abs() * cp).clamp(max=1.0))
        ok_d = tilt <= ROLL_TILT_MAX + 1e-6
        near_perp = torch.cos(psi).abs() < math.cos(LEN_PSI)
        s_max = (math.sin(ROLL_TILT_MAX) / cp).clamp(max=1.0)
        phi_part = torch.sign(phi) * torch.asin(s_max)
        roll = torch.where(ok_d, phi, torch.where(near_perp, torch.zeros_like(phi), phi_part))
        mode = torch.where(ok_d, 1, torch.where(near_perp, 2, 3))
        # 축⊥반경인 누운 퍽을 지름으로 쥐려면 집게가 반경 방향으로 열림 → 옆 퍽이 앞/뒤(반경 방향)에 있으면
        # 열린 손가락(바깥면 ±48 mm)이 옆 퍽(표면 25 mm)을 침 (11/10: 이 조건 53 %) → 긴 쪽 쥠(롤 0, 집게가 반경⊥)으로 바꿈
        if row is not None and LEN_WHEN_ROW:
            swap = near_perp & (row <= 1)
            roll = torch.where(swap, torch.zeros_like(roll), roll)
            mode = torch.where(swap, torch.full_like(mode, 2), mode)
        if not LIE_ROLL:
            roll, mode = torch.zeros_like(phi), torch.full_like(mode, 4)
        roll = torch.where(lie, roll, torch.zeros_like(roll))
        mode = torch.where(lie, mode, torch.zeros_like(mode))
        return roll, mode.long()

    def _jaw_axis(self) -> torch.Tensor | None:
        """실제 손가락 두 개 위치 차 = 집게 닫힘 축 (월드, 단위벡터). 몸체를 못 찾으면 None"""
        if self._finger_ids is None:
            try:
                ids, _ = self._asset.find_bodies(FINGER_BODIES, preserve_order=True)
                self._finger_ids = list(ids) if len(ids) == 2 else []
            except Exception:
                self._finger_ids = []
            if not self._finger_ids:
                print(f"[NZ2-RES] 경고: 손가락 몸체 {FINGER_BODIES} 를 못 찾음 → 집게 축 자기검사 생략")
        if not self._finger_ids:
            return None
        bp = self._asset.data.body_pos_w.torch
        v = bp[:, self._finger_ids[0]] - bp[:, self._finger_ids[1]]
        return v / v.norm(dim=1, keepdim=True).clamp(min=1e-6)

    # ------------------------------------------------------------------
    # 계획 (새 에피소드 첫 스텝)
    # ------------------------------------------------------------------
    def _plan_bin(self, ids: torch.Tensor, wps: torch.Tensor) -> torch.Tensor:
        """(11/11) 적재통 놓기 계획. 반환 ok (k,)
        누운 퍽(하향 90° 로 쥠): 놓을 때 공구 수평(하향 0°), 롤 0. 쥔 상태에서 TCP − 퍽 중심 = +off·(위로 갈 끝) − 20 mm·공구축
          → 세운 뒤 TCP = 컵 중심 − 20 mm·r̂ + (바닥 + 3 + 30 + off) z
        세운 퍽: 쥘 때 하향각 그대로(퍽이 계속 서 있게), 롤 0. TCP = 퍽 윗면 → 컵 중심 + (바닥 + 3 + 60) z"""
        env = self._env_ref
        dev = self.device
        k = len(ids)
        st = VC.vp(env)
        c = st.bin_xy[ids]
        rc = c / c.norm(dim=1, keepdim=True).clamp(min=1e-6)
        lie = self.lie[ids]
        P = self.pitch[ids]
        half_up = G.PUCK_BOX[0] / 2 + VC.LIE_GRASP_UP                         # 쥘 때 TCP 가 퍽 중심 위로 (15 + 5 mm)
        z_c = VC.BIN_FLOOR_Z + PUCK_GAP_M + G.PUCK_BOX[2] / 2                   # 놓은 뒤 퍽 중심 높이
        place = torch.zeros(k, 3, device=dev)
        place[:, :2] = torch.where(lie.unsqueeze(1), c - half_up * rc, c)
        place[:, 2] = torch.where(lie, z_c + BIN_GRASP_OFF_M, z_c + G.PUCK_BOX[2] / 2)
        pp = torch.where(lie, torch.zeros_like(P), P)
        via = place.clone()
        via[:, 2] += BIN_VIA_UP
        ret = place.clone()
        ret[:, 2] += BIN_RETREAT_UP
        self.place_pitch[ids] = pp
        self.bin_wps[ids] = torch.stack([via, place, ret], dim=1)
        self.bin_center[ids] = torch.cat([c, torch.full((k, 1), z_c, device=dev)], dim=1)
        # 나르는 자세: 집은 방향(j1) 그대로 반경 CARRY_R, 높이 CARRY_Z / 컵 방향으로 돌린 같은 자세
        g = wps[:, 2]
        rg = g[:, :2] / g[:, :2].norm(dim=1, keepdim=True).clamp(min=1e-6)
        carry = torch.cat([CARRY_R * rg, torch.full((k, 1), CARRY_Z, device=dev)], dim=1)
        zero = torch.zeros(k, device=dev)
        qc, ok1 = K.ik(carry, P, self.roll[ids])
        via_p = torch.cat([via[:, :2].norm(dim=1, keepdim=True) * rg, via[:, 2:3]], dim=1)   # 컵 위 경유점을 집은 방향으로
        qr, ok2 = K.ik(via_p, pp, zero)
        qv, ok3 = K.ik(via, pp, zero)
        _, ok4 = K.ik(place, pp, zero)
        _, ok5 = K.ik(ret, pp, zero)
        self.q_carry[ids], self.q_rot[ids], self.q_bv[ids] = qc, qr, qv
        return ok1 & ok2 & ok3 & ok4 & ok5

    def _plan(self, ids: torch.Tensor):
        env = self._env_ref
        dev = self.device
        k = len(ids)
        self.pose_bias[ids] = torch.randn(k, 3, device=dev) * self._bias_sig
        self.pose_noise[ids] = torch.randn(k, 3, device=dev) * self._noise_sig
        gp = VC._grasp_point(env) - env.scene.env_origins                       # 정답 목표 TCP (팔 좌표)
        self._classify(ids, gp)
        gp_hat = gp[ids] + self.pose_bias[ids]                                   # IK 는 '인식된' 위치를 쓴다
        sh = env.scene["shelf"].data.root_pos_w.torch - env.scene.env_origins
        near, far = sh[ids, 0] - G.BOX[0] / 2, sh[ids, 0] + G.BOX[0] / 2
        # 실기 높이(팔 베이스 0.275 m)면 차체 앞부분 충돌 근사를 켬 (시뮬은 차체를 숨겼지만 실물에는 있음)
        keepout = tuple(K.CHASSIS_KEEPOUT) if G.ARM_Z > 0.1 else ()
        if OTHER_KEEPOUT:
            keepout = keepout + self._other_keepout(ids, gp_hat)
        pitch_req = None
        if TO_BIN:
            # 누운 퍽: 하향 90°(수직)로만 쥠 → 퍽 축 ⟂ 공구축 → 공구를 수평으로 세우면 퍽이 정확히 섬
            lie_i = self.lie[ids]
            pitch_req = torch.where(lie_i, torch.full((k,), math.pi / 2, device=dev), torch.full((k,), float("nan"), device=dev))
            r90, _ = self._roll_for(self.psi[ids], torch.full((k,), math.pi / 2, device=dev), lie_i, None)
            # 위로 갈 끝 = link5 +z 쪽 = cosφ·r̂ − sinφ·ŝ (하향 90°, 롤 φ) → 그쪽으로 쥐는 점을 옮김
            rad = gp_hat[:, :2] / gp_hat[:, :2].norm(dim=1, keepdim=True).clamp(min=1e-6)
            s_hat = torch.stack([-rad[:, 1], rad[:, 0]], dim=1)
            u = torch.cos(r90).unsqueeze(1) * rad - torch.sin(r90).unsqueeze(1) * s_hat
            gp_hat = gp_hat.clone()
            gp_hat[:, :2] += lie_i.float().unsqueeze(1) * BIN_GRASP_OFF_M * u
        pitch, wps, ok, _clear = K.choose_plan(gp_hat, gp_hat[:, 2], near, far, BOX_TOP, pitch_req=pitch_req,
                                               pitch_min_deg=PITCH_MIN_DEG, step_deg=PITCH_STEP_DEG, keepout=keepout)
        self.pitch[ids] = torch.nan_to_num(pitch, nan=0.0)
        if VIA_UP_M > 0.0:
            hi = wps[:, 0].clone()
            hi[:, 2] += VIA_UP_M
            _, ok_hi = K.ik(hi, self.pitch[ids])
            wps = wps.clone()
            wps[:, 0] = torch.where(ok_hi.unsqueeze(1), hi, wps[:, 0])
        if LIFT_UP_M > 0.0:
            lf = wps[:, 2].clone()
            lf[:, 2] += LIFT_UP_M
            _, ok_lf = K.ik(lf, self.pitch[ids])
            wps = wps.clone()
            wps[:, 3] = torch.where(ok_lf.unsqueeze(1), lf, wps[:, 3])
        self.wps[ids] = wps
        self.roll[ids], self.roll_mode[ids] = self._roll_for(self.psi[ids], self.pitch[ids], self.lie[ids],
                                                             None if TO_BIN else self.row[ids])   # 적재통: 항상 지름(30 mm 면) 쥠
        self.jaw_cos[ids] = 0.0
        q_via, okv = K.ik(wps[:, 0], self.pitch[ids], self.roll[ids])
        ok = ok & okv
        self.bin_ok[ids] = False
        self.bin_fail[ids] = False
        self.tilt_rec[ids] = 0.0
        self.verify_err[ids] = 0.0
        if TO_BIN:
            okb = self._plan_bin(ids, wps)
            self.n_bin_unreach += int((ok & ~okb).sum())
            ok = ok & okb
        self.q_via[ids] = q_via
        q0 = self._q_meas()[ids, :5]
        self.q_start[ids] = q0
        self.via_T[ids] = torch.clamp((q_via - q0).abs().amax(dim=1) / (0.6 * float(self._vmax[:5].min())), min=0.8)
        self.q_cmd[ids] = self._q_meas()[ids]
        self.q_int[ids] = 0.0
        self.q_cmd_prev[ids] = self._q_meas()[ids, :5]
        self.qm_prev[ids] = self._q_meas()[ids, :5]
        self.settle_t[ids] = 0.0
        self.wait_t[ids] = 0.0
        self.prev_tcp[ids] = K.fk_tcp(self._q_meas()[ids, :5])
        self.p_cmd[ids] = self.prev_tcp[ids]
        self.dip[ids] = 0.0
        self.dip_ph[ids] = 0
        self.vz_min[ids] = 0.0
        self.vz_ph[ids] = 0
        self.qint_close[ids] = 0.0
        self.res_off[ids] = 0.0
        self.res_cmd[ids] = 0.0
        self.s[ids] = 0.0
        self.t_ph[ids] = 0.0
        self.g_cmd[ids] = float(self._hi[5])
        self.close_err[ids] = 0.0
        self.close_rec[ids] = False
        self.ep_step[ids] = 0
        self.dist_step[ids] = -1
        self.phase[ids] = torch.where(ok, torch.full_like(ids, P_WAIT), torch.full_like(ids, P_SKIP))
        self.active[ids] = True
        self.n_skip += int((~ok).sum())

    # ------------------------------------------------------------------
    # 상태기계 (한 스텝)
    # ------------------------------------------------------------------
    def _seg(self, a: torch.Tensor, b: torch.Tensor, speed: float, track_ok: torch.Tensor):
        """직선 a→b 진행도 s 를 올리고, 그 점(+ 매 스텝 인식 잡음)의 IK 를 낸다"""
        length = torch.clamp((b - a).norm(dim=1), min=1e-4)
        self.s = torch.where(track_ok, torch.clamp(self.s + speed * self._dt / length, max=1.0), self.s)
        p = a + (b - a) * self.s.unsqueeze(1) + self.pose_noise
        self._last_p = p
        q, ok = K.ik(p, self.pitch, self.roll)
        return self.s >= 1.0 - 1e-6, q, ok

    def _watch_disturb(self):
        """목표·옆 퍽이 처음 흔들린(밀림·기울어짐) 순간을 기록. 들어 올린 목표 퍽은 제외"""
        env = self._env_ref
        self.ep_step += 1
        bad_t, pos_t, init_t = VC._disturbed(env, "target", WATCH_PUSH_M, WATCH_TILT_DEG)
        # 목표 퍽: 집기 단계(CLOSE 이후)의 움직임은 정상 → 그 전 단계에서만
        bad_t = bad_t & ((pos_t[:, 2] - init_t[:, 2]) < 0.01) & (self.phase < P_CLOSE)
        bad_o, _, _ = VC._disturbed(env, "other", WATCH_PUSH_M, WATCH_TILT_DEG)
        st = VC.vp(env)
        bad_o = bad_o & st.on_box.gather(1, (1 - st.color).unsqueeze(1)).squeeze(1)
        new = (bad_t | bad_o) & (self.dist_step < 0)
        if bool(new.any()):
            q = self._q_meas()[:, :5]
            pts = K.planar_points(q[:, 1], q[:, 2], q[:, 3])
            self.dist_step[new] = self.ep_step[new]
            self.dist_phase[new] = self.phase[new]
            self.dist_tcp[new] = K.fk_tcp(q)[new]
            self.dist_tip_z[new] = pts["TIP"][1][new]
            self.dist_which[new] = torch.where(bad_t, 1, 2)[new]
            col = torch.where(bad_t, st.color, 1 - st.color).unsqueeze(1)
            kind = st.lying.gather(1, col).squeeze(1).long() * 2 + st.shape.gather(1, col).squeeze(1)
            self.dist_kind[new] = kind[new]

    def _dist_stats(self, done: torch.Tensor):
        st = self.dist_step[done].tolist()
        ph = self.dist_phase[done].tolist()
        tcp = self.dist_tcp[done].tolist()
        tz = self.dist_tip_z[done].tolist()
        wh = self.dist_which[done].tolist()
        kd = self.dist_kind[done].tolist()
        kn = ["세운사각", "세운원기둥", "누운사각", "누운원기둥"]
        for j in range(len(st)):
            if st[j] < 0:
                continue
            key = f"{PHASE_NAMES[ph[j]]}/{'목표' if wh[j] == 1 else '옆퍽'}/{kn[kd[j]]}"
            r = math.hypot(tcp[j][0], tcp[j][1])
            self.dist_log.setdefault(key, []).append((st[j], r, tcp[j][2], tz[j]))

    def _dist_report(self) -> dict:
        out = {}
        print(f"[NZ2-RES] ===== 퍽이 처음 흔들린 순간 [밀림 {WATCH_PUSH_M * 1000:.0f} mm 또는 기울기 {WATCH_TILT_DEG:.0f}°] (단계/어느 퍽: 횟수, 스텝 평균·최소, 손끝 반경·TCP z·손가락 끝 z 평균) =====")
        for k in sorted(self.dist_log, key=lambda x: -len(self.dist_log[x])):
            v = self.dist_log[k]
            n = len(v)
            m = [sum(x[i] for x in v) / n for i in range(4)]
            mn = min(x[0] for x in v)
            early = sum(1 for x in v if x[0] <= 3)
            print(f"[NZ2-RES]  {k}: {n}회, 스텝 평균 {m[0]:.0f} (최소 {mn}, 3스텝 이내 {early}회) | 반경 {m[1] * 1000:.0f} mm, "
                  f"TCP z {m[2] * 1000:.0f} mm, 손가락 끝 z {m[3] * 1000:.0f} mm")
            out[k] = {"n": n, "step_mean": round(m[0], 1), "step_min": mn, "within3": early, "r_mm": round(m[1] * 1000),
                      "tcp_z_mm": round(m[2] * 1000), "tip_z_mm": round(m[3] * 1000)}
        return out

    def _enter_jmove(self, mask: torch.Tensor, q_target: torch.Tensor):
        """관절 보간 시작: 시작점 = 현재 명령각, 시간 = 가장 많이 움직이는 관절 / (최대속도 × JMOVE_VFRAC)"""
        if not bool(mask.any()):
            return
        q0 = self.q_cmd[:, :5]
        T = (q_target - q0).abs().amax(dim=1) / (JMOVE_VFRAC * float(self._vmax[:5].min()))
        self.q_start[mask] = q0[mask]
        self.jT[mask] = torch.clamp(T, min=0.5)[mask]

    def _jmove(self, m: torch.Tensor, q_target: torch.Tensor) -> torch.Tensor:
        """smoothstep 관절 보간. 반환: 경로 끝 도달 (m 안에서)"""
        u = torch.clamp(self.t_ph / self.jT, 0.0, 1.0)
        u = u * u * (3.0 - 2.0 * u)
        q = self.q_start + (q_target - self.q_start) * u.unsqueeze(1)
        self.q_cmd[m, :5] = q[m]
        return m & (self.t_ph >= self.jT)

    def _line(self, m: torch.Tensor, a: torch.Tensor, b: torch.Tensor, speed: float, track_ok: torch.Tensor):
        """직선 a→b (자세 = 놓기 하향각, 롤 0). 반환 (끝 도달, IK ok)"""
        length = torch.clamp((b - a).norm(dim=1), min=1e-4)
        self.s = torch.where(m & track_ok, torch.clamp(self.s + speed * self._dt / length, max=1.0), self.s)
        p = a + (b - a) * self.s.unsqueeze(1)
        q, ok = K.ik(p, self.place_pitch, torch.zeros_like(self.place_pitch))
        good = m & ok
        self.q_cmd[good, :5] = q[good]
        self._last_p = torch.where(m.unsqueeze(1), p, self._last_p)
        return m & (self.s >= 1.0 - 1e-6) & ok, ok

    def _step_bin(self, to, settled, q_meas, track_ok, tcp_now):
        """(11/11) 적재통 단계 (LIFT 뒤). 팔 좌표 = 환경 원점 기준"""
        env = self._env_ref
        dt = self._dt
        # LIFT 에서 막 넘어온 환경: CARRY 보간 시작 (to() 가 t_ph = 0 으로 만든 직후)
        m = (self.phase == P_CARRY) & (self.t_ph == 0.0)           # to() 직후 (같은 스텝)
        self._enter_jmove(m, self.q_carry)
        # CARRY → ALIGN
        m = self.phase == P_CARRY
        if m.any():
            nx = self._jmove(m, self.q_carry)
            to(P_ALIGN, nx)
            if bool(nx.any()):
                self.q_start[nx] = self.q_cmd[nx, :5]
                self.jT[nx] = torch.clamp((self.q_cmd[:, 4] - 0.0).abs() / ALIGN_RAD_S, min=0.3)[nx]
        # ALIGN (j5 만) → TILT
        m = self.phase == P_ALIGN
        if m.any():
            q_al = self.q_start.clone()
            q_al[:, 4] = 0.0
            go = self._jmove(m, q_al)
            to(P_TILT, go)
            self._enter_jmove(go, self.q_rot)
        # TILT (집은 방향, 경유점 높이, 놓기 하향각) → 정지 확인 → ROTATE
        m = self.phase == P_TILT
        if m.any():
            end = self._jmove(m, self.q_rot)
            go = settled(end, "tilt") if SETTLE else end
            if bool(go.any()):
                pos_t, quat_t, _, _ = VC._color_pose(env, "target")
                pos_t = pos_t - env.scene.env_origins
                ax = math_utils.quat_apply(quat_t, torch.tensor([0.0, 0.0, 1.0], device=self.device).expand_as(pos_t))
                # 누운 퍽은 자기 z 축이 수평 → 퍽 긴 축 = 퍽 몸체 z (직육면체 60 mm 축)
                self.tilt_rec[go, 0] = ax[go, 2].abs()
                self.tilt_rec[go, 1] = (tcp_now - pos_t)[go, 2]
                self.tilt_rec[go, 2] = 1.0
            to(P_ROTATE, go)
            self._enter_jmove(go, self.q_bv)
        # ROTATE (j1 만, 같은 자세) → 정지 확인 → BIN_DESC
        m = self.phase == P_ROTATE
        if m.any():
            end = self._jmove(m, self.q_bv)
            go = settled(end, "bin") if SETTLE else end
            to(P_BIN_DESC, go)
        # BIN_DESC: 컵 위 → 놓는 점 직선 → 정지 확인 → RELEASE
        m = self.phase == P_BIN_DESC
        if m.any():
            end, okl = self._line(m, self.bin_wps[:, 0], self.bin_wps[:, 1], V_BIN, track_ok)
            bad = m & ~okl
            self.phase[bad] = P_SKIP
            self.n_ikfail += int(bad.sum())
            go = settled(end, "place") if SETTLE else end
            to(P_RELEASE, go)
        # RELEASE: 그리퍼를 RELEASE_FRAC 까지 엶 (손가락이 옆 컵 퍽을 치지 않게 끝까지 열지 않음)
        m = self.phase == P_RELEASE
        if m.any():
            span = float(self._hi[5] - self._lo[5])
            g_rel = float(self._lo[5]) + RELEASE_FRAC * span
            self.g_cmd = torch.where(m, torch.clamp(self.g_cmd + span * dt / RELEASE_S, max=g_rel), self.g_cmd)
            to(P_RETREAT, m & (self.t_ph >= RELEASE_S))
        # RETREAT: 수직 위로
        m = self.phase == P_RETREAT
        if m.any():
            end, okl = self._line(m, self.bin_wps[:, 1], self.bin_wps[:, 2], V_RETREAT, track_ok)
            bad = m & ~okl
            self.phase[bad] = P_SKIP
            self.n_ikfail += int(bad.sum())
            to(P_VERIFY, end)
        # VERIFY: VERIFY_S 동안 가만히 → 퍽 위치·자세 판정
        m = (self.phase == P_VERIFY) & (self.t_ph >= VERIFY_S)
        if bool(m.any()):
            pos_t, quat_t, _, _ = VC._color_pose(env, "target")
            pos_t = pos_t - env.scene.env_origins
            ax = math_utils.quat_apply(quat_t, torch.tensor([0.0, 0.0, 1.0], device=self.device).expand_as(pos_t))
            # 세운 퍽의 긴 축(몸체 z) 이 수직이어야 함 → |z 성분|
            tilt = torch.rad2deg(torch.acos(ax[:, 2].abs().clamp(max=1.0)))
            exy = (pos_t[:, :2] - self.bin_center[:, :2]).norm(dim=1)
            ez = (pos_t[:, 2] - self.bin_center[:, 2]).abs()
            ok = (exy < BIN_XY_TOL) & (ez < BIN_Z_TOL) & (tilt < BIN_TILT_DEG)
            self.verify_err[m] = torch.stack([exy, ez, tilt], dim=1)[m]
            self.bin_ok |= m & ok
            self.bin_fail |= m & ~ok
            to(P_DONE, m)

    def _step_machine(self):
        dt = self._dt
        q_meas = self._q_meas()
        self.t_ph += dt
        # 인식 잡음 갱신 (AR(1), 정상상태 σ = POSE_NOISE_M)
        a_n = self._noise_a
        self.pose_noise = a_n * self.pose_noise + math.sqrt(max(1.0 - a_n * a_n, 0.0)) * self._noise_sig * torch.randn_like(self.pose_noise)
        track_err = (self.q_cmd[:, :5] - q_meas[:, :5]).abs().amax(dim=1)
        track_ok = track_err < TRACK_GATE_RAD
        # (11/11 오후) 정지 판정: 측정 관절속도(팔 5축 최대) + 작은 추종 오차가 연속 SETTLE_HOLD_S
        qd_meas = (q_meas[:, :5] - self.qm_prev).abs().amax(dim=1) / dt
        self.qm_prev[:] = q_meas[:, :5]
        at_rest = (track_err < SETTLE_ERR_RAD) & (qd_meas < SETTLE_QD_RAD_S)
        self.settle_t = torch.where(at_rest, self.settle_t + dt, torch.zeros_like(self.settle_t))
        via, pre, g, lift = self.wps[:, 0], self.wps[:, 1], self.wps[:, 2], self.wps[:, 3]
        # 손끝 실제 위치 (FK 자기검사 0.1 mm 이므로 FK(실제 관절) = 시뮬 TCP)
        tcp_now = K.fk_tcp(q_meas[:, :5])
        g_now = g + self.pose_noise
        # (11/11) 내려감 진단: DESCEND·APPROACH·CLOSE 동안
        watch = (self.phase >= P_DESCEND) & (self.phase <= P_CLOSE)
        dz = tcp_now[:, 2] - self.p_cmd[:, 2]
        vz = (tcp_now[:, 2] - self.prev_tcp[:, 2]) / dt
        nd = watch & (dz < self.dip)
        self.dip[nd] = dz[nd]
        self.dip_ph[nd] = self.phase[nd]
        nv = watch & (vz < self.vz_min)
        self.vz_min[nv] = vz[nv]
        self.vz_ph[nv] = self.phase[nv]
        self.prev_tcp[:] = tcp_now
        e_vec = tcp_now - g_now
        if CLOSE_GATE_M > 0.0:
            seg_T = (g - pre).norm(dim=1) / V_APPROACH
            cart_ok = (e_vec.norm(dim=1) < CLOSE_GATE_M) | (self.t_ph >= seg_T + CLOSE_WAIT_S)
        else:
            cart_ok = torch.ones_like(track_ok)

        def to(new: int, mask: torch.Tensor):
            self.phase[mask] = new
            self.t_ph[mask] = 0.0
            self.s[mask] = 0.0
            self.wait_t[mask] = 0.0

        def settled(cand: torch.Tensor, key: str) -> torch.Tensor:
            """cand(경로 끝 도달) 인 환경 중 정지 확인 통과(또는 시간 초과)한 것"""
            self.wait_t = torch.where(cand, self.wait_t + dt, self.wait_t)
            ok_s = cand & (self.settle_t >= SETTLE_HOLD_S)
            to_s = cand & ~ok_s & (self.wait_t >= SETTLE_MAX_S)
            go_s = ok_s | to_s
            if bool(go_s.any()):
                self.n_settle[key] += int(go_s.sum())
                self.n_settle_to[key] += int(to_s.sum())
                self.settle_wait_sum[key] += float(self.wait_t[go_s].sum())
            return go_s

        m = (self.phase == P_WAIT) & (self.t_ph >= WAIT_S)
        PD.dump(self._env_ref, m, "look")
        self.q_start[m] = q_meas[m, :5]
        to(P_TO_VIA, m)
        m = self.phase == P_TO_VIA
        if m.any():
            u = torch.clamp(self.t_ph / self.via_T, 0.0, 1.0)
            u = u * u * (3.0 - 2.0 * u)
            qv = self.q_start + (self.q_via - self.q_start) * u.unsqueeze(1)
            self.q_cmd[m, :5] = qv[m]
            self.p_cmd[m] = self.wps[m, 0]                                      # (11/11) 진단 기준 = via
            if SETTLE:
                g_via = settled(m & (self.t_ph >= self.via_T), "via")
            else:
                g_via = m & (self.t_ph >= self.via_T) & (track_err < 0.05)
            PD.dump(self._env_ref, g_via, "via")
            to(P_DESCEND, g_via)
        for phase, a, b, spd, nxt in ((P_DESCEND, via, pre, V_DESCEND, P_APPROACH),
                                      (P_APPROACH, pre, g, V_APPROACH, P_CLOSE),
                                      (P_LIFT, g, lift, V_LIFT, LIFT_NEXT)):
            m = self.phase == phase
            if not m.any():
                continue
            fin, q, ok = self._seg(a, b, spd, track_ok)
            good = m & ok
            self.q_cmd[good, :5] = q[good]
            self.p_cmd[good] = self._last_p[good]
            bad = m & ~ok                                                        # IK 실패: 그 자리 정지 + SKIP
            self.phase[bad] = P_SKIP
            self.n_ikfail += int(bad.sum())
            if SETTLE and phase == P_DESCEND:
                go = settled(m & fin & ok, "pre")
            else:
                go = m & fin & ok & (track_err < 0.03)
            if phase == P_DESCEND:
                PD.dump(self._env_ref, go, "pre")
            if phase == P_APPROACH:
                go = go & cart_ok
                # CLOSE 진입 순간 손끝 추종 오차 기록 (z: 양수 = 목표보다 위, 수평 거리)
                self.close_err[go, 0] = e_vec[go, 2]
                self.close_err[go, 1] = e_vec[go, :2].norm(dim=1)
                self.qint_close[go] = self.q_int[go, 1:4].abs().amax(dim=1)
                self.close_rec[go] = True
                if bool(go.any()):
                    jaw = self._jaw_axis()
                    if jaw is not None:
                        quat_t = VC._color_pose(self._env_ref, "target")[1]
                        ax = math_utils.quat_apply(quat_t, torch.tensor([0.0, 0.0, 1.0], device=self.device).expand_as(jaw))
                        self.jaw_cos[go] = (jaw * ax).sum(dim=1).abs()[go]
            to(nxt, go)
            if phase == P_DESCEND and REFINE_M >= 0.0 and bool(go.any()):
                ids = go.nonzero().squeeze(-1)
                sig = torch.tensor([REFINE_M, REFINE_M, min(REFINE_M, POSE_BIAS_Z_M)], device=self.device)
                new_b = torch.randn(len(ids), 3, device=self.device) * sig
                d = new_b - self.pose_bias[ids]
                g_new = self.wps[ids, 2] + d
                _, ok_g = K.ik(g_new, self.pitch[ids], self.roll[ids])
                ids = ids[ok_g]
                d = d[ok_g]
                self.wps[ids, 2] += d
                self.wps[ids, 3] += d
                self.pose_bias[ids] += d
        m = self.phase == P_CLOSE
        if m.any():
            span = float(self._hi[5] - self._lo[5])
            self.g_cmd = torch.where(m, torch.clamp(self.g_cmd - span * dt / CLOSE_S, min=float(self._lo[5])), self.g_cmd)
            to(P_LIFT, m & (self.t_ph >= CLOSE_S))
        to(P_DONE, (self.phase == P_HOLD) & (self.t_ph >= HOLD_S))
        if TO_BIN:
            self._step_bin(to, settled, q_meas, track_ok, tcp_now)
        self.q_cmd[:, 5] = self.g_cmd

    # ------------------------------------------------------------------
    # 행동 처리
    # ------------------------------------------------------------------
    def process_actions(self, actions: torch.Tensor):
        env = self._env_ref
        if not self._checked:
            self._fk_self_check()
        new_ep = env.episode_length_buf == 0
        if bool(new_ep.any()):
            ids = new_ep.nonzero().squeeze(-1)
            self._stats(ids)
            self._plan(ids)
        self._step_machine()
        self._watch_disturb()
        # 잔차: tanh(π) (팔 5축) × gate → 오프셋 적분
        pi = torch.nan_to_num(actions[:, :5].float(), nan=0.0)
        gate = torch.zeros_like(self.phase, dtype=torch.bool)
        for p in RES_PHASES:
            gate |= self.phase == p
        r = torch.tanh(pi) * gate.unsqueeze(1).float()
        self.res_cmd = r
        if self.alpha > 0.0:
            self.res_off = torch.clamp(self.res_off + self.alpha * r * self._max_step[:, :5], -RES_MAX_RAD, RES_MAX_RAD)
        # 처짐 보상
        q_meas = self._q_meas()
        moving = (((self.phase >= P_TO_VIA) & (self.phase <= P_HOLD)) | (self.phase >= P_CARRY)).unsqueeze(1)
        err = self.q_cmd[:, :5] - q_meas[:, :5]
        if SAG_MODE == "static":
            qd_cmd = (self.q_cmd[:, :5] - self.q_cmd_prev).abs().amax(dim=1) / self._dt
            moving = moving & (qd_cmd < SAG_STATIC_RAD_S).unsqueeze(1)
        self.q_cmd_prev = self.q_cmd[:, :5].clone()
        self.q_int = torch.clamp(self.q_int + SAG_KI * err * self._dt * moving, -SAG_MAX, SAG_MAX)
        goal = self.q_cmd.clone()
        goal[:, :5] += self.q_int + self.res_off
        goal = torch.maximum(torch.minimum(goal, self._hi), self._lo)
        a = torch.clamp((goal - self._prev_target) / self._max_step, -1.0, 1.0)
        stop = (self.phase == P_DONE) | (self.phase == P_SKIP)
        a = torch.where(stop.unsqueeze(1), torch.zeros_like(a), a)
        super().process_actions(a)

    # ------------------------------------------------------------------
    # 통계 (expert-check)
    # ------------------------------------------------------------------
    @staticmethod
    def _pitch_bin(deg: float) -> str:
        for lo, hi, name in PITCH_BINS:
            if lo <= deg < hi:
                return name
        return "하향 기타"

    def _cond_stats(self, done: torch.Tensor, succ: torch.Tensor, terms: dict[str, torch.Tensor]):
        """끝난 에피소드를 조건(종류·자세 × 옆퍽 줄)별로 집계"""
        kind = self.kind[done].tolist()
        row = self.row[done].tolist()
        ok = succ[done].tolist()
        ph = self.phase[done].tolist()
        pitch = torch.rad2deg(self.pitch[done]).tolist()
        rec = self.close_rec[done].tolist()
        mode = self.roll_mode[done].tolist()
        jc = self.jaw_cos[done].tolist()
        rl = torch.rad2deg(self.roll[done]).abs().tolist()
        cerr = self.close_err[done].tolist()
        tv = {k: v.tolist() for k, v in terms.items()}
        for j in range(len(kind)):
            key = f"{KIND_NAMES[kind[j]]} / {ROW_NAMES[row[j]]}"
            d = self.by_cond.setdefault(key, {"n": 0, "ok": 0, "fail_phase": {}, "fail_reason": {},
                                              "close_n": 0, "close_ez_sum": 0.0, "close_ez_min": 1.0, "close_exy_sum": 0.0})
            d["n"] += 1
            d["ok"] += int(ok[j])
            if not ok[j]:
                pn = PHASE_NAMES[ph[j]]
                d["fail_phase"][pn] = d["fail_phase"].get(pn, 0) + 1
                for name, vals in tv.items():
                    if name != "success" and vals[j]:
                        d["fail_reason"][name] = d["fail_reason"].get(name, 0) + 1
            if rec[j]:
                d["close_n"] += 1
                d["close_ez_sum"] += cerr[j][0]
                d["close_ez_min"] = min(d["close_ez_min"], cerr[j][0])
                d["close_exy_sum"] += cerr[j][1]
            mv = self.by_mode.setdefault(MODE_NAMES[mode[j]], [0, 0, 0.0, 0, 0.0])   # n, ok, |롤|합, CLOSE n, |cos|합
            mv[0] += 1
            mv[1] += int(ok[j])
            mv[2] += rl[j]
            if rec[j]:
                mv[3] += 1
                mv[4] += jc[j]
            pb = self._pitch_bin(pitch[j]) if ph[j] != P_SKIP else "SKIP(계획 실패)"
            v = self.by_pitch.setdefault(pb, [0, 0])
            v[0] += 1
            v[1] += int(ok[j])

    def _dip_stats(self, done: torch.Tensor, succ: torch.Tensor):
        """(11/11) 퍽 자세별: 손끝이 명령 경로보다 아래로 내려간 최대량·단계, 최대 하강 속도, CLOSE 때 q_int"""
        lie = self.lie[done].tolist()
        dip = self.dip[done].tolist()
        dph = self.dip_ph[done].tolist()
        vz = self.vz_min[done].tolist()
        vph = self.vz_ph[done].tolist()
        qi = self.qint_close[done].tolist()
        rec = self.close_rec[done].tolist()
        ok = succ[done].tolist()
        for j in range(len(lie)):
            key = "누운 퍽" if lie[j] else "세운 퍽"
            d = self.by_dip.setdefault(key, {"n": 0, "dip_sum": 0.0, "dip_min": 0.0, "warn": 0, "warn_fail": 0, "dip_phase": {},
                                             "vz_sum": 0.0, "vz_min": 0.0, "vz_phase": {}, "qi_n": 0, "qi_sum": 0.0, "qi_max": 0.0})
            d["n"] += 1
            d["dip_sum"] += dip[j]
            d["dip_min"] = min(d["dip_min"], dip[j])
            if dip[j] < -DIP_WARN_M:
                d["warn"] += 1
                d["warn_fail"] += int(not ok[j])
                pn = PHASE_NAMES[dph[j]]
                d["dip_phase"][pn] = d["dip_phase"].get(pn, 0) + 1
            d["vz_sum"] += vz[j]
            d["vz_min"] = min(d["vz_min"], vz[j])
            pn = PHASE_NAMES[vph[j]]
            d["vz_phase"][pn] = d["vz_phase"].get(pn, 0) + 1
            if rec[j]:
                d["qi_n"] += 1
                d["qi_sum"] += qi[j]
                d["qi_max"] = max(d["qi_max"], qi[j])

    def _dip_report(self) -> dict:
        out = {}
        print(f"[NZ2-RES] ===== 내려감 진단 (DESCEND~CLOSE, 손끝 z − 명령 경로 z, 음수 = 경로 아래 / 처짐 보상 ki {SAG_KI}, 모드 {SAG_MODE}) =====")
        for key in sorted(self.by_dip):
            d = self.by_dip[key]
            n = max(1, d["n"])
            qn = max(1, d["qi_n"])
            out[key] = {"n": d["n"], "dip_mean_mm": round(1000 * d["dip_sum"] / n, 2), "dip_min_mm": round(1000 * d["dip_min"], 2),
                        "dip_over_3mm": d["warn"], "dip_over_3mm_fail": d["warn_fail"], "dip_phase": d["dip_phase"],
                        "vz_mean_mms": round(1000 * d["vz_sum"] / n, 1), "vz_min_mms": round(1000 * d["vz_min"], 1),
                        "vz_phase": d["vz_phase"], "qint_close_mean_deg": round(math.degrees(d["qi_sum"] / qn), 2),
                        "qint_close_max_deg": round(math.degrees(d["qi_max"]), 2)}
            o = out[key]
            print(f"[NZ2-RES]  {key} (n={d['n']}): 경로 아래 최대 평균 {o['dip_mean_mm']:+.1f} mm (최저 {o['dip_min_mm']:+.1f}), "
                  f"3 mm 넘게 아래 {d['warn']}회 (그중 실패 {d['warn_fail']}) 단계 {d['dip_phase']} | "
                  f"최대 하강속도 평균 {o['vz_mean_mms']:.0f} mm/s (최저 {o['vz_min_mms']:.0f}) 단계 {d['vz_phase']} | "
                  f"CLOSE 때 |q_int| 평균 {o['qint_close_mean_deg']:.1f}° (최대 {o['qint_close_max_deg']:.1f}°)")
        return out

    def _cond_report(self) -> dict:
        out = {}
        print("[NZ2-RES] ===== 조건별 분해 (집게 닫힘 = 반경에 수직인 수평 방향) =====")
        for key in sorted(self.by_cond):
            d = self.by_cond[key]
            rate = 100.0 * d["ok"] / max(1, d["n"])
            cn = max(1, d["close_n"])
            ez = 1000.0 * d["close_ez_sum"] / cn
            exy = 1000.0 * d["close_exy_sum"] / cn
            ezmin = 1000.0 * d["close_ez_min"] if d["close_n"] else float("nan")
            fp = ", ".join(f"{k} {v}" for k, v in sorted(d["fail_phase"].items(), key=lambda x: -x[1]))
            fr = ", ".join(f"{k} {v}" for k, v in sorted(d["fail_reason"].items(), key=lambda x: -x[1]))
            print(f"[NZ2-RES]  {key}: {d['ok']}/{d['n']} ({rate:.0f} %) | CLOSE 진입 손끝 z오차 평균 {ez:+.1f} mm(최저 {ezmin:+.1f}), "
                  f"수평 {exy:.1f} mm (n={d['close_n']})")
            if fp:
                print(f"[NZ2-RES]      실패 단계: {fp}")
            if fr:
                print(f"[NZ2-RES]      실패 이유: {fr}")
            out[key] = {"n": d["n"], "ok": d["ok"], "rate_pct": round(rate, 1), "fail_phase": d["fail_phase"],
                        "fail_reason": d["fail_reason"], "close_n": d["close_n"], "close_ez_mean_mm": round(ez, 2),
                        "close_ez_min_mm": round(ezmin, 2), "close_exy_mean_mm": round(exy, 2)}
        pit = {}
        for k in sorted(self.by_pitch):
            n, ok = self.by_pitch[k]
            pit[k] = {"n": n, "ok": ok, "rate_pct": round(100.0 * ok / max(1, n), 1)}
        print("[NZ2-RES]  하향각별: " + ", ".join(f"{k} {v['ok']}/{v['n']} ({v['rate_pct']:.0f} %)" for k, v in pit.items()))
        md = {}
        for k in sorted(self.by_mode):
            n, ok, rsum, cn, csum = self.by_mode[k]
            md[k] = {"n": n, "ok": ok, "rate_pct": round(100.0 * ok / max(1, n), 1), "roll_abs_mean_deg": round(rsum / max(1, n), 1),
                     "jaw_cos_mean": round(csum / cn, 3) if cn else None}
            jtxt = f"{md[k]['jaw_cos_mean']:.2f}" if cn else "-"
            print(f"[NZ2-RES]  롤 모드 {k}: {ok}/{n} ({md[k]['rate_pct']:.0f} %), |롤| 평균 {md[k]['roll_abs_mean_deg']:.0f}°, "
                  f"CLOSE 시 |cos(집게축, 퍽축)| {jtxt} (n={cn})")
        return {"by_cond": out, "by_pitch": pit, "by_roll_mode": md}

    def _bin_stats(self, done: torch.Tensor, succ: torch.Tensor):
        """(11/11) 적재통: 끝난 단계·VERIFY 오차·TILT 자기검사 누적"""
        b = getattr(self, "_bin_log", None)
        if b is None:
            b = self._bin_log = {"end_phase": {}, "verify": [], "tilt": [], "slot": {}}
        st = VC.vp(self._env_ref)
        ph = self.phase[done].tolist()
        ok = succ[done].tolist()
        slot = st.bin_slot[done].tolist()
        ve = self.verify_err[done].tolist()
        tr = self.tilt_rec[done].tolist()
        vf = self.bin_fail[done].tolist()
        for j in range(len(ph)):
            key = "성공" if ok[j] else PHASE_NAMES[ph[j]]
            b["end_phase"][key] = b["end_phase"].get(key, 0) + 1
            sl = b["slot"].setdefault(str(slot[j]), [0, 0])
            sl[0] += 1
            sl[1] += int(ok[j])
            if ok[j] or vf[j]:
                b["verify"].append((ve[j][0], ve[j][1], ve[j][2], int(ok[j])))
            if tr[j][2] > 0:
                b["tilt"].append((tr[j][0], tr[j][1]))

    def _bin_report(self) -> dict:
        b = getattr(self, "_bin_log", {"end_phase": {}, "verify": [], "tilt": [], "slot": {}})
        out = {"end_phase": b["end_phase"], "unreachable_bin": self.n_bin_unreach,
               "slot": {k: {"n": v[0], "ok": v[1]} for k, v in sorted(b["slot"].items())},
               "floor_z": VC.BIN_FLOOR_Z, "grasp_off_mm": BIN_GRASP_OFF_M * 1000, "carry": [CARRY_R, CARRY_Z]}
        print(f"[NZ2-BIN] ===== 적재통 결과 (계획 단계 컵 도달 불가 {self.n_bin_unreach}) =====")
        print(f"[NZ2-BIN]  끝난 단계(실패) / 성공: {b['end_phase']}")
        print("[NZ2-BIN]  칸별 성공: " + ", ".join(f"{k}: {v[1]}/{v[0]}" for k, v in sorted(b["slot"].items())))
        v = b["verify"]
        if v:
            n = len(v)
            m = [sum(x[i] for x in v) / n for i in range(3)]
            mx = [max(x[i] for x in v) for i in range(3)]
            fx = sum(1 for x in v if not x[3] and x[0] >= BIN_XY_TOL)
            fz = sum(1 for x in v if not x[3] and x[1] >= BIN_Z_TOL)
            ft = sum(1 for x in v if not x[3] and x[2] >= BIN_TILT_DEG)
            print(f"[NZ2-BIN]  VERIFY 실패 원인(중복 가능): xy ≥ {BIN_XY_TOL * 1000:.0f} mm {fx}, z ≥ {BIN_Z_TOL * 1000:.0f} mm {fz}, "
                  f"기울기 ≥ {BIN_TILT_DEG:.0f}° {ft}")
            print(f"[NZ2-BIN]  VERIFY {n}회: xy 오차 평균 {m[0] * 1000:.1f} (최대 {mx[0] * 1000:.1f}) mm, z 평균 {m[1] * 1000:.1f} mm, "
                  f"기울기 평균 {m[2]:.1f}° (최대 {mx[2]:.1f}°)")
            out["verify_fail"] = {"xy": fx, "z": fz, "tilt": ft}
            out["verify"] = {"n": n, "xy_mean_mm": round(m[0] * 1000, 1), "xy_max_mm": round(mx[0] * 1000, 1),
                             "z_mean_mm": round(m[1] * 1000, 1), "tilt_mean_deg": round(m[2], 1), "tilt_max_deg": round(mx[2], 1)}
        t = b["tilt"]
        if t:
            n = len(t)
            c = sum(x[0] for x in t) / n
            dz = sum(x[1] for x in t) / n
            up = sum(1 for x in t if x[1] > 0) / n
            print(f"[NZ2-BIN]  TILT 끝 {n}회: |퍽축·z| 평균 {c:.3f} (1 = 섬), TCP − 퍽 중심 z 평균 {dz * 1000:+.1f} mm "
                  f"(기대 +{BIN_GRASP_OFF_M * 1000:.0f}, 양수 비율 {up:.2f} → 쥐는 점이 위쪽 끝인지)")
            out["tilt"] = {"n": n, "axis_z_mean": round(c, 3), "tcp_minus_center_z_mm": round(dz * 1000, 1), "tcp_above_frac": round(up, 2)}
        if TO_BIN:
            for k_ in ("tilt", "bin", "place"):
                n_s = max(self.n_settle[k_], 1)
                out[f"settle_{k_}"] = {"n": self.n_settle[k_], "timeout": self.n_settle_to[k_],
                                       "wait_mean_s": round(self.settle_wait_sum[k_] / n_s, 3)}
        return out

    def _stats(self, ids: torch.Tensor):
        env = self._env_ref
        try:
            succ = env.termination_manager.get_term("success")
        except Exception:
            succ = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        done = ids[self.active[ids]]
        self.active[ids] = False
        if len(done) == 0:
            return
        before = self.n_done
        self.n_done += len(done)
        self.n_ok += int(succ[done].sum())
        # 끝난 이유별 집계 (실패 원인 분석)
        tm = env.termination_manager
        terms: dict[str, torch.Tensor] = {}
        for name in tm.active_terms:
            try:
                t = tm.get_term(name)[done]
            except Exception:
                continue
            terms[name] = t
            self.term_count[name] = self.term_count.get(name, 0) + int(t.sum())
        skip_now = int((self.phase[done] == P_SKIP).sum())
        self.term_count["(SKIP 상태로 끝남)"] = self.term_count.get("(SKIP 상태로 끝남)", 0) + skip_now
        for p in range(NUM_PHASES):
            k = f"끝난 단계:{PHASE_NAMES[p]}"
            self.term_count[k] = self.term_count.get(k, 0) + int((self.phase[done] == p).sum())
        self._cond_stats(done, succ, terms)
        self._dip_stats(done, succ)
        self._dist_stats(done)
        if TO_BIN:
            self._bin_stats(done, succ)
        if self.n_done // 64 != before // 64:
            print(f"[NZ2-RES] 에피소드 {self.n_done}: 성공 {self.n_ok} ({100 * self.n_ok / self.n_done:.0f} %) / "
                  f"도달불가 {self.n_skip} / 중간 IK 실패 {self.n_ikfail}")
            print("[NZ2-RES]   끝난 이유: " + ", ".join(f"{k} {v}" for k, v in self.term_count.items() if v))
        if self.max_eps and self.n_done >= self.max_eps:
            rate = 100 * self.n_ok / self.n_done
            res = {"episodes": self.n_done, "success": self.n_ok, "rate_pct": round(rate, 1), "unreachable": self.n_skip,
                   "ik_fail_midway": self.n_ikfail, "alpha": self.alpha, "pose_bias_mm": POSE_BIAS_M * 1000,
                   "pose_noise_mm": POSE_NOISE_M * 1000, "pose_bias_z_mm": POSE_BIAS_Z_M * 1000, "pose_noise_z_mm": POSE_NOISE_Z_M * 1000, "close_gate_mm": CLOSE_GATE_M * 1000, "refine_bias_mm": REFINE_M * 1000,
                   "other_keepout": OTHER_KEEPOUT, "other_keep_mm": OTHER_KEEP_M * 1000, "lie_grasp_up_mm": VC.LIE_GRASP_UP * 1000,
                   "stage": int(VC.vp(env).stage), "kin_j1_z": K.J1_Z,
                   "arm_z": G.ARM_Z, "kin_source": K.DIMS_SOURCE, "kin_fk_vs_urdf_mm": K.KIN_INFO.get("fk_vs_urdf_max_mm"),
                   "end_reasons": {k: v for k, v in self.term_count.items() if v}}
            res.update(self._cond_report())
            res["disturb_first"] = self._dist_report()
            res["sag_ki"], res["sag_mode"] = SAG_KI, SAG_MODE
            res["settle"] = {"on": SETTLE, "err_rad": SETTLE_ERR_RAD, "qd_rad_s": SETTLE_QD_RAD_S, "hold_s": SETTLE_HOLD_S,
                             "max_s": SETTLE_MAX_S}
            for k in ("via", "pre"):
                n_s = max(self.n_settle[k], 1)
                res["settle"][k] = {"n": self.n_settle[k], "timeout": self.n_settle_to[k],
                                    "wait_mean_s": round(self.settle_wait_sum[k] / n_s, 3)}
                if SETTLE:
                    print(f"[NZ2-RES] 정지 게이트 {k}: {self.n_settle[k]}회, 시간 초과 {self.n_settle_to[k]}회, "
                          f"평균 대기 {self.settle_wait_sum[k] / n_s:.2f} s")
            res["dip"] = self._dip_report()
            if TO_BIN:
                res["bin"] = self._bin_report()
            os.makedirs(os.path.join("logs", "nz2_vispick"), exist_ok=True)
            path = os.path.join("logs", "nz2_vispick", "expert_check_" + time.strftime("%Y%m%d_%H%M%S") + ".json")
            with open(path, "w", encoding="utf-8") as f:
                json.dump(res, f, ensure_ascii=False, indent=2)
            print(f"[NZ2-RES] 완료: 성공 {self.n_ok}/{self.n_done} ({rate:.1f} %) → {os.path.abspath(path)}")
            os._exit(0)
