"""NZ2 바닥 고정 팔 + 카메라 영상 end-to-end RL 집기 (Isaac Lab 3.0 / Isaac Sim 6.1 / PhysX)

설치 위치: ~/manipulator-isaac61-lyrical-practice/src/arm_practice/tasks/nz2_vispick_cfg.py
같은 폴더 필요: nz2_common, nz2_reach_cfg, nz2_smooth_action, nz2_vispick_geom, nz2_vispick_obs
등록(__init__.py): NZ2-VisPick-Fixed (영상), NZ2-VisPick-Fixed-State (상태 — 환경·보상 검증용)
로봇: 팔만 (2026-11-10 사용자 결정: 차체 없앰). R12 학습 USD 에서 차체·바퀴를 숨기고 충돌을 끈 _armonly 사본
      → ARM_CONFIG=~/git/lv2_assignment/nz2/R12_NO_USB_URDF_USD/isaac/nz2_arm_armonly.json (scripts/nz2_make_armonly_usd.py)
      팔 베이스(arm_joint1) = 바닥 원점 (0, 0, 0). 좌표 원점 = 팔 베이스 바로 아래 바닥
      URDF 안에서 팔이 차체 어디에 붙어 있든(새 URDF 에서 앞으로 나와도) _armonly json 의 arm_j1_xyz 를 읽어 자동으로 원점에 맞춤

장면 (맵 없음, 바닥의 팔 + 작업대)
  - 박스(선반) 205×205×70 mm, 차체 앞 끝(0.22 m)에서 간격 10~40 mm, 좌우 ±20 mm, yaw ±3°
  - 퍽 4개 준비(빨강/초록 × 직육면체/원기둥). 한 환경에서 색마다 모양 하나만 박스 위, 나머지는 로봇 뒤 바닥
  - 뒷벽 3종(흰/회/나무색) 중 하나를 박스 뒤 100~450 mm 에 세움 (배경 다양화)
  - 카메라: 손목 D435(arm_link5, 30° 하향) + 상단 정면 D435(0.0876, 0, 0.875, 하향 NZ2_TOPCAM_PITCH_DEG 기본 65°)
과제: 명령 색(빨강/초록) 퍽을 집어 40 mm 들고 0.5 s 유지 → 성공(종료). 다른 퍽·박스·차체 건드림 = 감점/실패
단계 (성공률 기준: 에피소드 256개 창의 성공률 ≥ 0.6 이면 올림, NZ2_STAGE 로 고정 가능)
  0: 퍽 1개(명령 색만), 세움, 박스 앞 절반, 간격 10~20 mm
  1: 퍽 2개(빨강+초록), 세움, 박스 전체, 간격 10~40 mm
  2: + 누운 퍽 35 %
관측: 정책 = 관절(위치·속도·실제목표·직전행동) + 명령색 + 손목 RGB-D + 상단 RGB-D (각 4×96×128)
      비평가 = 관절 + 퍽·박스 실제 위치 등 특권 정보 (비대칭 actor-critic)
환경 변수: NZ2_LOOK_POSE, NZ2_ITER_OFFSET(이어 학습 시작 반복), NZ2_STAGE_START(이어 학습 시작 단계), NZ2_TOPCAM_PITCH_DEG(65), NZ2_STAGE, NZ2_CAM_CHECK=1(영상 저장·투영 검사), NZ2_PLAY=1(증강·잡음 끔),
          NZ2_EXIT_WHEN_DONE=1(카메라 점검 후 종료)
"""
from __future__ import annotations

import math
import os
import re

import torch

import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObjectCfg
from isaaclab.managers import CurriculumTermCfg, EventTermCfg, ObservationGroupCfg, ObservationTermCfg
from isaaclab.managers import RewardTermCfg, SceneEntityCfg, TerminationTermCfg
from isaaclab.sensors import CameraCfg, ContactSensorCfg
from isaaclab.sim import RigidBodyMaterialCfg
from isaaclab.utils import configclass
from isaaclab.utils import math as math_utils
from isaaclab_rl.rsl_rl import RslRlCNNModelCfg, RslRlMLPModelCfg, RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg

from arm_practice.config import load_robot_config
from arm_practice.tasks import nz2_common as C
from arm_practice.tasks import nz2_vispick_geom as G
from arm_practice.tasks import nz2_vispick_obs as V
from arm_practice.tasks.nz2_reach_cfg import NZ2ArmReachCfg

# ===== 실험 상수 =====
NUM_ENVS_CAM = 32                       # 8 GB: 영상 2대 × 4ch × 96×128, 32 env × 32 step 저장 ≈ 0.4 GB
NUM_ENVS_STATE = 128
STEPS_PER_ENV = 32
EPISODE_S = 12.0
# ===== (11/11 저녁) 누운 퍽 세워 적재통 넣기 (환경 A = S1 집기 + 적재통 놓기). NZ2_TO_BIN=1 일 때만 =====
#   사용자 결정: 팔 = 차체 정면 12시, 적재통 컵 8개 = 3시 4개 + 9시 4개. 최종 URDF 전이라 아래 치수는 전부 [가정]
#   좌표 = 팔 좌표(팔 베이스 arm_joint1 이 환경 원점). 팔은 차체 중심 앞 0.12 m [가정] → 차체 뒤쪽 좌우에 2×2 씩
TO_BIN = os.environ.get("NZ2_TO_BIN", "0") == "1"
TARGET_LIE = os.environ.get("NZ2_TARGET_LIE", "0") == "1"     # 1 = 목표 퍽을 항상 누운 직육면체로 (단계별 학습용)
if TO_BIN:
    EPISODE_S = float(os.environ.get("NZ2_EPISODE_S", "30.0"))
BIN_SLOTS = ((-0.18, 0.11), (-0.18, 0.17), (-0.24, 0.11), (-0.24, 0.17),       # 9시(왼쪽, +y) 4개 [가정]
             (-0.18, -0.11), (-0.18, -0.17), (-0.24, -0.11), (-0.24, -0.17))   # 3시(오른쪽, −y) 4개 [가정]
# 컵 바닥 윗면 높이 [m, 바닥 기준]. 차체 윗면(0.192)에 두면 수평 공구로 세워 놓을 수 있는 반경이 r ≥ 0.30 m 뿐
#   (11/11 CPU IK 검사, 관절 한계 포함) → 시뮬은 받침대 위 0.30 m [가정]. URDF 확정 후 NZ2_BIN_FLOOR_MM 로 바꿈
BIN_FLOOR_Z = float(os.environ.get("NZ2_BIN_FLOOR_MM", "300")) / 1000.0
BIN_IN, BIN_WALL_T, BIN_WALL_H, BIN_FLOOR_T = 0.047, 0.003, 0.022, 0.003       # 컵 안쪽 47 mm 정사각 [가정: 실물 안지름 47]
BIN_SLOT_FIX = int(os.environ.get("NZ2_BIN_SLOT", "-1"))                        # 0~7 이면 그 칸만, −1 = 무작위
BIN_PARTS = ("bin_floor", "bin_w0", "bin_w1", "bin_w2", "bin_w3")
ENV_SPACING = 3.0
TOP_PITCH_DEG = float(os.environ.get("NZ2_TOPCAM_PITCH_DEG", G.TOP_CAM_PITCH_DEG_DEFAULT))
TOP_CROP = G.top_crop(TOP_PITCH_DEG)
LOOK_NOISE_RAD = 0.05                   # 시작 자세 무작위 (팔 5축)
SHELF_LAT, SHELF_YAW = 0.02, math.radians(3.0)
STAGE_WINDOW_EPS = 256                  # 단계 판정 창: 이만큼 에피소드가 끝날 때마다 성공률 계산
STAGE_UP_RATE = 0.6                     # 창 안 성공률이 이 이상이면 다음 단계 (v3 은 반복 횟수로 올려 0 % 인데 2단계까지 감 → 폐기)
STAGE = {0: dict(gap=(0.01, 0.02), lx=(-G.PUCK_LOCAL_MAX, 0.0), ly=(-0.05, 0.05), two=False, p_lying=0.0),
         1: dict(gap=(0.01, 0.04), lx=(-G.PUCK_LOCAL_MAX, G.PUCK_LOCAL_MAX), ly=(-G.PUCK_LOCAL_MAX, G.PUCK_LOCAL_MAX),
                 two=True, p_lying=0.0),
         2: dict(gap=(0.01, 0.04), lx=(-G.PUCK_LOCAL_MAX, G.PUCK_LOCAL_MAX), ly=(-G.PUCK_LOCAL_MAX, G.PUCK_LOCAL_MAX),
                 two=True, p_lying=0.35),
         # 3: mission.pdf v5 배치 — 시작 박스(205×205×70) 위 퍽 2개, 박스 중심 ±40 mm 한 줄(Y=0).
         #    로봇이 어느 면으로 접근하느냐에 따라 줄이 로봇 기준 앞뒤(x) 또는 좌우(y) → 둘 다 무작위. 놓임 오차 ±3 mm [가정]
         3: dict(gap=(0.01, 0.04), lx=(-0.003, 0.003), ly=(-0.003, 0.003), two=True, p_lying=0.35, mission=True)}
MISSION_PUCK_OFF = 0.040                # mission.pdf: 박스 중심 X±40 mm
# (11/11) 운영진 공지: 원통 퍽은 항상 서 있음. 1 이면 예전처럼 원통도 누움 (비교용)
CYL_CAN_LIE = os.environ.get("NZ2_CYL_CAN_LIE", "0") != "0"
COLORS = {"r": (0.85, 0.10, 0.10), "g": (0.10, 0.65, 0.20)}     # [가정] 실물 퍽 색 (사진으로 확인 필요)
PUCKS = ["puck_r_box", "puck_r_cyl", "puck_g_box", "puck_g_cyl"]  # 번호 = 색*2 + 모양
PARK = [(-0.60, -0.45), (-0.60, -0.15), (-0.60, 0.15), (-0.60, 0.45)]
PUCK_MASS = {"box": 0.0175, "cyl": 0.0135}                       # 실측 (nz2-team)
PUCK_FRICTION = (0.30, 0.45)
# 누운 원기둥이 손대지 않아도 저절로 굴러감 (11/10 진단: 팔 정지 상태에서 256 에피소드 중 55회, 1 s 전후; 감쇠 3 → 부족, 20 → 정답 위치 95.3 %)
# 실물은 구름 저항으로 멈춰 있음 → 회전 감쇠로 흉내 [가정], 놓을 때 낙하 높이도 줄임
PUCK_ANG_DAMP = float(os.environ.get("NZ2_PUCK_ANG_DAMP", "20.0"))
PUCK_DROP_M = float(os.environ.get("NZ2_PUCK_DROP_MM", "0.1")) / 1000.0
WALLS = [(0.92, 0.92, 0.90), (0.60, 0.60, 0.60), (0.76, 0.64, 0.48)]   # [가정] 배경 벽 색
WALL_SIZE = (0.01, 0.80, 0.38)
WALL_DIST = (0.10, 0.45)                # 박스 뒤 끝 → 벽 [m]
# 판정
GRASP_Z_MIN = G.BOX[2] + 0.025          # 손끝(TCP+20 mm)이 박스 위 5 mm 이상 (누운 퍽)
GRASP_BELOW_TOP = 0.0                   # 목표 TCP = 퍽 윗면 높이 → 손끝(TCP+20 mm)이 윗면 아래 20 mm (실측 15 mm 면 충분 + 여유 5 mm)
LIE_GRASP_UP = float(os.environ.get("NZ2_LIE_GRASP_UP_MM", "5")) / 1000.0   # 누운 퍽만 목표 TCP 를 이만큼 올림 (11/10 시험: 0 → 5 mm 에서 누운 퍽 3 % → 43 %)
GRIP_N, WRONG_N = 0.2, 0.2
LIFT_M, HOLD_STEPS = 0.04, 15           # 40 mm, 0.5 s @30 Hz
NEAR_M, FAR_M = 0.02, 0.05
TARGET_PUSH_M, TARGET_TILT_DEG = 0.04, 35.0
OTHER_PUSH_M, OTHER_TILT_DEG = 0.015, 20.0
ITER_OFFSET = int(os.environ.get("NZ2_ITER_OFFSET", "0"))   # 이어 학습: 체크포인트 반복 수 (단계·벌점 커리큘럼이 처음으로 돌아가지 않게)
W = dict(reach=1.0, reach_fine=1.5, open_far=0.3, close_near=0.5, grasp=2.0, lift=5.0, success=5.0,
         success_bonus=20.0, wrong_touch=-1.0, puck_fail=-10.0)      # [가정]


# ---------------------------------------------------------------------------
# 환경 상태
# ---------------------------------------------------------------------------
class _VPState:
    def __init__(self, n, dev):
        self.color = torch.zeros(n, dtype=torch.long, device=dev)        # 0 빨강, 1 초록 (명령)
        self.shape = torch.zeros(n, 2, dtype=torch.long, device=dev)     # 색별 모양 0 직육면체, 1 원기둥
        self.lying = torch.zeros(n, 2, dtype=torch.bool, device=dev)
        self.on_box = torch.zeros(n, 2, dtype=torch.bool, device=dev)
        self.init_pos = torch.zeros(n, 2, 3, device=dev)
        self.init_axis = torch.zeros(n, 2, 3, device=dev)
        self.init_axis[..., 2] = 1.0
        self.hold = torch.zeros(n, dtype=torch.long, device=dev)
        self.bin_slot = torch.zeros(n, dtype=torch.long, device=dev)     # (11/11) 목표 컵 번호
        self.bin_xy = torch.zeros(n, 2, device=dev)                      # 목표 컵 중심 (팔 좌표)
        self.stage = int(os.environ.get("NZ2_STAGE") or os.environ.get("NZ2_STAGE_START") or 0)
        self.n_eps, self.n_succ, self.last_rate = 0, 0, 0.0      # 단계 판정 창


def vp(env) -> _VPState:
    st = getattr(env, "_nz2_vp", None)
    if st is None:
        st = _VPState(env.num_envs, env.device)
        env._nz2_vp = st
    return st


def _puck_all(env):
    pos = torch.stack([env.scene[k].data.root_pos_w.torch for k in PUCKS], dim=1)    # (N,4,3)
    quat = torch.stack([env.scene[k].data.root_quat_w.torch for k in PUCKS], dim=1)  # (N,4,4)
    return pos, quat


def _color_pose(env, which: str):
    """which = 'target' | 'other' → 그 색 퍽의 월드 위치·자세, 퍽 번호(0~3), 색"""
    st = vp(env)
    col = st.color if which == "target" else 1 - st.color
    idx = col * 2 + st.shape.gather(1, col.unsqueeze(1)).squeeze(1)
    pos, quat = _puck_all(env)
    ar = torch.arange(env.num_envs, device=env.device)
    return pos[ar, idx], quat[ar, idx], idx, col


def target_pos_w(env):
    return _color_pose(env, "target")[0]


def _grasp_point(env):
    """잡기 목표 TCP: 퍽 윗면 높이 (세운 퍽 z 0.130 → 손끝 박스 위 40 mm, 누운 퍽 z 0.100 → 손끝 박스 위 10 mm)"""
    st = vp(env)
    p = target_pos_w(env).clone()
    lie = st.lying.gather(1, st.color.unsqueeze(1)).squeeze(1)
    half = torch.where(lie, torch.full_like(p[:, 2], G.PUCK_BOX[0] / 2), torch.full_like(p[:, 2], G.PUCK_BOX[2] / 2))
    p[:, 2] = torch.clamp(p[:, 2] + half - GRASP_BELOW_TOP + lie.float() * LIE_GRASP_UP, min=GRASP_Z_MIN)
    return p


def _tcp_cfg(env):
    cfg = getattr(env, "_nz2_tcp_cfg", None)
    if cfg is None:
        cfg = C.tcp_cfg()
        cfg.resolve(env.scene)
        env._nz2_tcp_cfg = cfg
    return cfg


def _tcp_dist(env):
    return torch.linalg.norm(C.tcp_pos_w(env, _tcp_cfg(env)) - _grasp_point(env), dim=1)


def _finger_puck_force(env, side: str):
    """손가락 → 퍽 4개 접촉력 (N,4) [N]"""
    fm = env.scene.sensors[f"grip_{side}"].data.normal_force_matrix_w_history.torch     # (N,H,B,4,3)
    return torch.linalg.norm(fm, dim=-1).sum(dim=2).amax(dim=1)


def _bad_force(env, name: str):
    fm = env.scene.sensors[name].data.normal_force_matrix_w_history.torch
    return torch.linalg.norm(fm, dim=-1).sum(dim=-1).amax(dim=(1, 2))


def _grasped(env):
    idx = _color_pose(env, "target")[2].unsqueeze(1)
    fl = _finger_puck_force(env, "l").gather(1, idx).squeeze(1)
    fr = _finger_puck_force(env, "r").gather(1, idx).squeeze(1)
    return (fl > GRIP_N) & (fr > GRIP_N)


def _lift(env):
    st = vp(env)
    p = target_pos_w(env)
    init = st.init_pos.gather(1, st.color.view(-1, 1, 1).expand(-1, 1, 3)).squeeze(1)
    return p[:, 2] - init[:, 2]


def _success_now(env):
    return _grasped(env) & (_lift(env) >= LIFT_M) & ~_other_disturbed(env)


def _disturbed(env, which, push_m, tilt_deg):
    st = vp(env)
    pos, quat, _, col = _color_pose(env, which)
    init = st.init_pos.gather(1, col.view(-1, 1, 1).expand(-1, 1, 3)).squeeze(1)
    axis0 = st.init_axis.gather(1, col.view(-1, 1, 1).expand(-1, 1, 3)).squeeze(1)
    pushed = torch.linalg.norm(pos[:, :2] - init[:, :2], dim=1) > push_m
    axis = math_utils.quat_apply(quat, torch.tensor([0.0, 0.0, 1.0], device=env.device).expand_as(pos))
    tilted = torch.sum(axis * axis0, dim=1) < math.cos(math.radians(tilt_deg))
    return pushed | tilted, pos, init


def _other_disturbed(env):
    st = vp(env)
    bad, _, _ = _disturbed(env, "other", OTHER_PUSH_M, OTHER_TILT_DEG)
    on = st.on_box.gather(1, (1 - st.color).unsqueeze(1)).squeeze(1)
    return bad & on


# ---------------------------------------------------------------------------
# 리셋: 박스·벽·퍽 배치
# ---------------------------------------------------------------------------
def _bin_part_geom():
    """컵 부품 (이름, 크기, 컵 중심 기준 위치). 안쪽 BIN_IN 정사각, 벽 두께 BIN_WALL_T, 높이 BIN_WALL_H"""
    a = BIN_IN / 2 + BIN_WALL_T / 2
    full = BIN_IN + 2 * BIN_WALL_T
    return (("bin_floor", (full, full, BIN_FLOOR_T), (0.0, 0.0, -BIN_FLOOR_T / 2)),
            ("bin_w0", (BIN_WALL_T, full, BIN_WALL_H), (a, 0.0, BIN_WALL_H / 2)),
            ("bin_w1", (BIN_WALL_T, full, BIN_WALL_H), (-a, 0.0, BIN_WALL_H / 2)),
            ("bin_w2", (BIN_IN, BIN_WALL_T, BIN_WALL_H), (0.0, a, BIN_WALL_H / 2)),
            ("bin_w3", (BIN_IN, BIN_WALL_T, BIN_WALL_H), (0.0, -a, BIN_WALL_H / 2)))


def _place_bin(env, env_ids, origin):
    """(11/11) 목표 컵 1개를 8칸 중 하나에 놓음 (시뮬엔 차체가 없으므로 컵만 공중 받침 높이에 고정)"""
    st = vp(env)
    dev = env.device
    n = len(env_ids)
    slots = torch.tensor(BIN_SLOTS, device=dev)
    k = torch.full((n,), BIN_SLOT_FIX, dtype=torch.long, device=dev) if BIN_SLOT_FIX >= 0 \
        else torch.randint(len(BIN_SLOTS), (n,), device=dev)
    c = slots[k]
    st.bin_slot[env_ids], st.bin_xy[env_ids] = k, c
    ident = torch.tensor([0.0, 0.0, 0.0, 1.0], device=dev).expand(n, 4)
    for name, _size, off in _bin_part_geom():
        p = torch.stack([c[:, 0] + off[0], c[:, 1] + off[1], torch.full((n,), BIN_FLOOR_Z + off[2], device=dev)], 1)
        env.scene[name].write_root_pose_to_sim_index(root_pose=torch.cat([p + origin, ident], -1), env_ids=env_ids)


def reset_vispick_scene(env, env_ids):
    st = vp(env)
    dev = env.device
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=dev)
    n = len(env_ids)
    S = STAGE[st.stage]
    U = lambda r, *shape: math_utils.sample_uniform(r[0], r[1], (n, *shape) if shape else (n,), device=dev)
    zeros = torch.zeros(n, device=dev)
    origin = env.scene.env_origins[env_ids]
    ident = torch.tensor([0.0, 0.0, 0.0, 1.0], device=dev).expand(n, 4)
    # 박스
    d, w, h = G.BOX
    s_yaw = U((-SHELF_YAW, SHELF_YAW))
    s_q = math_utils.quat_from_euler_xyz(zeros, zeros, s_yaw)
    s_pos = torch.stack([G.CHASSIS_FRONT_X + U(S["gap"]) + d / 2, U((-SHELF_LAT, SHELF_LAT)), zeros + h / 2], dim=1)
    env.scene["shelf"].write_root_pose_to_sim_index(root_pose=torch.cat([s_pos + origin, s_q], -1), env_ids=env_ids)
    # 뒷벽: 하나만 박스 뒤, 나머지는 땅속
    wi = torch.randint(len(WALLS), (n,), device=dev)
    for k in range(len(WALLS)):
        use = (wi == k).unsqueeze(-1)
        on_pos = torch.stack([s_pos[:, 0] + d / 2 + U(WALL_DIST), U((-0.10, 0.10)), zeros + WALL_SIZE[2] / 2], 1)
        off_pos = torch.tensor([0.0, 0.0, -3.0 - k], device=dev).expand(n, 3)
        pose = torch.cat([torch.where(use, on_pos, off_pos) + origin, ident], -1)
        env.scene[f"wall{k}"].write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
    # 퍽: 명령 색·모양·누움
    color = torch.randint(2, (n,), device=dev)
    shape = torch.randint(2, (n, 2), device=dev)
    lying = torch.rand(n, 2, device=dev) < S["p_lying"]
    if not CYL_CAN_LIE:                      # (11/11) 운영진 공지: 원통은 항상 서 있음 → 누움은 직육면체만
        lying &= shape == 0
    if TARGET_LIE:                           # (11/11) 목표 퍽 = 누운 직육면체 고정
        ar_ = torch.arange(n, device=dev)
        shape[ar_, color] = 0
        lying[ar_, color] = True
    if TO_BIN:
        _place_bin(env, env_ids, origin)
    if S.get("mission"):
        # 두 퍽이 박스 중심 ±40 mm 한 줄 (줄 방향·목표 쪽 무작위) + 놓임 오차
        along_x = torch.rand(n, device=dev) < 0.5
        sgn = torch.where(torch.rand(n, device=dev) < 0.5, 1.0, -1.0)
        off = torch.stack([torch.where(along_x, sgn, zeros), torch.where(along_x, zeros, sgn)], 1) * MISSION_PUCK_OFF
        lt = off + torch.stack([U(S["lx"]), U(S["ly"])], 1)
        lo = -off + torch.stack([U(S["lx"]), U(S["ly"])], 1)
        other_on = torch.ones(n, dtype=torch.bool, device=dev) & S["two"]
    else:
        lt = torch.stack([U(S["lx"]), U(S["ly"])], 1)                       # 목표 퍽 (박스 좌표)
        cand = torch.stack([U(S["lx"], 16), U(S["ly"], 16)], -1)             # 다른 퍽 후보 16개
        okc = torch.linalg.norm(cand - lt.unsqueeze(1), dim=-1) >= G.PUCK_MIN_SEP
        first = torch.where(okc.any(1), okc.float().argmax(1), torch.zeros(n, dtype=torch.long, device=dev))
        lo = cand[torch.arange(n, device=dev), first]
        other_on = okc.any(1) & S["two"]
    on_box = torch.zeros(n, 2, dtype=torch.bool, device=dev)
    on_box[torch.arange(n, device=dev), color] = True
    on_box[torch.arange(n, device=dev), 1 - color] = other_on
    st.init_pos[env_ids] = 0.0
    zax = torch.tensor([0.0, 0.0, 1.0], device=dev).expand(n, 3)
    for c in range(2):
        local = torch.where((color == c).unsqueeze(-1), lt, lo)
        p_xy = s_pos + math_utils.quat_apply(s_q, torch.cat([local, zeros.unsqueeze(-1)], -1))
        lie = lying[:, c]
        hz = torch.where(lie, torch.full((n,), G.PUCK_BOX[0] / 2, device=dev), torch.full((n,), G.PUCK_BOX[2] / 2, device=dev))
        p = torch.stack([p_xy[:, 0], p_xy[:, 1], h + hz + PUCK_DROP_M], 1)
        q = math_utils.quat_from_euler_xyz(torch.where(lie, torch.full((n,), math.pi / 2, device=dev), zeros), zeros,
                                           U((-math.pi, math.pi)))
        for sh in range(2):
            k = c * 2 + sh
            use = (on_box[:, c] & (shape[:, c] == sh)).unsqueeze(-1)
            park = torch.tensor([*PARK[k], G.PUCK_BOX[2] / 2 + 0.001], device=dev).expand(n, 3)
            pose = torch.cat([torch.where(use, p, park) + origin, torch.where(use, q, ident)], -1)
            obj = env.scene[PUCKS[k]]
            obj.write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
            obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros(n, 6, device=dev), env_ids=env_ids)
        st.init_pos[env_ids, c] = p + origin
        st.init_axis[env_ids, c] = math_utils.quat_apply(q, zax)
    st.color[env_ids], st.shape[env_ids], st.lying[env_ids], st.on_box[env_ids] = color, shape, lying, on_box
    st.hold[env_ids] = 0
    if env.common_step_counter > 0:
        st.n_eps += n


def vispick_stage(env, env_ids):
    """성공률 기반 단계: 에피소드 STAGE_WINDOW_EPS 개마다 성공률을 보고 STAGE_UP_RATE 이상이면 한 단계 올림"""
    st = vp(env)
    if st.n_eps >= STAGE_WINDOW_EPS:
        st.last_rate = st.n_succ / st.n_eps
        if not os.environ.get("NZ2_STAGE") and st.last_rate >= STAGE_UP_RATE and st.stage < 2:
            st.stage += 1
            print(f"[NZ2] 성공률 {st.last_rate:.2f} → 단계 {st.stage}")
        st.n_eps, st.n_succ = 0, 0
    return float(st.stage)


def window_success_rate(env, env_ids):
    """TensorBoard Curriculum/success_rate 로 보이는 최근 창 성공률 (기록용)"""
    return float(vp(env).last_rate)


# ---------------------------------------------------------------------------
# 관측
# ---------------------------------------------------------------------------
def _to_root(env, pos_w, quat_w=None):
    r = env.scene["robot"].data
    return math_utils.subtract_frame_transforms(r.root_pos_w.torch, r.root_quat_w.torch, pos_w, quat_w)


def obs_target_color(env):
    return torch.nn.functional.one_hot(vp(env).color, 2).float()


def obs_target_pose(env):
    pos, quat, _, _ = _color_pose(env, "target")
    p, q = _to_root(env, pos, quat)
    return torch.cat([p, q], -1)


def obs_target_kind(env):
    st = vp(env)
    sh = st.shape.gather(1, st.color.unsqueeze(1)).squeeze(1)
    lie = st.lying.gather(1, st.color.unsqueeze(1)).float()
    return torch.cat([torch.nn.functional.one_hot(sh, 2).float(), lie], -1)


def obs_other(env):
    st = vp(env)
    p = _to_root(env, _color_pose(env, "other")[0])[0]
    on = st.on_box.gather(1, (1 - st.color).unsqueeze(1)).float()
    return torch.cat([p * on, on], -1)


def obs_tcp_to_grasp(env):
    return _grasp_point(env) - C.tcp_pos_w(env, _tcp_cfg(env))


def obs_shelf(env):
    return _to_root(env, env.scene["shelf"].data.root_pos_w.torch)[0]


def obs_grasp_lift(env):
    return torch.stack([_grasped(env).float(), _lift(env)], -1)


# ---------------------------------------------------------------------------
# 보상·종료
# ---------------------------------------------------------------------------
def _openness(env, lo, hi):
    j = env.scene["robot"].data.joint_pos.torch
    jid = env.scene["robot"].find_joints("gripper_joint")[0][0]
    return torch.clamp((j[:, jid] - lo) / (hi - lo), 0.0, 1.0)


def rew_reach(env, std: float):
    return 1.0 - torch.tanh(_tcp_dist(env) / std)


def rew_open_far(env, lo: float, hi: float):
    return _openness(env, lo, hi) * (_tcp_dist(env) > FAR_M).float()


def rew_close_near(env, lo: float, hi: float):
    return (1.0 - _openness(env, lo, hi)) * (_tcp_dist(env) < NEAR_M).float()


def rew_grasp(env):
    return _grasped(env).float()


def rew_lift(env):
    return _grasped(env).float() * torch.clamp(_lift(env) / LIFT_M, 0.0, 1.0)


def rew_success(env):
    return _success_now(env).float()


def rew_wrong_touch(env):
    idx = (1 - vp(env).color) * 2 + vp(env).shape.gather(1, (1 - vp(env).color).unsqueeze(1)).squeeze(1)
    f = torch.maximum(_finger_puck_force(env, "l"), _finger_puck_force(env, "r")).gather(1, idx.unsqueeze(1)).squeeze(1)
    return (f > WRONG_N).float()


def rew_bad_contact(env, sensor_name: str, threshold: float):
    return (_bad_force(env, sensor_name) > threshold).float()


def term_bad_contact(env, sensor_name: str, threshold: float):
    return _bad_force(env, sensor_name) > threshold


def term_success(env):
    st = vp(env)
    s = _success_now(env)
    st.hold = torch.where(s, st.hold + 1, torch.zeros_like(st.hold))
    done = st.hold >= HOLD_STEPS
    st.n_succ += int(done.sum())
    return done


def term_target_disturbed(env):
    bad, pos, init = _disturbed(env, "target", TARGET_PUSH_M, TARGET_TILT_DEG)
    return bad & ((pos[:, 2] - init[:, 2]) < 0.01)


def term_other_disturbed(env):
    return _other_disturbed(env)


def term_target_dropped(env):
    return target_pos_w(env)[:, 2] < G.BOX[2] - 0.02


# ---------------------------------------------------------------------------
# 장면 부품
# ---------------------------------------------------------------------------
def _puck_cfg(name):
    c, sh = name.split("_")[1], name.split("_")[2]
    common = dict(
        rigid_props=sim_utils.RigidBodyPropertiesCfg(solver_position_iteration_count=16, solver_velocity_iteration_count=1,
                                                     max_depenetration_velocity=1.0, angular_damping=PUCK_ANG_DAMP),
        mass_props=sim_utils.MassPropertiesCfg(mass=PUCK_MASS[sh]),
        collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.002, rest_offset=0.0),
        physics_material=RigidBodyMaterialCfg(static_friction=0.35, dynamic_friction=0.30, restitution=0.1),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=COLORS[c], roughness=0.6))
    spawn = (sim_utils.CuboidCfg(size=G.PUCK_BOX, **common) if sh == "box"
             else sim_utils.CylinderCfg(radius=G.PUCK_CYL_R, height=G.PUCK_CYL_H, axis="Z", **common))
    k = PUCKS.index(name)
    return RigidObjectCfg(prim_path="{ENV_REGEX_NS}/" + name, spawn=spawn,
                          init_state=RigidObjectCfg.InitialStateCfg(pos=(*PARK[k], 0.031), rot=(0.0, 0.0, 0.0, 1.0)))


def _kinematic_box(path, size, color, pos):
    return RigidObjectCfg(
        prim_path=path,
        spawn=sim_utils.CuboidCfg(size=size, rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
                                  collision_props=sim_utils.CollisionPropertiesCfg(),
                                  physics_material=RigidBodyMaterialCfg(static_friction=0.5, dynamic_friction=0.5),
                                  visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color, roughness=0.8)),
        init_state=RigidObjectCfg.InitialStateCfg(pos=pos, rot=(0.0, 0.0, 0.0, 1.0)))


def _pinhole():
    return sim_utils.PinholeCameraCfg(focal_length=G.FOCAL_MM, horizontal_aperture=G.H_APERTURE_MM,
                                      clipping_range=G.CLIP_RANGE)


def _percep_flush(env, env_ids=None):
    """interval 이벤트 → nz2_percep_dump.flush_event (순환 import 피하려고 실행 시점 import)"""
    from arm_practice.tasks import nz2_percep_dump as PD
    PD.flush_event(env, env_ids)


def _qy(deg):
    return (0.0, math.sin(math.radians(deg) / 2), 0.0, math.cos(math.radians(deg) / 2))   # (x,y,z,w), +Y 회전 = 아래로


def _group(terms: dict, corrupt: bool = False):
    g = ObservationGroupCfg()
    for k, v in terms.items():
        setattr(g, k, v)
    g.enable_corruption = corrupt
    g.concatenate_terms = True
    return g


# ---------------------------------------------------------------------------
# 환경 설정
# ---------------------------------------------------------------------------
@configclass
class NZ2VisPickCfg(NZ2ArmReachCfg):
    nz2_cameras: bool = True

    def __post_init__(self):
        super().__post_init__()
        c = load_robot_config(os.environ["ARM_CONFIG"])
        if "armonly" not in os.path.basename(c["usd_path"]):
            print(f"[NZ2][경고] 차체가 남은 USD: {c['usd_path']} → nz2_make_armonly_usd.py 로 _armonly 를 만들고 nz2_arm_armonly.json 사용")
        names = [re.escape(n) for n in c["joint_names"]]
        g_lo, g_hi = c["lower"][-1], c["upper"][-1]
        self.scene.num_envs = NUM_ENVS_CAM if self.nz2_cameras else NUM_ENVS_STATE
        self.scene.env_spacing = ENV_SPACING
        self.episode_length_s = EPISODE_S

        # ---- 시작 자세 = 관측 자세 (손목 카메라가 박스 전체를 봄) ----
        look = dict(zip(names, [*G.look_pose(), G.GRIPPER_OPEN], strict=True))
        self.scene.robot.init_state.joint_pos = look
        # 팔 베이스를 바닥 원점에: 숨긴 차체(base_link) 뿌리를 arm_joint1 위치만큼 반대로 옮긴다 (뿌리는 USD root_joint 로 고정)
        j1 = c.get("arm_j1_xyz")
        if j1 is None:
            raise ValueError(f"{os.environ['ARM_CONFIG']} 에 arm_j1_xyz 없음 → bash nz2_vispick_run.sh usd 로 다시 만드세요")
        self.scene.robot.init_state.pos = (G.ARM_X - j1[0], -j1[1], G.ARM_Z - j1[2])
        self.events.look_noise = EventTermCfg(
            func=mdp.reset_joints_by_offset, mode="reset",
            params={"position_range": (-LOOK_NOISE_RAD, LOOK_NOISE_RAD), "velocity_range": (0.0, 0.0),
                    "asset_cfg": SceneEntityCfg("robot", joint_names=["arm_joint[1-5]"])})

        # ---- Reach 전용 항목 제거 ----
        for k in ("end_effector_position_tracking", "tcp_tanh", "tcp_tanh_fine", "success", "near20"):
            setattr(self.rewards, k, None)
        self.curriculum.target_range = None
        self.observations.policy.pose_command = None
        self.commands.ee_pose.debug_vis = False

        # ---- 장면 ----
        d, w, h = G.BOX
        self.scene.shelf = _kinematic_box("{ENV_REGEX_NS}/Shelf", G.BOX, (0.80, 0.70, 0.55),
                                          (G.CHASSIS_FRONT_X + 0.02 + d / 2, 0.0, h / 2))
        for k, col in enumerate(WALLS):
            setattr(self.scene, f"wall{k}", _kinematic_box("{ENV_REGEX_NS}/Wall%d" % k, WALL_SIZE, col, (0.0, 0.0, -3.0 - k)))
        for name in PUCKS:
            setattr(self.scene, name, _puck_cfg(name))
        if TO_BIN:                                    # (11/11) 적재통 컵 1개 (바닥판 + 벽 4장, 운동학)
            for name, size, off in _bin_part_geom():
                setattr(self.scene, name, _kinematic_box("{ENV_REGEX_NS}/" + name.capitalize(), size, (0.35, 0.35, 0.40),
                                                         (BIN_SLOTS[0][0] + off[0], BIN_SLOTS[0][1] + off[1], BIN_FLOOR_Z + off[2])))

        # ---- 카메라 ----
        if self.nz2_cameras:
            chk = os.environ.get("NZ2_CAM_CHECK") == "1"
            # (11/11) NZ2_WRIST_FULLRES=1 → 손목 카메라를 실기 640×480 으로 렌더 (관측은 rgbd_tensor 가 96×128 로 줄임)
            _WR = (G.REAL_W, G.REAL_H) if os.environ.get("NZ2_WRIST_FULLRES") == "1" else G.WRIST_RENDER
            self.scene.wrist_cam = CameraCfg(
                prim_path="{ENV_REGEX_NS}/Robot/" + C.LINK_PATHS["arm_link5"] + "/nz2_wrist_cam",
                offset=CameraCfg.OffsetCfg(pos=G.WRIST_CAM_POS, rot=_qy(G.WRIST_CAM_TILT_DEG), convention="world"),
                spawn=_pinhole(), width=_WR[0], height=_WR[1],
                data_types=["rgb", "distance_to_image_plane"], update_latest_camera_pose=chk or _WR != G.WRIST_RENDER)
            self.scene.top_cam = CameraCfg(
                prim_path="{ENV_REGEX_NS}/nz2_top_cam",
                offset=CameraCfg.OffsetCfg(pos=G.TOP_CAM_POS, rot=_qy(TOP_PITCH_DEG), convention="world"),
                spawn=_pinhole(), width=G.TOP_RENDER[0], height=G.TOP_RENDER[1],
                data_types=["rgb", "distance_to_image_plane"], update_latest_camera_pose=chk)
            # (11/11) 인식 실험: 손목 D435 자리에 실기 해상도 640×480 카메라 추가 (NZ2_PERCEP_DUMP 일 때만)
            # 손목을 640×480 으로 렌더하면 percep_cam 을 따로 달지 않음 (NZ2_PERCEP_SENSOR=wrist_cam 으로 저장)
            if os.environ.get("NZ2_PERCEP_DUMP"):
                # (11/11 v2) 영상 저장은 렌더 뒤 interval 이벤트에서 (행동 처리 안에서 읽으면 빈 영상)
                self.events.percep_flush = EventTermCfg(func=_percep_flush, mode="interval",
                                                        interval_range_s=(0.0, 0.0), is_global_time=True)
            if os.environ.get("NZ2_PERCEP_DUMP") and _WR == G.WRIST_RENDER:
                self.scene.percep_cam = CameraCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/" + C.LINK_PATHS["arm_link5"] + "/nz2_percep_cam",
                    offset=CameraCfg.OffsetCfg(pos=G.WRIST_CAM_POS, rot=_qy(G.WRIST_CAM_TILT_DEG), convention="world"),
                    spawn=_pinhole(), width=G.REAL_W, height=G.REAL_H,
                    data_types=["rgb", "distance_to_image_plane"], update_latest_camera_pose=True)
            if chk:
                self.events.cam_check = EventTermCfg(
                    func=V.cam_check_event, mode="interval", interval_range_s=(1.0, 1.0), is_global_time=True,
                    params={"cams": {"wrist_cam": None, "top_cam": TOP_CROP,
                                     **({"percep_cam": None} if os.environ.get("NZ2_PERCEP_DUMP") and _WR == G.WRIST_RENDER else {})},
                            "target_fn": target_pos_w})

        # ---- 접촉: 손가락(나쁜 상대 / 퍽 4개), 팔 링크는 Reach 의 '무엇에든' 센서 유지 ----
        for key in ("finger_l", "finger_r"):
            setattr(self.scene, f"contact_{key}", None)
            setattr(self.rewards, f"contact_{key}_pen", None)
            setattr(self.terminations, f"contact_{key}_stop", None)
        bad = ["{ENV_REGEX_NS}/Shelf"] + ["{ENV_REGEX_NS}/Wall%d" % k for k in range(len(WALLS))]
        if TO_BIN:                                    # 손가락이 컵을 치면 실격과 같음
            bad += ["{ENV_REGEX_NS}/" + nm.capitalize() for nm in BIN_PARTS]
        pucks = ["{ENV_REGEX_NS}/" + p for p in PUCKS]
        for side, key in (("l", "finger_l"), ("r", "finger_r")):
            path = "{ENV_REGEX_NS}/Robot/" + C.LINK_PATHS[key]
            bn = f"contact_fbad_{side}"
            setattr(self.scene, bn, ContactSensorCfg(prim_path=path, history_length=3, filter_prim_paths_expr=bad))
            setattr(self.scene, f"grip_{side}", ContactSensorCfg(prim_path=path, history_length=3, filter_prim_paths_expr=pucks))
            setattr(self.rewards, f"{bn}_pen", RewardTermCfg(func=rew_bad_contact, weight=C.W_CONTACT,
                    params={"sensor_name": bn, "threshold": C.THRESH_PENALTY_N}))
            setattr(self.terminations, f"{bn}_stop", TerminationTermCfg(func=term_bad_contact,
                    params={"sensor_name": bn, "threshold": C.THRESH_TERMINATE_N}))

        # ---- 리셋·DR·단계 ----
        self.events.reset_vispick = EventTermCfg(func=reset_vispick_scene, mode="reset")
        for name in PUCKS:
            setattr(self.events, f"{name}_mat", EventTermCfg(
                func=mdp.randomize_rigid_body_material, mode="startup",
                params={"asset_cfg": SceneEntityCfg(name), "static_friction_range": PUCK_FRICTION,
                        "dynamic_friction_range": PUCK_FRICTION, "restitution_range": (0.1, 0.4),
                        "num_buckets": 32, "make_consistent": True}))
        self.curriculum.vispick_stage = CurriculumTermCfg(func=vispick_stage)
        self.curriculum.success_rate = CurriculumTermCfg(func=window_success_rate)
        # 이어 학습: 벌점 커리큘럼(action_rate·joint_vel)이 이미 끝난 시점이면 최종 가중치로 시작
        if ITER_OFFSET * STEPS_PER_ENV >= self.curriculum.action_rate.params["num_steps"]:
            self.rewards.action_rate.weight = self.curriculum.action_rate.params["weight"]
            self.rewards.joint_vel.weight = self.curriculum.joint_vel.params["weight"]

        # ---- 관측 ----
        pol = self.observations.policy
        pol.target_color = ObservationTermCfg(func=obs_target_color)
        self.observations.critic = _group({
            "joint_pos": ObservationTermCfg(func=mdp.joint_pos_rel),
            "joint_vel": ObservationTermCfg(func=mdp.joint_vel_rel),
            "applied_target": ObservationTermCfg(func=C.applied_target_rel),
            "target_color": ObservationTermCfg(func=obs_target_color),
            "target_pose": ObservationTermCfg(func=obs_target_pose),
            "target_kind": ObservationTermCfg(func=obs_target_kind),
            "other": ObservationTermCfg(func=obs_other),
            "tcp_to_grasp": ObservationTermCfg(func=obs_tcp_to_grasp),
            "shelf": ObservationTermCfg(func=obs_shelf),
            "grasp_lift": ObservationTermCfg(func=obs_grasp_lift)})
        if self.nz2_cameras:
            self.observations.wrist_cam = _group({"rgbd": ObservationTermCfg(
                func=V.obs_cam_rgbd, params={"sensor_name": "wrist_cam", "crop": None})})
            self.observations.top_cam = _group({"rgbd": ObservationTermCfg(
                func=V.obs_cam_rgbd, params={"sensor_name": "top_cam", "crop": TOP_CROP})})
        if os.environ.get("NZ2_PLAY") == "1":
            pol.enable_corruption = False

        # ---- 보상 ----
        gp = {"lo": g_lo, "hi": g_hi}
        R = self.rewards
        R.reach = RewardTermCfg(func=rew_reach, weight=W["reach"], params={"std": 0.10})
        R.reach_fine = RewardTermCfg(func=rew_reach, weight=W["reach_fine"], params={"std": 0.02})
        R.open_far = RewardTermCfg(func=rew_open_far, weight=W["open_far"], params=gp)
        R.close_near = RewardTermCfg(func=rew_close_near, weight=W["close_near"], params=gp)
        R.grasp = RewardTermCfg(func=rew_grasp, weight=W["grasp"])
        R.lift = RewardTermCfg(func=rew_lift, weight=W["lift"])
        R.success = RewardTermCfg(func=rew_success, weight=W["success"])
        R.success_bonus = RewardTermCfg(func=mdp.is_terminated_term, weight=W["success_bonus"], params={"term_keys": "success"})
        R.wrong_touch = RewardTermCfg(func=rew_wrong_touch, weight=W["wrong_touch"])
        R.puck_fail = RewardTermCfg(func=mdp.is_terminated_term, weight=W["puck_fail"],
                                    params={"term_keys": "(target_disturbed|other_disturbed|target_dropped)"})

        # ---- 종료 ----
        T = self.terminations
        T.success = TerminationTermCfg(func=term_success)
        T.target_disturbed = TerminationTermCfg(func=term_target_disturbed)
        T.other_disturbed = TerminationTermCfg(func=term_other_disturbed)
        T.target_dropped = TerminationTermCfg(func=term_target_dropped)


@configclass
class NZ2VisPickStateCfg(NZ2VisPickCfg):
    nz2_cameras: bool = False


# ---------------------------------------------------------------------------
# PPO
# ---------------------------------------------------------------------------
def _algo():
    return RslRlPpoAlgorithmCfg(value_loss_coef=1.0, use_clipped_value_loss=True, clip_param=0.2, entropy_coef=0.01,
                                num_learning_epochs=5, num_mini_batches=4, learning_rate=3.0e-4, schedule="adaptive",
                                gamma=0.99, lam=0.95, desired_kl=0.01, max_grad_norm=1.0)


def _critic():
    return RslRlMLPModelCfg(hidden_dims=[256, 128, 64], activation="elu", obs_normalization=True)


@configclass
class NZ2VisPickPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = STEPS_PER_ENV
    max_iterations = 1500
    save_interval = 50
    experiment_name = "nz2_vispick"
    obs_groups = {"actor": ["policy", "wrist_cam", "top_cam"], "critic": ["critic"]}
    actor = RslRlCNNModelCfg(
        cnn_cfg=RslRlCNNModelCfg.CNNCfg(output_channels=[16, 32, 32, 32], kernel_size=[5, 3, 3, 3],
                                        stride=[2, 2, 2, 2], activation="relu"),
        hidden_dims=[256, 128], activation="elu", obs_normalization=True,
        distribution_cfg=RslRlCNNModelCfg.GaussianDistributionCfg(init_std=0.6))
    critic = _critic()
    algorithm = _algo()


@configclass
class NZ2VisPickStatePPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = STEPS_PER_ENV        # 단계 전환 계산과 같은 값
    max_iterations = 1500
    save_interval = 50
    experiment_name = "nz2_vispick_state"
    obs_groups = {"actor": ["policy", "critic"], "critic": ["critic"]}
    actor = RslRlMLPModelCfg(hidden_dims=[256, 128, 64], activation="elu", obs_normalization=True,
                             distribution_cfg=RslRlMLPModelCfg.GaussianDistributionCfg(init_std=0.6))
    critic = _critic()
    algorithm = _algo()


# ---------------------------------------------------------------------------
# IK + 잔차(residual) RL — 행동 클래스: nz2_vispick_residual.py, 정책 모델: nz2_vispick_residual_model.py
#   a = a_IK(pose_hat) + α·tanh(π(o))·gate  (gate = APPROACH·CLOSE·LIFT, 팔 5축, 오프셋 ±0.08 rad)
#   pose_hat = 정답 + 인식 오차 흉내 (편향 σ 6 mm/에피소드 + 잡음 σ 2 mm/스텝)  [가정]
#   expert-check 도 같은 클래스 (zero_agent → π = 0 → IK 단독 성능)
# ---------------------------------------------------------------------------
RES_ALPHA = float(os.environ.get("NZ2_RES_ALPHA", "0.10"))     # 최대 속도 비율. 0.10 으로 시작 → 성공률 보고 0.20
W_RES = -0.01                                                   # λ‖tanh π‖² [가정]
RES_TERM = C.ACTION_TERM


def _res_term(env):
    return env.action_manager.get_term(RES_TERM)


def obs_res_phase(env):
    """IK 상태기계 단계 one-hot (9)"""
    return _res_term(env).phase_onehot()


def obs_res_offset(env):
    """누적 잔차 오프셋 [rad] (팔 5축) / 한계 → −1~1"""
    from arm_practice.tasks import nz2_vispick_residual as RS     # 실행 시점 import
    return _res_term(env).res_off / RS.RES_MAX_RAD


def obs_res_pose_err(env):
    """IK 가 쓰는 퍽 위치 오차 [m] (비평가 전용 특권 정보)"""
    return _res_term(env).pose_err()


def rew_res_l2(env):
    """잔차 크기 ‖tanh(π)·gate‖² (팔 5축) — 필요할 때만 보정하게"""
    return torch.sum(_res_term(env).res_cmd ** 2, dim=1)


def _grip_closed_fully(env):
    """그리퍼가 끝까지 닫힘 = 퍽을 물지 못함 (헛잡기)"""
    t = _res_term(env)
    g = t._asset.data.joint_pos.torch[:, t._joint_ids][:, 5]
    return g <= float(t._lo[5]) + 0.01


def term_bin_success(env):
    """(11/11) 적재통 성공: 상태기계 VERIFY 통과 (퍽이 컵 안·세움·높이 맞음, 손가락 빠짐)"""
    done = _res_term(env).bin_ok.clone()
    vp(env).n_succ += int(done.sum())
    return done


def term_bin_fail(env):
    """(11/11) VERIFY 실패 → 바로 끝냄 (시간 초과까지 기다리지 않음)"""
    return _res_term(env).bin_fail.clone()


def term_bin_skip(env):
    """(11/11) 계획 단계에서 도달 불가(SKIP) → 바로 끝냄"""
    from arm_practice.tasks import nz2_vispick_residual as RS
    return _res_term(env).phase == RS.P_SKIP


def term_success_grip(env):
    """성공 = 파지 + 40 mm 들림 0.5 s 유지 + 다른 퍽 무사 + 열림 폭 검사(끝까지 닫히지 않음)"""
    st = vp(env)
    s = _success_now(env) & ~_grip_closed_fully(env)
    st.hold = torch.where(s, st.hold + 1, torch.zeros_like(st.hold))
    done = st.hold >= HOLD_STEPS
    st.n_succ += int(done.sum())
    return done


@configclass
class NZ2VisPickResidualActionCfg(C.NZ2SmoothJointPositionActionCfg):
    class_type: type | str = "arm_practice.tasks.nz2_vispick_residual:NZ2VisPickResidualAction"
    residual_alpha: float = RES_ALPHA


@configclass
class NZ2VisPickResidualCfg(NZ2VisPickCfg):
    def __post_init__(self):
        super().__post_init__()
        a = self.actions.arm_action
        self.actions.arm_action = NZ2VisPickResidualActionCfg(
            asset_name=a.asset_name, joint_names=a.joint_names, preserve_order=True, use_default_offset=a.use_default_offset,
            scale=a.scale, clip=a.clip, mode=a.mode, alpha=a.alpha, max_lead=a.max_lead, max_velocity=a.max_velocity,
            residual_alpha=RES_ALPHA)
        # 관측: 정책 = 영상 2대 + 단계 one-hot + 누적 오프셋 (+ 기존 관절·명령색), 비평가 = + 인식 오차
        self.observations.policy.res_phase = ObservationTermCfg(func=obs_res_phase)
        self.observations.policy.res_offset = ObservationTermCfg(func=obs_res_offset)
        self.observations.critic.res_phase = ObservationTermCfg(func=obs_res_phase)
        self.observations.critic.res_offset = ObservationTermCfg(func=obs_res_offset)
        self.observations.critic.res_pose_err = ObservationTermCfg(func=obs_res_pose_err)
        # 보상: IK 가 접근을 맡으므로 접근 유도항 제거. 파지·들기·유지·성공 + 실패 벌점 + 잔차 크기 벌점
        R = self.rewards
        for k in ("reach", "reach_fine", "open_far", "close_near", "action_rate", "joint_vel", "action_l2"):
            if hasattr(R, k):
                setattr(R, k, None)
        for k in ("action_rate", "joint_vel"):
            if hasattr(self.curriculum, k):
                setattr(self.curriculum, k, None)
        R.res_l2 = RewardTermCfg(func=rew_res_l2, weight=W_RES)
        # 성공 판정에 열림 폭 검사 추가 (이름은 success 유지 → success_bonus·단계 판정이 그대로 씀)
        self.terminations.success = TerminationTermCfg(func=term_success_grip)
        if TO_BIN:                                    # (11/11) 성공 = 컵 안에 세워 놓고 팔이 빠진 뒤 확인 통과
            self.terminations.success = TerminationTermCfg(func=term_bin_success)
            self.terminations.bin_fail = TerminationTermCfg(func=term_bin_fail)
            self.terminations.bin_skip = TerminationTermCfg(func=term_bin_skip)
        self.scene.num_envs = int(os.environ.get("NZ2_RES_ENVS", str(self.scene.num_envs)))


@configclass
class NZ2VisPickResidualStateCfg(NZ2VisPickResidualCfg):
    nz2_cameras: bool = False                 # 카메라 없음: expert-check(IK 단독 성공률) 용


def _res_actor(cnn: bool):
    """잔차 정책: 마지막 층 0 초기화 모델, 초기 std 0.05, 관측 정규화 끔"""
    cls = "arm_practice.tasks.nz2_vispick_residual_model:ZeroLastCNNModel"
    dist = RslRlCNNModelCfg.GaussianDistributionCfg(init_std=0.05)
    m = RslRlCNNModelCfg(
        cnn_cfg=RslRlCNNModelCfg.CNNCfg(output_channels=[16, 32, 32, 32], kernel_size=[5, 3, 3, 3],
                                        stride=[2, 2, 2, 2], activation="relu"),
        hidden_dims=[256, 128], activation="elu", obs_normalization=False, distribution_cfg=dist)
    m.class_name = cls                         # rsl_rl resolve_callable("모듈:클래스")
    return m


@configclass
class NZ2VisPickResidualPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = STEPS_PER_ENV
    max_iterations = 600
    save_interval = 25
    experiment_name = "nz2_vispick_residual"
    obs_groups = {"actor": ["policy", "wrist_cam", "top_cam"], "critic": ["critic"]}
    actor = _res_actor(True)
    critic = RslRlMLPModelCfg(hidden_dims=[256, 128, 64], activation="elu", obs_normalization=False)
    algorithm = RslRlPpoAlgorithmCfg(value_loss_coef=1.0, use_clipped_value_loss=True, clip_param=0.2, entropy_coef=0.0,
                                     num_learning_epochs=5, num_mini_batches=4, learning_rate=1.0e-4, schedule="fixed",
                                     gamma=0.99, lam=0.95, desired_kl=0.01, max_grad_norm=1.0)
