"""NZ2 팔 Reach 학습 설정 v3 (delta 행동 + 연속 정지 보상) — 전체 로봇(차체 고정) + 팔·그리퍼만 학습 (Isaac Lab 3.0, Isaac Sim 6.1)

설치 위치: ~/manipulator-isaac61-lyrical-practice/src/arm_practice/tasks/nz2_reach_cfg.py
  같은 폴더에 nz2_common.py 필요. 등록은 __init__.py 의 NZ2-Reach-Arm.
로봇: R12_NO_USB_URDF_USD/urdf/r12_arm_train_pos/r12_arm_train/r12_arm_train.usda (그리퍼 mimic 확인 완료)
물리엔진: 반드시 physics=isaacsim_physx (손가락 연동이 PhysX mimic)

v1(2026-11-03, 1000회, 성공 48 %) 재생 때 심하게 떨린 원인과 수정
  1. 성공 = 에피소드 종료 → 목표에 닿는 순간 home 으로 리셋 → "닿고-튕겨 돌아가기" 반복.
     또 '멈춰서 버티기'를 한 번도 배우지 못함.          → 성공 종료 삭제, 목표 근처에서 '정지 유지' 보상
  2. 행동 1 = 1 rad 절대 목표, 변화율 벌점 −0.005(보상의 0.1 % 수준) → 출력이 ±로 흔들려도 손해가 거의 없음
                                                       → 행동 EMA + 속도 제한(nz2_common), 변화율 벌점 10배
  3. 정책이 필터 상태를 모름                           → 관측에 '실제 보낸 목표 − 현재 각' 추가
  관측 차원이 바뀌었으므로 v1 체크포인트로 이어 학습(resume) 불가 → 처음부터 학습

v2(2026-11-03 14-48-10, 100 env × 1000회) 결과와 v3 수정
  1. 결정적 재생에서도 손끝 떨림 → 절대 목표 + EMA + 속도 제한 고리의 리밋 사이클
                                                       → nz2_common ACTION_MODE = "delta" (a = 관절 속도, a=0 정지)
  2. success(20 mm 안 & 5축 속도 노름 ≤ 0.2 rad/s) 유지 비율 최대 0.5 % → 탐색 노이즈 때문에 조건이 거의 불가능,
     보상 신호가 없어 정밀 정지를 못 배움        → 연속값 (1 − tanh(d/20 mm))·exp(−‖q̇‖/0.5)
                                                    + near20(20 mm 안 시간 비율, 기록용 겸 약한 보상)
  3. 실전 목표 x 0.25 m 는 차체 앞 끝(0.22 m)에서 30 mm → link3·손가락 충돌 종료 17 %
                                                       → TARGET_STAGE1 x 0.28~0.38, z 0.12~0.20 (Pick 퍽 범위와 맞춤)
                                                          (nz2_arm.json 의 target_range 는 더 이상 쓰지 않음)
  4. 목표 범위 확대(250회)와 벌점 10배(300회)가 붙어 있어 성능 붕괴 → 벌점 강화를 500회로 분리
  관측 차원은 v2 와 같지만 행동 의미가 바뀌었으므로 v2 체크포인트로 resume 하지 말 것
"""
import math
import os

import torch

from isaaclab.managers import CurriculumTermCfg, RewardTermCfg, SceneEntityCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import combine_frame_transforms

from arm_practice.config import load_robot_config
from arm_practice.tasks import nz2_common as C
from arm_practice.tasks.ppo_cfg import OwnArmPPORunnerCfg
from arm_practice.tasks.reach_cfg import OwnArmReachCfg

# ===== 상수 =====
NUM_ENVS = 64                                    # RTX 5060 8 GB 기준 시작값
ACTION_SCALE_RAD = 1.0                           # 행동 1 = home 에서 1 rad (관절 한계 전체를 쓸 수 있게)
LIMIT_MARGIN_RAD = math.radians(2.0)
SUCCESS_TOL_M = 0.02                             # 도달 판정 거리 (학습되면 0.01 로)
HOLD_SPEED_SCALE_RAD_S = 0.5                     # [가정] 정지 보상 exp(−‖q̇‖/이 값) (팔 5축 노름)
W_NEAR = 1.0                                     # near20: 20 mm 안이면 1 → Episode_Reward/near20 = 시간 비율
W_POS_L2 = -1.0
W_TCP_TANH_COARSE, TCP_TANH_STD_COARSE_M = 1.0, 0.30
W_TCP_TANH_FINE, TCP_TANH_STD_FINE_M = 2.0, 0.05
W_HOLD = 3.0                                     # [가정] 목표 근처에서 멈춰 있을수록 큰 연속 보상 (최대 3)
TARGET_STAGE0 = {"x": (0.22, 0.30), "y": (-0.05, 0.05), "z": (0.28, 0.36)}   # 덱 위 높은 곳 (쉬움)
TARGET_STAGE1 = {"x": (0.28, 0.38), "y": (-0.08, 0.08), "z": (0.12, 0.20)}   # 선반 퍽 높이·깊이 (실전)
CURRICULUM_SWITCH_STEPS = 24 * 250               # 목표 범위 확대 시점 (약 250회차)
SMOOTH_CURRICULUM_STEPS = 24 * 500               # 변화율·속도 벌점 강화 시점 (약 500회차, 범위 확대와 분리)
ENTROPY_COEF = 0.005
ARM_JOINTS = ["arm_joint[1-5]"]


def tcp_position_error(env, command_name: str, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """명령 위치와 TCP 사이 거리 [m]"""
    asset = env.scene[asset_cfg.name]
    command = env.command_manager.get_command(command_name)
    des_pos_w, _ = combine_frame_transforms(asset.data.root_pos_w.torch, asset.data.root_quat_w.torch, command[:, :3])
    return torch.linalg.norm(C.tcp_pos_w(env, asset_cfg) - des_pos_w, dim=1)


def tcp_position_tanh(env, command_name: str, asset_cfg: SceneEntityCfg, std: float) -> torch.Tensor:
    return 1.0 - torch.tanh(tcp_position_error(env, command_name, asset_cfg) / std)


def tcp_hold(env, command_name: str, asset_cfg: SceneEntityCfg, joint_cfg: SceneEntityCfg,
             tolerance: float, speed_scale: float) -> torch.Tensor:
    """연속 정지 보상 (0~1): 가까울수록 × 느릴수록 큼. 탐색 노이즈가 있어도 기울기가 살아 있다"""
    d = tcp_position_error(env, command_name, asset_cfg)
    qd = env.scene[joint_cfg.name].data.joint_vel.torch[:, joint_cfg.joint_ids]
    return (1.0 - torch.tanh(d / tolerance)) * torch.exp(-torch.linalg.norm(qd, dim=1) / speed_scale)


def tcp_near(env, command_name: str, asset_cfg: SceneEntityCfg, tolerance: float) -> torch.Tensor:
    """TCP 가 tolerance 안이면 1. weight 1 → TensorBoard Episode_Reward/near20 = 20 mm 안 시간 비율"""
    return (tcp_position_error(env, command_name, asset_cfg) <= tolerance).float()


def target_range_curriculum(env, env_ids, command_name: str, switch_steps: int, stage0: dict, stage1: dict):
    stage = 0 if env.common_step_counter < switch_steps else 1
    r = env.command_manager.get_term(command_name).cfg.ranges
    s = stage0 if stage == 0 else stage1
    r.pos_x, r.pos_y, r.pos_z = tuple(s["x"]), tuple(s["y"]), tuple(s["z"])
    return float(stage)


@configclass
class NZ2ArmReachCfg(OwnArmReachCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = NUM_ENVS
        c = load_robot_config(os.environ["ARM_CONFIG"])

        # ---- 떨림 방지: 행동 EMA + 속도 제한, 변화율·속도 벌점, 관측에 실제 목표 ----
        C.setup_smooth_action(self, c, ACTION_SCALE_RAD, LIMIT_MARGIN_RAD)
        C.setup_smoothness(self, SMOOTH_CURRICULUM_STEPS)

        # ---- 보상: TCP(arm_link5 + 105 mm) 기준 ----
        tcp = C.tcp_cfg()
        arm = SceneEntityCfg("robot", joint_names=ARM_JOINTS)
        p = {"command_name": "ee_pose", "asset_cfg": tcp}
        self.rewards.end_effector_position_tracking.func = tcp_position_error
        self.rewards.end_effector_position_tracking.weight = W_POS_L2
        self.rewards.end_effector_position_tracking.params = dict(p)
        self.rewards.tcp_tanh = RewardTermCfg(
            func=tcp_position_tanh, weight=W_TCP_TANH_COARSE, params={**p, "std": TCP_TANH_STD_COARSE_M})
        self.rewards.tcp_tanh_fine = RewardTermCfg(
            func=tcp_position_tanh, weight=W_TCP_TANH_FINE, params={**p, "std": TCP_TANH_STD_FINE_M})
        # 성공 = 종료 아님. 도달 후 멈춰 있는 동안 계속 보상 (v1 의 '닿고 리셋' 반복 제거)
        self.terminations.success = None
        self.rewards.success = RewardTermCfg(
            func=tcp_hold, weight=W_HOLD,
            params={**p, "joint_cfg": arm, "tolerance": SUCCESS_TOL_M, "speed_scale": HOLD_SPEED_SCALE_RAD_S})
        self.rewards.near20 = RewardTermCfg(func=tcp_near, weight=W_NEAR, params={**p, "tolerance": SUCCESS_TOL_M})

        # ---- 목표 범위 커리큘럼 ----
        stage1 = TARGET_STAGE1
        r = self.commands.ee_pose.ranges
        r.pos_x, r.pos_y, r.pos_z = TARGET_STAGE0["x"], TARGET_STAGE0["y"], TARGET_STAGE0["z"]
        self.curriculum.target_range = CurriculumTermCfg(
            func=target_range_curriculum,
            params={"command_name": "ee_pose", "switch_steps": CURRICULUM_SWITCH_STEPS,
                    "stage0": TARGET_STAGE0, "stage1": stage1})

        # ---- 고정 관절·자기충돌·접촉(팔 링크 + 손가락: Reach 에서는 무엇에든 닿으면 안 됨) ----
        C.setup_robot_common(self.scene.robot)
        C.add_net_contact_terms(self, list(C.LINK_PATHS))
        C.add_contact_termination_penalty(self)
        C.add_finger_friction_dr(self)

        # ---- 재생용: NZ2_PLAY=1 이면 관측 노이즈 끄기 (실기 성능에 가까운 확인) ----
        if os.environ.get("NZ2_PLAY") == "1":
            self.observations.policy.enable_corruption = False


@configclass
class NZ2ArmPPORunnerCfg(OwnArmPPORunnerCfg):
    experiment_name = "nz2_arm_reach"
    max_iterations = 1000

    def __post_init__(self):
        self.algorithm.entropy_coef = ENTROPY_COEF
