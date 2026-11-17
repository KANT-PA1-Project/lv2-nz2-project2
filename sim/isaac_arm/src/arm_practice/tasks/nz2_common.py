"""NZ2 팔 학습 공통 부품 (Reach·Pick 이 같이 씀) — Isaac Lab 3.0.0 / Isaac Sim 6.1, 물리엔진 PhysX

설치 위치: ~/manipulator-isaac61-lyrical-practice/src/arm_practice/tasks/nz2_common.py
(R12_NO_USB_URDF_USD/isaac/ 에 같은 파일 보관)

담고 있는 것
  1. 떨림 방지 행동(NZ2SmoothJointPositionAction): 정책 출력 → 관절 목표를
       ① 저역통과(EMA)  ② 한 스텝 최대 변화량 제한(= 관절 속도 한계)  으로 부드럽게 만든다.
     실기 arm_bridge 도 목표 각을 속도 제한으로 따라가므로 시뮬·실기 동작이 비슷해진다.
  2. 조향·바퀴·손가락 고정 액추에이터, 자기충돌 설정
  3. 링크별 접촉 센서 + 감점·종료 항목
  4. 손가락 마찰 DR
API 대조: ~/pa-course/IsaacLab-6.1 소스 (JointPositionAction, string_utils.resolve_matching_names_values,
  ContactSensorCfg, mdp.undesired_contacts / illegal_contact, modify_reward_weight)
"""
from __future__ import annotations

import math
import re
from collections.abc import Sequence

import torch

import isaaclab.envs.mdp as mdp
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.managers import EventTermCfg, ObservationTermCfg, RewardTermCfg, SceneEntityCfg, TerminationTermCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import quat_apply
from isaaclab_physx.sim.schemas import PhysxArticulationRootPropertiesCfg

# ===== 로봇 USD 안의 링크 경로 (GUI 가져오기 구조: 링크가 계층으로 중첩됨) =====
ARM_PATH = "Geometry/base_link/arm_link1/arm_link2"
LINK_PATHS = {
    "arm_link2": ARM_PATH,
    "arm_link3": ARM_PATH + "/arm_link3",
    "arm_link4": ARM_PATH + "/arm_link3/arm_link4",
    "arm_link5": ARM_PATH + "/arm_link3/arm_link4/arm_link5",
    "finger_l": ARM_PATH + "/arm_link3/arm_link4/arm_link5/gripper_left_finger",
    "finger_r": ARM_PATH + "/arm_link3/arm_link4/arm_link5/gripper_right_finger",
}
BASE_LINK_PATH = "Geometry/base_link"          # 차체 강체 (손가락↔차체 접촉 필터용)
ACTION_TERM = "arm_action"

# ===== TCP =====
# 질량 없는 고정 링크(gripper_tcp)는 강체가 아니라 body 로 못 찾음 → arm_link5 + 오프셋
TCP_BODY = "arm_link5"
TCP_OFFSET_M = (0.105, 0.0, 0.0)               # 롤 축 → 패드 중심 (URDF gripper_tcp 와 같은 값)
L5_BODY = "arm_link5"                           # j5 원점 몸체 (FK 자기검사)
FINGER_BODIES = ["gripper_left_finger", "gripper_right_finger"]
FINGER_BODY_REGEX = "gripper_.*_finger"
FINGER_MIMIC_REGEX = ["gripper_(left|right)_joint"]
ARM_JOINT_NAMES = ["arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"]
GRIP_JOINT = "gripper_joint"

# ===== (11/12) 최종 URDF 전체 로봇 (ARM_CONFIG json 의 robot = "nz2_full") =====
#   USD: models/nz2_full/nz2_full_robot_light.usda (assembly.usd + 질량 ×0.1·R12 관절 부호·펌웨어 한계·집게 mimic·root_joint)
#   링크 이름·TCP·손가락은 json 에서 읽음. 관절 값 규약은 R12 와 같음 (q_USD = q_R12) → IK·상태기계 그대로
import json as _json
import os as _os
ROBOT_CFG: dict = {}
try:
    _p = _os.environ.get("ARM_CONFIG")
    if _p and _os.path.isfile(_os.path.expanduser(_p)):
        ROBOT_CFG = _json.load(open(_os.path.expanduser(_p), encoding="utf-8"))
except Exception as _e:                          # json 이 깨졌으면 load_robot_config 가 다시 오류를 냄
    print(f"[NZ2] ARM_CONFIG 읽기 실패: {_e}")
FULL_ROBOT = ROBOT_CFG.get("robot") == "nz2_full"
if FULL_ROBOT:
    LINK_PATHS = dict(ROBOT_CFG["link_paths"])
    BASE_LINK_PATH = ROBOT_CFG.get("base_link_path", "base_link")
    TCP_BODY = ROBOT_CFG["tcp_link"]
    TCP_OFFSET_M = tuple(ROBOT_CFG["tcp_offset_m"])
    L5_BODY = LINK_PATHS["arm_link5"]
    FINGER_BODIES = list(ROBOT_CFG["finger_bodies"])
    FINGER_BODY_REGEX = "(" + "|".join(re.escape(b) for b in FINGER_BODIES) + ")"
    FINGER_MIMIC_REGEX = [re.escape(j) for j in ROBOT_CFG.get("finger_mimic_joints", [])]
    ARM_JOINT_NAMES = list(ROBOT_CFG["joint_names"][:5])
    GRIP_JOINT = ROBOT_CFG["joint_names"][5]
    print(f"[NZ2] 전체 로봇 모드: {ROBOT_CFG.get('usd_path')} / 관절 {ROBOT_CFG['joint_names']} / TCP {TCP_BODY}+{TCP_OFFSET_M}")
ARM_JOINT_REGEX = [re.escape(n) for n in ARM_JOINT_NAMES]

# ===== 떨림 방지 =====
ACTION_MODE = "delta"                          # v3: 행동 = 관절 속도 명령(a=0 이면 정지). "absolute" = v2 방식(떨림 확인됨)
SMOOTH_ALPHA = 0.5                             # [가정] EMA 가중치 (1 = 필터 없음). 30 Hz 에서 시정수 약 48 ms
MAX_LEAD_RAD = 0.15                            # [가정] 목표가 현재각보다 앞설 수 있는 최대값 (막힘 시 와인드업·충격 방지)
# 한 스텝 최대 변화 = 관절 속도 한계(nz2_arm.json velocity) × step_dt(1/30 s)
#   arm 1.0 rad/s → 0.033 rad/step(1.9°), gripper 3.0 rad/s → 0.1 rad/step
# 행동 변화율·관절 속도 벌점: 시작은 약하게(탐색), CURRICULUM 이후 강하게(떨림 제거)
W_ACTION_RATE = (-0.005, -0.05)                # (시작, 최종)  [가정]
W_JOINT_VEL = (-0.001, -0.01)                  # (시작, 최종)  [가정]

# ===== 고정 관절·충돌 =====
STEER_HOLD = dict(stiffness=40.0, damping=4.0)   # [가정] 조향 위치 유지
WHEEL_HOLD = dict(stiffness=0.0, damping=4.0)    # [가정] 바퀴 속도 0 유지
THRESH_PENALTY_N = 0.5                           # [가정] 감점 시작 접촉력
THRESH_TERMINATE_N = 2.0                         # [가정] 종료 접촉력
W_CONTACT = -2.0                                 # [가정] 링크 1개 접촉당 감점
W_CONTACT_TERMINATION = -10.0                    # [가정] 충돌로 끝날 때 1회 벌점
FINGER_FRICTION = (0.4, 0.8)                     # nz2_dr_experiment.md 범위


# ---------------------------------------------------------------------------
# 1. 떨림 방지 행동 (클래스 본체: nz2_smooth_action.py)
# ---------------------------------------------------------------------------
@configclass
class NZ2SmoothJointPositionActionCfg(mdp.JointPositionActionCfg):
    # 문자열로 지정 → 실제 클래스는 환경 생성 때(Isaac Sim 시작 후) 불러온다. 직접 import 하면 pxr 충돌
    class_type: type | str = "arm_practice.tasks.nz2_smooth_action:NZ2SmoothJointPositionAction"
    mode: str = ACTION_MODE
    alpha: float = SMOOTH_ALPHA
    max_lead: float = MAX_LEAD_RAD
    max_velocity: dict[str, float] = {".*": 1.0}     # [rad/s] 관절 이름(정규식) → 최대 속도


def setup_smooth_action(env_cfg, c: dict, scale_rad: float, margin_rad: float):
    """arm_action 을 떨림 방지 행동으로 교체. c = load_robot_config(ARM_CONFIG) 결과
    delta 모드에서는 scale_rad 를 쓰지 않는다 (속도 = max_velocity). absolute 모드 비교용으로 인자 유지"""
    names = [re.escape(n) for n in c["joint_names"]]
    env_cfg.actions.arm_action = NZ2SmoothJointPositionActionCfg(
        asset_name="robot", joint_names=names, preserve_order=True, use_default_offset=True,
        scale={n: scale_rad for n in names},
        clip={n: (lo + margin_rad, hi - margin_rad) if hi - lo > 4 * margin_rad else (lo, hi)
              for n, lo, hi in zip(names, c["lower"], c["upper"], strict=True)},
        mode=ACTION_MODE, alpha=SMOOTH_ALPHA, max_lead=MAX_LEAD_RAD,
        max_velocity={n: v for n, v in zip(names, c["velocity"], strict=True)},
    )


def applied_target_rel(env, action_name: str = ACTION_TERM) -> torch.Tensor:
    """관측: 실제로 보낸 관절 목표 − 현재 관절 위치 [rad] (필터 내부 상태를 정책이 알게)"""
    term = env.action_manager.get_term(action_name)
    q = term._asset.data.joint_pos.torch[:, term._joint_ids]
    return term.processed_actions - q


def setup_smoothness(env_cfg, curriculum_steps: int):
    """행동 변화율·관절 속도 벌점 + 커리큘럼(시작 약하게 → 최종 강하게), 관측에 실제 목표 추가"""
    env_cfg.rewards.action_rate.weight = W_ACTION_RATE[0]
    env_cfg.rewards.joint_vel.weight = W_JOINT_VEL[0]
    env_cfg.curriculum.action_rate.params = {
        "term_name": "action_rate", "weight": W_ACTION_RATE[1], "num_steps": curriculum_steps}
    env_cfg.curriculum.joint_vel.params = {
        "term_name": "joint_vel", "weight": W_JOINT_VEL[1], "num_steps": curriculum_steps}
    env_cfg.observations.policy.applied_target = ObservationTermCfg(func=applied_target_rel)


# ---------------------------------------------------------------------------
# 2. TCP
# ---------------------------------------------------------------------------
def tcp_pos_w(env, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """월드 좌표 TCP 위치 [m], shape [N, 3] = arm_link5 위치 + arm_link5 회전 · 오프셋"""
    asset = env.scene[asset_cfg.name]
    i = asset_cfg.body_ids[0]
    pos = asset.data.body_pos_w.torch[:, i]
    quat = asset.data.body_quat_w.torch[:, i]
    off = torch.tensor(TCP_OFFSET_M, device=pos.device, dtype=pos.dtype).expand_as(pos)
    return pos + quat_apply(quat, off)


def tcp_cfg() -> SceneEntityCfg:
    return SceneEntityCfg("robot", body_names=[re.escape(TCP_BODY)])


# ---------------------------------------------------------------------------
# 3. 고정 관절·자기충돌·접촉
# ---------------------------------------------------------------------------
def setup_robot_common(robot):
    """조향·바퀴 고정, 손가락은 mimic 이 움직이도록 게인 0, 자기충돌 ON"""
    robot.actuators["steer_hold"] = ImplicitActuatorCfg(joint_names_expr=[".*_steer_joint"], **STEER_HOLD)
    robot.actuators["wheel_hold"] = ImplicitActuatorCfg(joint_names_expr=[".*_wheel_joint"], **WHEEL_HOLD)
    if FINGER_MIMIC_REGEX:
        robot.actuators["finger_mimic"] = ImplicitActuatorCfg(
            joint_names_expr=list(FINGER_MIMIC_REGEX), stiffness=0.0, damping=0.0)
    mirror = ROBOT_CFG.get("finger_mirror_joint") if FULL_ROBOT else None
    if mirror:            # (11/12) 오른쪽 집게: 왼쪽과 같은 이득으로 위치 구동 (목표는 행동 클래스가 왼쪽 목표를 복사)
        robot.actuators["finger_mirror"] = ImplicitActuatorCfg(
            joint_names_expr=[re.escape(mirror)], stiffness=ROBOT_CFG["stiffness"][5], damping=ROBOT_CFG["damping"][5],
            joint_effort_limit=ROBOT_CFG["effort"][5], joint_velocity_limit=ROBOT_CFG["velocity"][5])
    robot.spawn.activate_contact_sensors = True
    # 차체 고정은 USD root_joint 가 담당. fix_root_link=True 는 RuntimeError (2026-11-03 확인) → 쓰지 않음
    robot.spawn.articulation_props = PhysxArticulationRootPropertiesCfg(
        enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=1)


def add_net_contact_terms(env_cfg, keys: Sequence[str]):
    """링크별 접촉 센서(모든 상대) + 감점 + 종료. 접촉 상대를 가리지 않으므로 '닿으면 안 되는 링크'에만 쓴다"""
    for key in keys:
        name = f"contact_{key}"
        setattr(env_cfg.scene, name, ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/Robot/" + LINK_PATHS[key], history_length=3))
        cfg = SceneEntityCfg(name)
        setattr(env_cfg.rewards, f"{name}_pen", RewardTermCfg(
            func=mdp.undesired_contacts, weight=W_CONTACT,
            params={"sensor_cfg": cfg, "threshold": THRESH_PENALTY_N}))
        setattr(env_cfg.terminations, f"{name}_stop", TerminationTermCfg(
            func=mdp.illegal_contact, params={"sensor_cfg": cfg, "threshold": THRESH_TERMINATE_N}))


def add_contact_termination_penalty(env_cfg):
    """contact_*_stop 종료(충돌)에 1회 큰 벌점 → '빨리 부딪혀 끝내기' 방지"""
    env_cfg.rewards.contact_termination = RewardTermCfg(
        func=mdp.is_terminated_term, weight=W_CONTACT_TERMINATION, params={"term_keys": "contact_.*_stop"})


def add_finger_friction_dr(env_cfg):
    env_cfg.events.finger_friction = EventTermCfg(
        func=mdp.randomize_rigid_body_material, mode="startup",
        params={"asset_cfg": SceneEntityCfg("robot", body_names=FINGER_BODY_REGEX),
                "static_friction_range": FINGER_FRICTION, "dynamic_friction_range": FINGER_FRICTION,
                "restitution_range": (0.0, 0.0), "num_buckets": 64, "make_consistent": True})
