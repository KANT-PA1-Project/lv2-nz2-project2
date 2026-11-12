"""NZ2 팔 Pick 학습 설정 — 선반(박스) 위 퍽 집어 들기 (Isaac Lab 3.0, Isaac Sim 6.1, PhysX)

설치 위치: ~/manipulator-isaac61-lyrical-practice/src/arm_practice/tasks/nz2_pick_cfg.py
  같은 폴더에 nz2_common.py, nz2_reach_cfg.py 필요. 등록은 __init__.py 의 NZ2-Pick-Arm.

치수 출처
  - maze 폴더(maze.usda + geo.usd): 벽만 있음. 벽판 높이 380 mm, 마운트 포함 390 mm, PLA 마찰 0.35/0.30.
    선반·퍽 형상은 들어 있지 않다 (2026-11-03 USD 전체 prim 확인)
  - 선반(박스) 치수는 mission.pdf v5 기준: 작은 박스 205×205×70 mm(시작 6 + L1·L3), 큰 박스 410×410×70 mm ×4(L2)
  - 퍽: 직육면체 30×30×60 mm, 원기둥 Ø30×60 mm (프로젝트 기준 정보)
학습 장면 (로봇 기준, base_link 원점 = 차체 중심 바닥)
  - 선반: 깊이 205 × 폭 410 × 높이 70 mm 정적(kinematic) 상자. 앞면 = 차체 앞(0.22 m) + 간격 10~30 mm DR,
    좌우 ±20 mm, yaw ±3° DR. 폭은 410 으로 잡아 큰 박스까지 보수적으로 덮는다(작은 박스보다 손이 지나갈 곳이 좁음)
  - 퍽 3종(매 리셋 무작위): ① 직육면체 세움 ② 직육면체 누움 ③ 원기둥 세움 — yaw 무작위
    위치: 선반 앞면에서 깊이 40~100 mm(1단계) → 40~165 mm(2단계), 좌우 ±40 → ±70 mm
    (205 박스 3×3 격자 −60/0/+60 → 앞면에서 42/102/162 mm, 시작 박스 퍽 X±40 을 포함)
  - 쓰지 않는 퍽은 로봇 뒤 바닥(팔이 못 닿는 곳)에 둔다
보상: 다가가기 → 열고 접근 → 닫기 → 양손가락 접촉(잡기) → 들기 40 mm 유지
실격 대응: 선반·차체에 손가락/팔 접촉 = 감점, 2 N 초과 = 즉시 종료 + 벌점. 퍽을 밀거나 넘어뜨리면 종료 + 벌점
"""
import math
import os

import torch

import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObjectCfg
from isaaclab.managers import (CurriculumTermCfg, EventTermCfg, ObservationTermCfg, RewardTermCfg,
                               SceneEntityCfg, TerminationTermCfg)
from isaaclab.sensors import ContactSensorCfg
from isaaclab.sim import RigidBodyMaterialCfg
from isaaclab.utils import configclass
from isaaclab.utils import math as math_utils

from arm_practice.config import load_robot_config
from arm_practice.tasks import nz2_common as C
from arm_practice.tasks.nz2_reach_cfg import NZ2ArmPPORunnerCfg, NZ2ArmReachCfg

# ===== 경기장 치수 [m] =====
SHELF_SIZE = (0.205, 0.410, 0.070)          # (깊이 x, 폭 y, 높이 z)  mission.pdf v5 박스 높이 70 mm
SHELF_FRICTION = 0.5                        # [가정] 박스 재질 미확인
CHASSIS_FRONT_X = 0.22                      # 차체 충돌 상자 앞 끝 (URDF 20 mm 격자 기준, 실물 0.20)
SHELF_GAP = (0.010, 0.030)                  # 차체 앞 ↔ 선반 앞면 간격 DR (접근 정지 오차)
SHELF_LAT = (-0.02, 0.02)                   # 선반 좌우 오차
SHELF_YAW = (-math.radians(3.0), math.radians(3.0))
PUCK_BOX = (0.03, 0.03, 0.06)               # 직육면체 퍽 (x, y, z=길이)
PUCK_CYL_R, PUCK_CYL_H = 0.015, 0.06        # 원기둥 퍽
PUCK_MASS = (0.012, 0.018)                  # [가정] 13~18 g 부근 DR
PUCK_FRICTION = (0.30, 0.45)                # maze PLA 0.35/0.30 주변 DR
PUCK_RESTITUTION = (0.1, 0.4)               # maze PLA 0.4 이하
MODE_PROBS = (1 / 3, 1 / 3, 1 / 3)          # ① 직육면체 세움 ② 직육면체 누움 ③ 원기둥 세움
DEPTH_STAGE = ((0.04, 0.10), (0.04, 0.165))  # 선반 앞면에서 퍽 중심까지 (1단계, 2단계)
LAT_STAGE = ((-0.04, 0.04), (-0.07, 0.07))
PARK_BOX = (-0.60, 0.45, 0.030)             # 안 쓰는 퍽 보관 위치 (로봇 뒤 바닥)
PARK_CYL = (-0.60, -0.45, 0.030)
# ===== 잡기 판정 =====
GRASP_Z_MIN = SHELF_SIZE[2] + 0.020         # [가정] TCP 아래 손가락 끝 약 12 mm → 선반 위 20 mm 보다 낮게 잡지 않음
GRIP_FORCE_N = 0.2                          # [가정] 손가락-퍽 접촉 판정
NEAR_M, FAR_M = 0.02, 0.05                  # 닫기 시작 / 열어 둘 거리
LIFT_M = 0.04                               # 들기 목표 높이 (선반 접촉 실격 방지 여유)
PUSH_LIMIT_M = 0.04                         # 들지 않은 퍽이 이만큼 밀리면 종료
TILT_LIMIT = math.cos(math.radians(35.0))   # 들지 않은 퍽이 35° 이상 기울면 종료
# ===== 보상 가중치 [가정] =====
W = dict(reach=1.0, reach_fine=1.5, open_far=0.3, close_near=0.5, grasp=2.0, lift=5.0, success=5.0,
         puck_fail=-10.0)
REACH_STD, REACH_FINE_STD = 0.10, 0.02
NUM_ENVS = 128                              # [가정] 8 GB 에서 64/128/256 비교 후 결정
EPISODE_S = 10.0
STAGE_SWITCH_STEPS = 24 * 400               # 퍽 범위 확대 (약 400회차)
SMOOTH_CURRICULUM_STEPS = 24 * 400
ACTION_SCALE_RAD = 1.0
LIMIT_MARGIN_RAD = math.radians(2.0)


# ---------------------------------------------------------------------------
# 환경별 상태 (퍽 종류, 리셋 때 퍽 위치·축) — 첫 리셋 전에도 관측 크기 계산이 되도록 지연 생성
# ---------------------------------------------------------------------------
class _PickState:
    def __init__(self, n: int, device):
        self.mode = torch.zeros(n, dtype=torch.long, device=device)
        self.init_pos = torch.zeros(n, 3, device=device)
        self.init_axis = torch.zeros(n, 3, device=device)
        self.init_axis[:, 2] = 1.0
        self.stage = 0


def pick_state(env) -> _PickState:
    st = getattr(env, "_nz2_pick", None)
    if st is None:
        st = _PickState(env.num_envs, env.device)
        env._nz2_pick = st
    return st


def _active_puck(env):
    """현재 환경에서 쓰는 퍽의 월드 위치·자세 (N,3), (N,4 xyzw)"""
    st = pick_state(env)
    b, c = env.scene["puck_box"].data, env.scene["puck_cyl"].data
    cyl = (st.mode == 2).unsqueeze(-1)
    pos = torch.where(cyl, c.root_pos_w.torch, b.root_pos_w.torch)
    quat = torch.where(cyl, c.root_quat_w.torch, b.root_quat_w.torch)
    return pos, quat


def _grasp_point(env):
    pos, _ = _active_puck(env)
    p = pos.clone()
    p[:, 2] = torch.clamp(p[:, 2], min=GRASP_Z_MIN)
    return p


def _tcp_dist(env, asset_cfg):
    return torch.linalg.norm(C.tcp_pos_w(env, asset_cfg) - _grasp_point(env), dim=1)


def _filtered_force(env, sensor_name: str) -> torch.Tensor:
    """필터 상대와의 접촉력 크기 [N], 최근 기록 중 최대값 (N,)"""
    d = env.scene.sensors[sensor_name].data
    fm = d.normal_force_matrix_w_history.torch                # (N, H, B, F, 3)
    return torch.linalg.norm(fm, dim=-1).sum(dim=-1).amax(dim=(1, 2))


def _grasped(env) -> torch.Tensor:
    return (_filtered_force(env, "grip_l") > GRIP_FORCE_N) & (_filtered_force(env, "grip_r") > GRIP_FORCE_N)


def _lift_height(env) -> torch.Tensor:
    pos, _ = _active_puck(env)
    return pos[:, 2] - pick_state(env).init_pos[:, 2]


def _gripper_openness(env, joint_cfg: SceneEntityCfg, lower: float, upper: float) -> torch.Tensor:
    q = env.scene[joint_cfg.name].data.joint_pos.torch[:, joint_cfg.joint_ids[0]]
    return torch.clamp((q - lower) / (upper - lower), 0.0, 1.0)


# ---------------------------------------------------------------------------
# 리셋 이벤트: 선반·퍽 배치
# ---------------------------------------------------------------------------
def reset_pick_scene(env, env_ids: torch.Tensor):
    st = pick_state(env)
    dev = env.device
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=dev)
    n = len(env_ids)
    U = lambda r: math_utils.sample_uniform(r[0], r[1], (n,), device=dev)
    zeros, origin = torch.zeros(n, device=dev), env.scene.env_origins[env_ids]
    vel0 = torch.zeros(n, 6, device=dev)
    # 선반
    d, w, h = SHELF_SIZE
    near_x = CHASSIS_FRONT_X + U(SHELF_GAP)
    s_yaw = U(SHELF_YAW)
    s_q = math_utils.quat_from_euler_xyz(zeros, zeros, s_yaw)
    s_pos = torch.stack([near_x + d / 2, U(SHELF_LAT), torch.full((n,), h / 2, device=dev)], dim=1)
    shelf = env.scene["shelf"]
    # kinematic 이므로 자세만 쓴다 (속도 쓰기는 PhysX 가 kinematic 에 대해 오류를 낼 수 있음)
    shelf.write_root_pose_to_sim_index(root_pose=torch.cat([s_pos + origin, s_q], dim=-1), env_ids=env_ids)
    # 퍽 종류·위치 (선반 좌표에서 뽑아 월드로)
    probs = torch.tensor(MODE_PROBS, device=dev)
    mode = torch.multinomial(probs, n, replacement=True)
    depth, lat = DEPTH_STAGE[st.stage], LAT_STAGE[st.stage]
    local = torch.stack([U(depth) - d / 2, U(lat), zeros], dim=1)
    p_xy = s_pos + math_utils.quat_apply(s_q, local)
    half = torch.where(mode == 1, torch.full((n,), PUCK_BOX[0] / 2, device=dev),
                       torch.full((n,), PUCK_BOX[2] / 2, device=dev))   # 누움 15 mm / 세움 30 mm
    p_pos = torch.stack([p_xy[:, 0], p_xy[:, 1], h + half + 0.0005], dim=1)
    yaw = U((-math.pi, math.pi))
    roll = torch.where(mode == 1, torch.full((n,), math.pi / 2, device=dev), zeros)
    p_q = math_utils.quat_from_euler_xyz(roll, zeros, yaw)
    ident = torch.tensor([0.0, 0.0, 0.0, 1.0], device=dev).expand(n, 4)
    use_cyl = (mode == 2).unsqueeze(-1)
    park_b = torch.tensor(PARK_BOX, device=dev).expand(n, 3)
    park_c = torch.tensor(PARK_CYL, device=dev).expand(n, 3)
    box_pose = torch.cat([torch.where(use_cyl, park_b, p_pos), torch.where(use_cyl, ident, p_q)], dim=-1)
    cyl_pose = torch.cat([torch.where(use_cyl, p_pos, park_c), torch.where(use_cyl, p_q, ident)], dim=-1)
    box_pose[:, :3] += origin
    cyl_pose[:, :3] += origin
    for name, pose in (("puck_box", box_pose), ("puck_cyl", cyl_pose)):
        env.scene[name].write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
        env.scene[name].write_root_velocity_to_sim_index(root_velocity=vel0, env_ids=env_ids)
    # 상태 기록
    st.mode[env_ids] = mode
    st.init_pos[env_ids] = p_pos + origin
    st.init_axis[env_ids] = math_utils.quat_apply(p_q, torch.tensor([0.0, 0.0, 1.0], device=dev).expand(n, 3))


def pick_stage_curriculum(env, env_ids, switch_steps: int):
    st = pick_state(env)
    st.stage = 0 if env.common_step_counter < switch_steps else 1
    return float(st.stage)


# ---------------------------------------------------------------------------
# 관측 (로봇 기준 좌표)
# ---------------------------------------------------------------------------
def _to_root(env, pos_w, quat_w=None):
    r = env.scene["robot"].data
    return math_utils.subtract_frame_transforms(r.root_pos_w.torch, r.root_quat_w.torch, pos_w, quat_w)


def obs_puck_pos(env):
    return _to_root(env, _active_puck(env)[0])[0]


def obs_puck_quat(env):
    pos, quat = _active_puck(env)
    return _to_root(env, pos, quat)[1]


def obs_puck_mode(env):
    return torch.nn.functional.one_hot(pick_state(env).mode, 3).float()


def obs_shelf_pos(env):
    return _to_root(env, env.scene["shelf"].data.root_pos_w.torch)[0]


def obs_tcp_to_grasp(env, asset_cfg: SceneEntityCfg):
    return _grasp_point(env) - C.tcp_pos_w(env, asset_cfg)


def obs_grasped(env):
    return _grasped(env).float().unsqueeze(-1)


# ---------------------------------------------------------------------------
# 보상
# ---------------------------------------------------------------------------
def rew_reach(env, asset_cfg: SceneEntityCfg, std: float):
    return 1.0 - torch.tanh(_tcp_dist(env, asset_cfg) / std)


def rew_open_far(env, asset_cfg, joint_cfg, lower, upper):
    return _gripper_openness(env, joint_cfg, lower, upper) * (_tcp_dist(env, asset_cfg) > FAR_M).float()


def rew_close_near(env, asset_cfg, joint_cfg, lower, upper):
    return (1.0 - _gripper_openness(env, joint_cfg, lower, upper)) * (_tcp_dist(env, asset_cfg) < NEAR_M).float()


def rew_grasp(env):
    return _grasped(env).float()


def rew_lift(env):
    return _grasped(env).float() * torch.clamp(_lift_height(env) / LIFT_M, 0.0, 1.0)


def rew_success(env):
    """잡은 채 LIFT_M 이상 들고 있음 → TensorBoard Episode_Reward/success 로 성공 시간 확인"""
    return (_grasped(env) & (_lift_height(env) >= LIFT_M)).float()


def rew_filtered_contact(env, sensor_name: str, threshold: float):
    return (_filtered_force(env, sensor_name) > threshold).float()


# ---------------------------------------------------------------------------
# 종료
# ---------------------------------------------------------------------------
def term_filtered_contact(env, sensor_name: str, threshold: float):
    return _filtered_force(env, sensor_name) > threshold


def term_puck_disturbed(env):
    """들지 않은 퍽을 밀거나 넘어뜨림"""
    st = pick_state(env)
    pos, quat = _active_puck(env)
    not_lifted = (pos[:, 2] - st.init_pos[:, 2]) < 0.01
    pushed = torch.linalg.norm(pos[:, :2] - st.init_pos[:, :2], dim=1) > PUSH_LIMIT_M
    axis = math_utils.quat_apply(quat, torch.tensor([0.0, 0.0, 1.0], device=env.device).expand_as(pos))
    tilted = torch.sum(axis * st.init_axis, dim=1) < TILT_LIMIT
    return not_lifted & (pushed | tilted)


def term_puck_dropped(env):
    """퍽이 선반 아래로 떨어짐"""
    return _active_puck(env)[0][:, 2] < SHELF_SIZE[2] - 0.02


# ---------------------------------------------------------------------------
# 장면 부품
# ---------------------------------------------------------------------------
def _puck_spawn(shape: str):
    common = dict(
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            solver_position_iteration_count=16, solver_velocity_iteration_count=1,
            max_depenetration_velocity=1.0, disable_gravity=False),
        mass_props=sim_utils.MassPropertiesCfg(mass=sum(PUCK_MASS) / 2),
        collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.002, rest_offset=0.0),
        physics_material=RigidBodyMaterialCfg(static_friction=0.35, dynamic_friction=0.30, restitution=0.1),
    )
    if shape == "box":
        return sim_utils.CuboidCfg(size=PUCK_BOX, visual_material=sim_utils.PreviewSurfaceCfg(
            diffuse_color=(0.8, 0.1, 0.1)), **common)
    return sim_utils.CylinderCfg(radius=PUCK_CYL_R, height=PUCK_CYL_H, axis="Z", visual_material=sim_utils.PreviewSurfaceCfg(
        diffuse_color=(0.1, 0.7, 0.2)), **common)


@configclass
class NZ2ArmPickCfg(NZ2ArmReachCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.num_envs = NUM_ENVS
        self.episode_length_s = EPISODE_S
        c = load_robot_config(os.environ["ARM_CONFIG"])
        g_lo, g_hi = c["lower"][-1], c["upper"][-1]              # gripper_joint 한계 (닫힘, 열림)
        tcp = C.tcp_cfg()
        grip = SceneEntityCfg("robot", joint_names=["gripper_joint"])

        # ---- Reach 전용 항목 제거 ----
        for name in ("end_effector_position_tracking", "tcp_tanh", "tcp_tanh_fine", "success", "near20"):
            setattr(self.rewards, name, None)
        self.curriculum.target_range = None
        self.observations.policy.pose_command = None
        self.commands.ee_pose.debug_vis = False                  # 명령은 남겨 두되 쓰지 않음

        # ---- 장면: 선반 + 퍽 2개(직육면체·원기둥, 한 번에 하나만 선반 위) ----
        self.scene.shelf = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Shelf",
            spawn=sim_utils.CuboidCfg(
                size=SHELF_SIZE,
                rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
                collision_props=sim_utils.CollisionPropertiesCfg(),
                physics_material=RigidBodyMaterialCfg(static_friction=SHELF_FRICTION, dynamic_friction=SHELF_FRICTION),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.75, 0.6, 0.4))),
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=(CHASSIS_FRONT_X + 0.02 + SHELF_SIZE[0] / 2, 0.0, SHELF_SIZE[2] / 2), rot=(0.0, 0.0, 0.0, 1.0)))
        self.scene.puck_box = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/PuckBox", spawn=_puck_spawn("box"),
            init_state=RigidObjectCfg.InitialStateCfg(pos=PARK_BOX, rot=(0.0, 0.0, 0.0, 1.0)))
        self.scene.puck_cyl = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/PuckCyl", spawn=_puck_spawn("cyl"),
            init_state=RigidObjectCfg.InitialStateCfg(pos=PARK_CYL, rot=(0.0, 0.0, 0.0, 1.0)))

        # ---- 접촉: Reach 의 손가락 '무엇에든' 센서 → '선반·차체' 필터 센서 + '퍽' 필터 센서로 교체 ----
        for key in ("finger_l", "finger_r"):
            setattr(self.scene, f"contact_{key}", None)
            setattr(self.rewards, f"contact_{key}_pen", None)
            setattr(self.terminations, f"contact_{key}_stop", None)
        bad = ["{ENV_REGEX_NS}/Shelf", "{ENV_REGEX_NS}/Robot/" + C.BASE_LINK_PATH]
        pucks = ["{ENV_REGEX_NS}/PuckBox", "{ENV_REGEX_NS}/PuckCyl"]
        for side, key in (("l", "finger_l"), ("r", "finger_r")):
            path = "{ENV_REGEX_NS}/Robot/" + C.LINK_PATHS[key]
            bad_name = f"contact_fbad_{side}"
            setattr(self.scene, bad_name, ContactSensorCfg(prim_path=path, history_length=3, filter_prim_paths_expr=bad))
            setattr(self.scene, f"grip_{side}", ContactSensorCfg(prim_path=path, history_length=3, filter_prim_paths_expr=pucks))
            setattr(self.rewards, f"{bad_name}_pen", RewardTermCfg(
                func=rew_filtered_contact, weight=C.W_CONTACT,
                params={"sensor_name": bad_name, "threshold": C.THRESH_PENALTY_N}))
            setattr(self.terminations, f"{bad_name}_stop", TerminationTermCfg(
                func=term_filtered_contact, params={"sensor_name": bad_name, "threshold": C.THRESH_TERMINATE_N}))
        # (arm_link2~5 는 Reach 와 같이 '무엇에든' 센서 유지 → 선반·퍽·차체에 팔이 닿으면 감점/종료)

        # ---- 리셋·DR ----
        self.events.reset_pick = EventTermCfg(func=reset_pick_scene, mode="reset")
        for name in ("puck_box", "puck_cyl"):
            setattr(self.events, f"{name}_mass", EventTermCfg(
                func=mdp.randomize_rigid_body_mass, mode="startup",
                params={"asset_cfg": SceneEntityCfg(name), "mass_distribution_params": PUCK_MASS,
                        "operation": "abs"}))
            setattr(self.events, f"{name}_material", EventTermCfg(
                func=mdp.randomize_rigid_body_material, mode="startup",
                params={"asset_cfg": SceneEntityCfg(name), "static_friction_range": PUCK_FRICTION,
                        "dynamic_friction_range": PUCK_FRICTION, "restitution_range": PUCK_RESTITUTION,
                        "num_buckets": 32, "make_consistent": True}))
        self.curriculum.pick_stage = CurriculumTermCfg(
            func=pick_stage_curriculum, params={"switch_steps": STAGE_SWITCH_STEPS})
        C.setup_smoothness(self, SMOOTH_CURRICULUM_STEPS)

        # ---- 관측 ----
        obs = self.observations.policy
        obs.puck_pos = ObservationTermCfg(func=obs_puck_pos)
        obs.puck_quat = ObservationTermCfg(func=obs_puck_quat)
        obs.puck_mode = ObservationTermCfg(func=obs_puck_mode)
        obs.shelf_pos = ObservationTermCfg(func=obs_shelf_pos)
        obs.tcp_to_grasp = ObservationTermCfg(func=obs_tcp_to_grasp, params={"asset_cfg": tcp})
        obs.grasped = ObservationTermCfg(func=obs_grasped)

        # ---- 보상 ----
        gp = {"asset_cfg": tcp, "joint_cfg": grip, "lower": g_lo, "upper": g_hi}
        self.rewards.reach = RewardTermCfg(func=rew_reach, weight=W["reach"], params={"asset_cfg": tcp, "std": REACH_STD})
        self.rewards.reach_fine = RewardTermCfg(
            func=rew_reach, weight=W["reach_fine"], params={"asset_cfg": tcp, "std": REACH_FINE_STD})
        self.rewards.open_far = RewardTermCfg(func=rew_open_far, weight=W["open_far"], params=gp)
        self.rewards.close_near = RewardTermCfg(func=rew_close_near, weight=W["close_near"], params=gp)
        self.rewards.grasp = RewardTermCfg(func=rew_grasp, weight=W["grasp"])
        self.rewards.lift = RewardTermCfg(func=rew_lift, weight=W["lift"])
        self.rewards.success = RewardTermCfg(func=rew_success, weight=W["success"])
        self.rewards.puck_fail = RewardTermCfg(
            func=mdp.is_terminated_term, weight=W["puck_fail"], params={"term_keys": "puck_.*"})

        # ---- 종료 ----
        self.terminations.puck_disturbed = TerminationTermCfg(func=term_puck_disturbed)
        self.terminations.puck_dropped = TerminationTermCfg(func=term_puck_dropped)


@configclass
class NZ2ArmPickPPORunnerCfg(NZ2ArmPPORunnerCfg):
    experiment_name = "nz2_arm_pick"
    max_iterations = 1500

    def __post_init__(self):
        super().__post_init__()
        # 관측이 늘었으므로 신경망을 키운다 (rsl_rl 4.x: actor/critic, 구버전: policy)
        for part in ("actor", "critic"):
            m = getattr(self, part, None)
            if m is not None and hasattr(m, "hidden_dims"):
                m.hidden_dims = [256, 128, 64]
        p = getattr(self, "policy", None)
        if p is not None and hasattr(p, "actor_hidden_dims"):
            p.actor_hidden_dims = [256, 128, 64]
            p.critic_hidden_dims = [256, 128, 64]
