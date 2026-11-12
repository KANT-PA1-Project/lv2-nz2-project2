"""NZ2 최대 도달 집기 실험(Sweep) + 원거리 잔차 RL(PickFar) 설정 — Isaac Lab 3.0, Isaac Sim 6.1, PhysX

설치 위치: ~/manipulator-isaac61-lyrical-practice/src/arm_practice/tasks/nz2_maxreach_cfg.py
  같은 폴더: nz2_common.py, nz2_reach_cfg.py, nz2_pick_cfg.py, nz2_arm_kin.py, nz2_arena.py, nz2_scripted_action.py
  등록(__init__.py): NZ2-MaxReach-Sweep, NZ2-PickFar-Arm
로봇: R12 학습 USD (팔 = 차체 중심) + 앞 D435 충돌 끔 → ARM_CONFIG=.../isaac/nz2_arm_noD435.json
장면: mission.pdf v5 맵의 벽·기둥·박스(nz2_arena) 를 로봇 기준으로 배치. 실험 박스는 kinematic 으로 환경마다 간격을 바꿈

1) NZ2-MaxReach-Sweep  (학습 없음, zero_agent 로 실행)
   환경 1개 = 시험 조건 1개: 퍽 칸 × 차체-박스 간격 × 하향각(auto/고정) × 퍽 모양(직육면체/원기둥, 세움)
   스크립트 IK 로 via → pre → grasp → 닫기 → 40 mm 들기 → 1.5 s 유지 후 판정
   성공 = 양손가락 접촉 & 들림 ≥ 25 mm & 닫기 전 퍽 밀림 없음 & 팔·손가락-선반/차체 접촉 < 2 N
   끝나면 logs/nz2_maxreach/<장소>_<시각>/trials.csv, summary.json + 터미널 표
2) NZ2-PickFar-Arm  (rsl_rl 학습)
   먼 줄 퍽만, 차체-박스 간격 0~60 mm 무작위, 인식 오차(계획 퍽 위치 ±σ) → 정책은 IK 궤적에 잔차(±0.08 rad)만 더한다
   (2026-11-04 결정: 기본은 IK, 마지막 정렬 보정만 잔차 RL)

환경 변수 (모두 선택)
  NZ2_SITE        L1 | L3 | S1~S6 | LINE (기본 L1). LINE = 벽 없는 평지, 정면 0.30~0.50 m 10 mm 간격
  NZ2_GAPS        "0.02,0.04,0.06"  차체 앞 ↔ 박스 간격 [m] (Sweep)
  NZ2_PITCHES     "auto" 또는 "auto,90,70,50" (Sweep, 도 단위. auto = 수직부터 내려가며 첫 통과 각)
  NZ2_SHAPES      "box,cyl"
  NZ2_TRIALS      2 (조건당 반복)
  NZ2_FULL_MAP    1 이면 맵 전체 벽 표시 (환경 1~4 개 권장). 기본은 로봇 1 m 안만
  NZ2_SAG_KI      3.0 (처짐 보상 I 이득 [1/s], 0 이면 끔 → 11/9 결과와 비교용)
  NZ2_EXIT_WHEN_DONE 1 이면 Sweep 결과 저장 후 프로세스 종료 (장소 여러 곳 연속 실행용)
  NZ2_GAP_RANGE   "0.0,0.06" (PickFar), NZ2_NOISE_MM 6 (PickFar 인식 오차 σ), NZ2_RESIDUAL_RAD 0.08
"""
import math
import os
from collections import Counter, defaultdict

import torch

import isaaclab.envs.mdp as mdp
from isaaclab.managers import EventTermCfg, ObservationTermCfg, RewardTermCfg, SceneEntityCfg, TerminationTermCfg
from isaaclab.utils import configclass
from isaaclab.utils import math as math_utils

from arm_practice.config import load_robot_config
from arm_practice.tasks import nz2_arena as A
from arm_practice.tasks import nz2_common as C
from arm_practice.tasks import nz2_pick_cfg as PK

# ===== 시험 상수 =====
BOX_TOP = 0.070                         # mission.pdf v5 박스 높이 / 퍽 바닥 Z
PUCK_H = 0.060
GRIP_DEPTH = 0.020                      # 손끝을 퍽 윗면 아래 20 mm 까지 넣음 (실측: 위쪽 15 mm 만 쥐어도 안 미끄러짐 + 여유 5 mm)
TIP_BEYOND_TCP = 0.020                  # URDF: 손끝 = TCP + 19.8 mm (도구축)
GRASP_TCP_Z = BOX_TOP + PUCK_H - GRIP_DEPTH + TIP_BEYOND_TCP   # 0.130 (11/10 v2). 수직일 때 TCP = 퍽 윗면 높이
#   손끝 z 0.110 → 박스 위 40 mm. 처짐 15~23 mm 가 생겨도 17 mm 이상 남음
#   (v0 0.105: 퍽 아래쪽을 쥐고 손끝이 박스 위 15 mm → 처짐에 손가락-박스 접촉 20~47 N 이 11/9 실패 원인)
PITCH_MAX_DEG, PITCH_STEP_DEG = 90.0, 5.0
SPEED_DESCEND, SPEED_APPROACH, SPEED_LIFT = 0.08, 0.04, 0.04   # [m/s] 실기 첫 시험 속도와 비슷하게 낮춤
CLOSE_S, HOLD_S = 1.0, 1.5
LIFT_OK_M = 0.025                       # 목표 40 mm 중 25 mm 이상 들리면 '들림'
CONTACT_FAIL_N = 2.0                    # 실격 판정 접촉력 (nz2_common THRESH_TERMINATE_N 과 같음)
PUSH_M, TILT_DEG = 0.010, 10.0          # 닫기 전 퍽 밀림 판정 (집기 전에 건드림 = 실전에서 위치 오차)
ARM_LINKS = ["arm_link2", "arm_link3", "arm_link4", "arm_link5"]
SWEEP_EPISODE_S = 25.0
FAR_EPISODE_S = 14.0
FAR_NUM_ENVS = 128
W_TRIAL_SUCCESS = 20.0


def _env_list(name, default, conv=float):
    raw = os.environ.get(name, default)
    return [conv(v.strip()) for v in raw.split(",") if v.strip()]


def run_settings():
    keys = ["NZ2_SAG_KI", "NZ2_SITE", "NZ2_GAPS", "NZ2_PITCHES", "NZ2_SHAPES", "NZ2_TRIALS", "NZ2_FULL_MAP", "ARM_CONFIG",
            "NZ2_GAP_RANGE", "NZ2_NOISE_MM", "NZ2_RESIDUAL_RAD"]
    return {k: os.environ.get(k) for k in keys if os.environ.get(k) is not None}


def box_size(site):
    return A.LINE_BOX if A.SITES[site]["box"] is None else A.SMALL_BOX


# ---------------------------------------------------------------------------
# 시험 조건 목록 (Sweep)
# ---------------------------------------------------------------------------
def build_specs(site: str) -> list[dict]:
    pitches = [None if p.lower() == "auto" else float(p) for p in _env_list("NZ2_PITCHES", "auto", str)]
    shapes = _env_list("NZ2_SHAPES", "box,cyl", str)
    for s in shapes:
        if s not in ("box", "cyl"):
            raise ValueError(f"NZ2_SHAPES 는 box,cyl 중: {s}")
    depth = box_size(site)[0]
    specs = []
    if A.SITES[site]["box"] is None:
        for r in A.LINE_R:
            for p in pitches:
                for sh in shapes:
                    specs.append(dict(site=site, cell=-1, gap=0.02, pitch_req=p, shape=sh,
                                      puck_x=r, puck_y=0.0, r=r))
        return specs
    gaps = _env_list("NZ2_GAPS", "0.02,0.04,0.06")
    for ci, (fwd, left) in enumerate(A.cells_robot(site)):
        for g in gaps:
            for p in pitches:
                for sh in shapes:
                    x = A.CHASSIS_FRONT_X + g + depth / 2 + fwd
                    specs.append(dict(site=site, cell=ci, gap=g, pitch_req=p, shape=sh,
                                      puck_x=round(x, 4), puck_y=round(left, 4), r=round(math.hypot(x, left), 4)))
    return specs


# ---------------------------------------------------------------------------
# 환경별 상태
# ---------------------------------------------------------------------------
class MRState:
    def __init__(self, env, mode: str):
        n, dev = env.num_envs, env.device
        self.mode, self.n, self.dev = mode, n, dev
        self.site = A.env_site()
        self.full_map = os.environ.get("NZ2_FULL_MAP") == "1"
        self.park = A.park_points(self.site, self.full_map)
        self.depth = box_size(self.site)[0]
        self.specs = build_specs(self.site) if mode == "sweep" else []
        self.trials = int(os.environ.get("NZ2_TRIALS", "2"))
        self.total = len(self.specs) * self.trials
        self.cursor = torch.arange(n, device=dev)                  # 전체 시험 번호 = env + k·n
        self.finished = torch.zeros(n, dtype=torch.bool, device=dev)
        if mode == "sweep":
            self.finished = self.cursor >= self.total
        self.trial_idx = torch.zeros(n, dtype=torch.long, device=dev)
        self.pitch_req = torch.full((n,), float("nan"), device=dev)
        self.puck = torch.zeros(n, 3, device=dev)                  # 실제 퍽 중심 (로봇 기준)
        self.plan_puck = torch.zeros(n, 3, device=dev)             # 계획에 쓰는 퍽 위치 (PickFar 는 오차 포함)
        self.box_near = torch.zeros(n, device=dev)
        self.box_far = torch.zeros(n, device=dev)
        self.gap = torch.zeros(n, device=dev)
        self.shape = torch.zeros(n, dtype=torch.long, device=dev)  # 0 직육면체 세움, 2 원기둥 세움 (nz2_pick 모드 번호)
        lo, hi = _env_list("NZ2_GAP_RANGE", "0.0,0.06")
        self.gap_range = (lo, hi)
        self.noise = float(os.environ.get("NZ2_NOISE_MM", "6")) / 1000.0
        self.done_count = 0

    def spec_of(self, e: int):
        if self.mode != "sweep" or bool(self.finished[e]):
            return None
        return self.specs[int(self.cursor[e]) % len(self.specs)]

    def mark_done(self, e: int):
        self.done_count += 1
        self.cursor[e] += self.n
        self.trial_idx[e] = int(self.cursor[e]) // max(len(self.specs), 1)
        if int(self.cursor[e]) >= self.total:
            self.finished[e] = True

    def all_done(self) -> bool:
        return bool(self.finished.all())


def mr_state(env) -> MRState:
    st = getattr(env, "_nz2_mr", None)
    if st is None:
        mode = env.cfg.nz2_mode if hasattr(env.cfg, "nz2_mode") else "sweep"
        st = MRState(env, mode)
        env._nz2_mr = st
        if mode == "sweep":
            print(f"[NZ2] Sweep {st.site}: 조건 {len(st.specs)}개 × {st.trials}회 = {st.total} 시험, 환경 {st.n}개")
            if st.n < len(st.specs):
                print(f"[NZ2] 환경 수 < 조건 수 → 환경마다 순서대로 여러 조건을 돌림 (약 {math.ceil(st.total / st.n)}회차)")
    return st


# ---------------------------------------------------------------------------
# 리셋: 박스·퍽 배치
# ---------------------------------------------------------------------------
def reset_maxreach_scene(env, env_ids: torch.Tensor):
    st = mr_state(env)
    dev = env.device
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=dev)
    n = len(env_ids)
    origin = env.scene.env_origins[env_ids]
    d, w, h = box_size(st.site)
    if st.mode == "sweep":
        gap = torch.zeros(n, device=dev)
        pos = torch.zeros(n, 3, device=dev)
        shape = torch.zeros(n, dtype=torch.long, device=dev)
        preq = torch.full((n,), float("nan"), device=dev)
        for j, e in enumerate(env_ids.tolist()):
            sp = st.spec_of(e)
            if sp is None:                                            # 끝난 환경: 마지막 조건 모양 유지
                sp = st.specs[(e % len(st.specs))]
            gap[j] = sp["gap"]
            pos[j, 0], pos[j, 1] = sp["puck_x"], sp["puck_y"]
            shape[j] = 2 if sp["shape"] == "cyl" else 0
            preq[j] = float("nan") if sp["pitch_req"] is None else math.radians(sp["pitch_req"])
        box_x = A.CHASSIS_FRONT_X + gap + d / 2
        plan = pos.clone()
    else:
        # PickFar: 먼 줄 칸만, 간격·좌우 오차·인식 오차 무작위
        U = lambda lo, hi: math_utils.sample_uniform(lo, hi, (n,), device=dev)
        gap = U(*st.gap_range)
        cells = A.cells_robot(st.site) if A.SITES[st.site]["box"] else [(0.06, 0.0)]
        far = max(c[0] for c in cells)
        far_cells = torch.tensor([c for c in cells if abs(c[0] - far) < 1e-6], device=dev)
        pick = far_cells[torch.randint(len(far_cells), (n,), device=dev)]
        box_x = A.CHASSIS_FRONT_X + gap + d / 2
        pos = torch.zeros(n, 3, device=dev)
        pos[:, 0] = box_x + pick[:, 0] + U(-0.005, 0.005)
        pos[:, 1] = pick[:, 1] + U(-0.005, 0.005)
        shape = torch.where(torch.rand(n, device=dev) < 0.5, 0, 2)
        preq = torch.full((n,), float("nan"), device=dev)
        plan = pos.clone()
        plan[:, :2] += torch.clamp(torch.randn(n, 2, device=dev) * st.noise, -2 * st.noise, 2 * st.noise)
    pos[:, 2] = BOX_TOP + PUCK_H / 2 + 0.0005
    plan[:, 2] = pos[:, 2]
    # 박스 (kinematic: 자세만 씀)
    ident = torch.tensor([0.0, 0.0, 0.0, 1.0], device=dev).expand(n, 4)
    s_pos = torch.stack([box_x, torch.zeros(n, device=dev), torch.full((n,), h / 2, device=dev)], dim=1)
    env.scene["shelf"].write_root_pose_to_sim_index(root_pose=torch.cat([s_pos + origin, ident], dim=-1), env_ids=env_ids)
    # 퍽: 직육면체는 면이 손가락과 평행하도록 yaw = 팔 방향
    yaw = torch.atan2(pos[:, 1], pos[:, 0])
    zeros = torch.zeros(n, device=dev)
    p_q = math_utils.quat_from_euler_xyz(zeros, zeros, yaw)
    park_b = torch.tensor([*st.park[0], 0.031], device=dev).expand(n, 3)
    park_c = torch.tensor([*st.park[1], 0.031], device=dev).expand(n, 3)
    use_cyl = (shape == 2).unsqueeze(-1)
    box_pose = torch.cat([torch.where(use_cyl, park_b, pos), torch.where(use_cyl, ident, p_q)], dim=-1)
    cyl_pose = torch.cat([torch.where(use_cyl, pos, park_c), torch.where(use_cyl, p_q, ident)], dim=-1)
    box_pose[:, :3] += origin
    cyl_pose[:, :3] += origin
    vel0 = torch.zeros(n, 6, device=dev)
    for name, pose in (("puck_box", box_pose), ("puck_cyl", cyl_pose)):
        env.scene[name].write_root_pose_to_sim_index(root_pose=pose, env_ids=env_ids)
        env.scene[name].write_root_velocity_to_sim_index(root_velocity=vel0, env_ids=env_ids)
    # nz2_pick 상태(보상·관측·종료가 씀) 갱신
    ps = PK.pick_state(env)
    ps.mode[env_ids] = shape
    ps.init_pos[env_ids] = pos + origin
    ps.init_axis[env_ids] = torch.tensor([0.0, 0.0, 1.0], device=dev)
    # 실험 상태
    st.puck[env_ids], st.plan_puck[env_ids] = pos, plan
    st.gap[env_ids], st.shape[env_ids], st.pitch_req[env_ids] = gap, shape, preq
    st.box_near[env_ids] = A.CHASSIS_FRONT_X + gap
    st.box_far[env_ids] = A.CHASSIS_FRONT_X + gap + d


# ---------------------------------------------------------------------------
# 측정 함수 (행동 클래스·보상·종료가 공용)
# ---------------------------------------------------------------------------
def _tcp_cfg(env):
    cfg = getattr(env, "_nz2_tcp_cfg", None)
    if cfg is None:
        cfg = C.tcp_cfg()
        cfg.resolve(env.scene)
        env._nz2_tcp_cfg = cfg
    return cfg


def tcp_in_root(env):
    r = env.scene["robot"].data
    p, _ = math_utils.subtract_frame_transforms(r.root_pos_w.torch, r.root_quat_w.torch, C.tcp_pos_w(env, _tcp_cfg(env)))
    return p


def arm_contact_force(env):
    """arm_link2~5 '무엇에든' 접촉력 최대 [N] (N,), 가장 큰 링크 번호 (N,)"""
    fs = []
    for key in ARM_LINKS:
        d = env.scene.sensors[f"contact_{key}"].data.net_normal_forces_w_history.torch   # (N, H, B, 3)
        fs.append(torch.linalg.norm(d, dim=-1).amax(dim=(1, 2)))
    f = torch.stack(fs, dim=1)
    return f.max(dim=1)


def finger_bad_force(env):
    return torch.maximum(PK._filtered_force(env, "contact_fbad_l"), PK._filtered_force(env, "contact_fbad_r"))


def grasped_now(env):
    return PK._grasped(env)


def puck_lift(env):
    return PK._lift_height(env)


def puck_disturbed(env):
    ps = PK.pick_state(env)
    pos, quat = PK._active_puck(env)
    pushed = torch.linalg.norm(pos[:, :2] - ps.init_pos[:, :2], dim=1) > PUSH_M
    axis = math_utils.quat_apply(quat, torch.tensor([0.0, 0.0, 1.0], device=env.device).expand_as(pos))
    tilted = torch.sum(axis * ps.init_axis, dim=1) < math.cos(math.radians(TILT_DEG))
    return pushed | tilted


def _arm_term(env):
    return env.action_manager.get_term(C.ACTION_TERM)


def term_trial_done(env):
    return _arm_term(env).done_flag.clone()


def obs_phase(env):
    from arm_practice.tasks.nz2_scripted_action import NUM_PHASES   # 실행 시점 (행동 항이 이미 만들어진 뒤)
    return torch.nn.functional.one_hot(_arm_term(env).phase, NUM_PHASES).float()


def rew_trial_success(env):
    t = _arm_term(env)
    return (t.done_flag & t.success_flag).float()


# ---------------------------------------------------------------------------
# 결과 요약
# ---------------------------------------------------------------------------
def summarize(records: list[dict]) -> list[dict]:
    g = defaultdict(list)
    for r in records:
        g[(r["cell"], r["gap"], r["pitch_req"], r["r"])].append(r)
    out = []
    for (cell, gap, preq, rr), rs in sorted(g.items(), key=lambda kv: (kv[0][3], str(kv[0][2]))):
        ok = sum(r["success"] for r in rs)
        pit = sorted({r["pitch_deg"] for r in rs if r["pitch_deg"] is not None})
        out.append(dict(cell=cell, gap_mm=round(gap * 1000), pitch_req=preq or "auto", r_mm=round(rr * 1000),
                        n=len(rs), success=ok, rate=round(ok / len(rs), 2), pitch_used=pit,
                        by_shape={s: sum(r["success"] for r in rs if r["shape"] == s) for s in ("box", "cyl")},
                        reasons=dict(Counter(r["reason"] for r in rs)),
                        first_hit=dict(Counter(r.get("first_hit", "-") for r in rs)),
                        tcp_dz_mm=[r.get("tcp_dz_mm") for r in rs]))
    return out


def summary_table(site: str, summary: list[dict]) -> str:
    lines = [f"=== NZ2 최대 도달 집기 결과: {site} (r = 팔 중심 → 퍽 중심 수평거리) ===",
             f"{'r[mm]':>6} {'칸':>3} {'간격':>5} {'각요청':>6} {'사용각':>10} {'성공':>7}  주된 실패 원인"]
    for s in summary:
        fail = {k: v for k, v in s["reasons"].items() if k != "성공"}
        top = max(fail, key=fail.get) if fail else "-"
        hits = {k: v for k, v in s.get("first_hit", {}).items() if k != "-"}
        if hits:
            top += f" (처음 닿은 단계: {max(hits, key=hits.get)})"
        lines.append(f"{s['r_mm']:>6} {s['cell']:>3} {s['gap_mm']:>5} {str(s['pitch_req']):>6} "
                     f"{str(s['pitch_used']):>10} {s['success']:>3}/{s['n']:<3}  {top}")
    full = [s["r_mm"] for s in summary if s["rate"] >= 1.0]
    lines.append(f"→ 100 % 성공한 최대 r = {max(full) if full else '없음'} mm")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# 설정
# ---------------------------------------------------------------------------
@configclass
class NZ2ScriptedPickActionCfg(mdp.JointPositionActionCfg):
    class_type: type | str = "arm_practice.tasks.nz2_scripted_action:NZ2ScriptedPickAction"
    mode: str = "sweep"                       # "sweep" | "residual"
    residual_rad: float = 0.08                # 잔차 최대 [rad] (팔 5축)
    residual_rate: float = 0.3                # 잔차 변화 속도 한계 [rad/s] (30 Hz 에서 스텝당 0.01 rad)
    sag_ki: float = 3.0                       # 처짐 보상 적분 이득 [1/s] (0 = 끔). 시정수 약 0.33 s
    sag_max: float = 0.15                     # 처짐 보상 최대 [rad] (8.6°) — 막혔을 때 무한히 밀지 않게
    joint_speed: list = [1.0, 1.0, 1.0, 1.0, 1.0, 3.0]   # [rad/s] nz2_arm.json velocity 순서


def _setup(cfg, mode: str):
    site = A.env_site()
    full = os.environ.get("NZ2_FULL_MAP") == "1"
    cfg.nz2_mode = mode
    c = load_robot_config(os.environ["ARM_CONFIG"])
    if "noD435" not in os.path.basename(c["usd_path"]) and os.environ.get("NZ2_ALLOW_D435") != "1":
        print(f"[NZ2][경고] USD 이름에 noD435 가 없음: {c['usd_path']} (실행 시 충돌상자 검사로 다시 확인)")
    # 환경 수
    if mode == "sweep":
        cfg.scene.num_envs = max(1, len(build_specs(site)))
        cfg.episode_length_s = SWEEP_EPISODE_S
    else:
        cfg.scene.num_envs = FAR_NUM_ENVS
        cfg.episode_length_s = FAR_EPISODE_S
    cfg.scene.env_spacing = 14.0 if full else 3.5
    # 실험 박스 (pick 의 410 폭 선반 → 실제 205 박스 / LINE 은 410 깊이)
    size = box_size(site)
    cfg.scene.shelf.spawn.size = size
    cfg.scene.shelf.init_state.pos = (A.CHASSIS_FRONT_X + A.NOMINAL_GAP + size[0] / 2, 0.0, size[2] / 2)
    park = A.park_points(site, full)
    cfg.scene.puck_box.init_state.pos = (*park[0], 0.031)
    cfg.scene.puck_cyl.init_state.pos = (*park[1], 0.031)
    # 경기장
    names = A.add_arena_to_scene(cfg.scene, site, None if full else float(os.environ.get("NZ2_ARENA_RADIUS", "1.0")))
    print(f"[NZ2] 장소 {site}: 경기장 요소 {len(names)}개 ({'전체 맵' if full else '로봇 주변'})")
    # 행동: 스크립트 IK (+ 잔차)
    names_j = [n for n in c["joint_names"]]
    margin = math.radians(2.0)
    cfg.actions.arm_action = NZ2ScriptedPickActionCfg(
        asset_name="robot", joint_names=names_j, preserve_order=True, use_default_offset=False, scale=1.0,
        clip={n: (lo + margin, hi - margin) for n, lo, hi in zip(names_j, c["lower"], c["upper"], strict=True)},
        mode=mode, residual_rad=float(os.environ.get("NZ2_RESIDUAL_RAD", "0.08")),
        joint_speed=list(c["velocity"]),
        sag_ki=float(os.environ.get("NZ2_SAG_KI", "3.0")))
    # 리셋·커리큘럼
    cfg.events.reset_pick = EventTermCfg(func=reset_maxreach_scene, mode="reset")
    cfg.curriculum.pick_stage = None
    # 종료
    cfg.terminations.trial_done = TerminationTermCfg(func=term_trial_done)
    if mode == "sweep":
        # 실험은 끝까지 진행하고 접촉·밀림은 '기록'만 (중간 종료하면 원인 구분이 어려움)
        for k in list(cfg.terminations.__dict__):
            if k.startswith("contact_") or k in ("puck_disturbed", "puck_dropped"):
                setattr(cfg.terminations, k, None)
        for k in ("contact_termination", "puck_fail"):
            setattr(cfg.rewards, k, None)
        cfg.observations.policy.enable_corruption = False
    else:
        cfg.rewards.trial_success = RewardTermCfg(func=rew_trial_success, weight=W_TRIAL_SUCCESS)
    # 관측: 상태기계 단계
    cfg.observations.policy.phase = ObservationTermCfg(func=obs_phase)


@configclass
class NZ2MaxReachSweepCfg(PK.NZ2ArmPickCfg):
    nz2_mode: str = "sweep"

    def __post_init__(self):
        super().__post_init__()
        _setup(self, "sweep")


@configclass
class NZ2PickFarCfg(PK.NZ2ArmPickCfg):
    nz2_mode: str = "residual"

    def __post_init__(self):
        super().__post_init__()
        _setup(self, "residual")
        if os.environ.get("NZ2_PLAY") == "1":
            self.observations.policy.enable_corruption = False


@configclass
class NZ2PickFarPPORunnerCfg(PK.NZ2ArmPickPPORunnerCfg):
    experiment_name = "nz2_arm_pickfar"
    max_iterations = 600

    def __post_init__(self):
        super().__post_init__()
        # 잔차 정책: 처음엔 거의 0 을 내도록 탐색 노이즈를 작게 (rsl_rl 버전별 이름 차이 대응)
        for part in ("actor", "policy"):
            m = getattr(self, part, None)
            if m is None:
                continue
            if hasattr(m, "init_noise_std"):
                m.init_noise_std = 0.3
            dist = getattr(m, "distribution_cfg", None)
            if dist is not None and hasattr(dist, "init_std"):
                dist.init_std = 0.3
