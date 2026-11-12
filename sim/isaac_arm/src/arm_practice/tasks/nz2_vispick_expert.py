"""NZ2 영상 집기 '전문가'(스크립트 IK) 행동 + 시연 기록기 — 모방학습(BC)용 (실행 시점에만 불러옴)

설치 위치: ~/manipulator-isaac61-lyrical-practice/src/arm_practice/tasks/nz2_vispick_expert.py
같은 폴더 필요: nz2_arm_kin, nz2_smooth_action, nz2_common, nz2_vispick_cfg, nz2_vispick_geom
주의: Isaac 실행용 모듈을 import 한다 → 설정에서는 문자열 class_type 으로만 가리킨다 (pxr 충돌 방지, 11/3 교훈).

동작 (환경마다 독립, 30 Hz, 바닥 고정 팔 좌표 = 팔 베이스 바로 아래 바닥이 원점)
  0 WAIT 0.3 s → 1 TO_VIA 관절 보간(현재 자세 → via) → 2 DESCEND 직선 via→pre → 3 APPROACH 도구축 직선 pre→grasp
  → 4 CLOSE 1.0 s → 5 LIFT 직선 +40 mm → 6 HOLD 1.5 s → 7 DONE (a = 0)       8 SKIP = 도달 불가(a = 0, 기록 안 함)
  직선 구간은 매 스텝 해석적 IK (nz2_arm_kin, NZ2_KIN_J1_Z = NZ2_ARM_Z) → 손끝이 직선으로 움직임
행동 형식: 정책과 같은 6차원 [−1, 1] (NZ2SmoothJointPositionAction delta 모드 = 관절 속도 명령)
  a = clip((원하는 목표각 − 현재 목표각) / (v_max · dt), −1, 1)  → 정책이 흉내 낼 수 있는 형식
기록(RECORD): 정책이 본 관측(증강 포함 영상 2대 + 관절·명령색)과 이 행동을 저장. 성공한 에피소드만 파일로 (T 스텝/sub)
  성공 = 환경의 success 종료 (파지 + 40 mm 들림 + 다른 퍽 무사) 그리고 그리퍼 열림 폭 검사 통과 (헛잡기 시연 제외)
안전장치
  - 시작 시 FK 자기검사: 해석 FK 와 시뮬 TCP 차이 > 20 mm 이면 중단 (부호·치수·팔 장착 위치 불일치)
  - 관절 속도 ≤ max_velocity, 목표는 관절 한계 안쪽 (부모 행동 클래스 처리), IK 실패 시 그 자리 정지 + SKIP
  - 처짐 보상(관절 I) 한계 ±0.15 rad: 중력으로 손끝이 15~23 mm 처지는 것을 보정 (11/9 실험)
  - 추종 오차 > 0.12 rad 이면 경로 진행 멈춤 (경로가 관절을 앞질러 달아나지 않게)
환경 변수: NZ2_EXPERT_DIR(저장 폴더, 기본 logs/nz2_demo/<시각>), NZ2_EXPERT_EPISODES(목표 성공 에피소드 수, 기본 300),
          NZ2_EXPERT_SUB(저장 간격 스텝, 기본 3), NZ2_EXPERT_RECORD=0(기록 끔, 전문가 성능만 측정)
"""
from __future__ import annotations

import json
import math
import os
import time
from collections.abc import Sequence

import torch

from isaaclab.utils import math as math_utils

from arm_practice.tasks import nz2_arm_kin as K
from arm_practice.tasks import nz2_common as C
from arm_practice.tasks import nz2_vispick_cfg as VC
from arm_practice.tasks import nz2_vispick_geom as G
from arm_practice.tasks.nz2_smooth_action import NZ2SmoothJointPositionAction

P_WAIT, P_TO_VIA, P_DESCEND, P_APPROACH, P_CLOSE, P_LIFT, P_HOLD, P_DONE, P_SKIP = range(9)
PHASE_NAMES = ["WAIT", "TO_VIA", "DESCEND", "APPROACH", "CLOSE", "LIFT", "HOLD", "DONE", "SKIP"]

# ===== 전문가 상수 (한곳에서 바꿈) =====
WAIT_S = 0.3
V_DESCEND = 0.10          # via → pre 직선 속도 [m/s]
V_APPROACH = 0.05         # pre → grasp (퍽에 다가가는 구간, 느리게)
V_LIFT = 0.05
CLOSE_S = 1.0
HOLD_S = 1.5
BACK_M, UP_M, LIFT_M = 0.05, 0.06, 0.04       # 경유점 (nz2_arm_kin.plan_grasp 와 같은 값)
SAG_KI, SAG_MAX = 3.0, 0.15                   # 처짐 보상 [1/s], 최대 [rad]
TRACK_GATE_RAD = 0.12
FK_CHECK_MM = 20.0
PITCH_MIN_DEG, PITCH_STEP_DEG = 10.0, 5.0
BOX_TOP = G.BOX[2]


class NZ2VisPickExpertAction(NZ2SmoothJointPositionAction):
    cfg: "object"

    def __init__(self, cfg: "object", env):
        super().__init__(cfg, env)
        if self.action_dim != 6:
            raise ValueError(f"관절 6개(arm_joint1~5, gripper_joint) 필요: {self._joint_names}")
        n, dev = self.num_envs, self.device
        self._env_ref = env
        self._dt = env.step_dt
        self._lo = self._clip[0, :, 0].clone()
        self._hi = self._clip[0, :, 1].clone()
        self._vmax = self._max_step[0] / self._dt                                # (6,) rad/s
        self.phase = torch.full((n,), P_SKIP, dtype=torch.long, device=dev)
        self.t_ph = torch.zeros(n, device=dev)                                   # 단계 경과 시간
        self.s = torch.zeros(n, device=dev)                                      # 직선 진행도 0~1
        self.pitch = torch.zeros(n, device=dev)
        self.wps = torch.zeros(n, 4, 3, device=dev)
        self.q_via = torch.zeros(n, 5, device=dev)
        self.q_start = torch.zeros(n, 5, device=dev)
        self.via_T = torch.ones(n, device=dev)
        self.g_cmd = torch.full((n,), float(self._hi[5]), device=dev)
        self.q_cmd = torch.zeros(n, 6, device=dev)
        self.q_int = torch.zeros(n, 5, device=dev)
        self.need_plan = torch.ones(n, dtype=torch.bool, device=dev)
        self.last_a = torch.zeros(n, self.action_dim, device=dev)
        self._checked = False
        self._tcfg = None
        # 통계 + 기록
        self.n_done = 0
        self.n_ok = 0
        self.n_skip = 0
        self.n_ikfail = 0
        self.active = torch.zeros(n, dtype=torch.bool, device=dev)      # 계획이 세워져 기록 중인 에피소드
        self.n_saved = 0
        self.record = os.environ.get("NZ2_EXPERT_RECORD", "1") != "0"
        self.sub = max(1, int(os.environ.get("NZ2_EXPERT_SUB", "3")))
        self.target_eps = int(os.environ.get("NZ2_EXPERT_EPISODES", "300"))
        self.out_dir = os.environ.get("NZ2_EXPERT_DIR") or os.path.join("logs", "nz2_demo", time.strftime("%Y%m%d_%H%M%S"))
        self.buf: list[dict | None] = [None] * n
        self.grip_ok = torch.zeros(n, dtype=torch.bool, device=dev)
        self.log_rows: list[dict] = []
        if self.record:
            os.makedirs(self.out_dir, exist_ok=True)
            print(f"[NZ2-EXPERT] 시연 저장 → {os.path.abspath(self.out_dir)} (목표 성공 {self.target_eps} 에피소드, {self.sub} 스텝마다 1개)")

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
        err = (K.fk_tcp(q) - tcp).norm(dim=1)
        e = float(err.max()) * 1000.0
        print(f"[NZ2-EXPERT] FK 자기검사: TCP 오차 최대 {e:.1f} mm (J1_Z={K.J1_Z}, ARM_Z={G.ARM_Z})")
        if abs(K.J1_Z - G.ARM_Z) > 1e-6:
            raise RuntimeError(f"NZ2_KIN_J1_Z({K.J1_Z}) 와 NZ2_ARM_Z({G.ARM_Z}) 가 다름 → 실행기 expert 모드로 실행 (둘 다 같은 값)")
        if e > FK_CHECK_MM:
            raise RuntimeError(f"해석 FK 와 시뮬 TCP 가 {e:.0f} mm 다름 → nz2_arm_kin 치수·부호 또는 팔 장착 위치(arm_j1_xyz) 확인")
        self._checked = True

    # ------------------------------------------------------------------
    # 계획 (새 에피소드 첫 스텝, 퍽이 놓인 뒤)
    # ------------------------------------------------------------------
    def _plan(self, ids: torch.Tensor):
        env = self._env_ref
        dev = self.device
        gp = VC._grasp_point(env) - env.scene.env_origins                       # (N,3) 팔 좌표 목표 TCP
        sh = env.scene["shelf"].data.root_pos_w.torch - env.scene.env_origins
        near, far = sh[:, 0] - G.BOX[0] / 2, sh[:, 0] + G.BOX[0] / 2
        puck, gz = gp[ids], gp[ids, 2]
        pitch, wps, ok, _clear = K.choose_plan(puck, gz, near[ids], far[ids], BOX_TOP, pitch_min_deg=PITCH_MIN_DEG,
                                               step_deg=PITCH_STEP_DEG, keepout=())
        # via 높이가 IK 에 안 닿으면 via 만 낮춤 (손끝 15 mm 아래까지)
        self.pitch[ids] = torch.nan_to_num(pitch, nan=0.0)
        self.wps[ids] = wps
        q_via, okv = K.ik(wps[:, 0], self.pitch[ids])
        ok = ok & okv
        self.q_via[ids] = q_via
        q0 = self._q_meas()[ids, :5]
        self.q_start[ids] = q0
        dq = (q_via - q0).abs().amax(dim=1)
        self.via_T[ids] = torch.clamp(dq / (0.6 * float(self._vmax[:5].min())), min=0.8)
        self.q_cmd[ids] = self._q_meas()[ids]
        self.q_int[ids] = 0.0
        self.s[ids] = 0.0
        self.t_ph[ids] = 0.0
        self.g_cmd[ids] = float(self._hi[5])
        self.grip_ok[ids] = False
        ph = torch.where(ok, torch.full_like(ids, P_WAIT), torch.full_like(ids, P_SKIP))
        self.phase[ids] = ph
        self.active[ids] = True
        self.n_skip += int((~ok).sum())

    # ------------------------------------------------------------------
    # 상태기계 (한 스텝)
    # ------------------------------------------------------------------
    def _seg(self, a: torch.Tensor, b: torch.Tensor, speed: float, track_ok: torch.Tensor):
        """직선 a→b 위 진행도 s 를 올리고 그 점의 IK 를 q_cmd 에 쓴다. 반환: (끝났나, IK 성공)"""
        length = torch.clamp((b - a).norm(dim=1), min=1e-4)
        self.s = torch.where(track_ok, torch.clamp(self.s + speed * self._dt / length, max=1.0), self.s)
        p = a + (b - a) * self.s.unsqueeze(1)
        q, ok = K.ik(p, self.pitch)
        return self.s >= 1.0 - 1e-6, q, ok

    def _step_machine(self):
        dt = self._dt
        q_meas = self._q_meas()
        n = self.num_envs
        ph = self.phase
        self.t_ph += dt
        track_err = (self.q_cmd[:, :5] - q_meas[:, :5]).abs().amax(dim=1)
        track_ok = track_err < TRACK_GATE_RAD
        g = self.wps[:, 2]                                                       # grasp
        pre, via, lift = self.wps[:, 1], self.wps[:, 0], self.wps[:, 3]
        idx = torch.arange(n, device=self.device)

        def to(new: int, mask: torch.Tensor):
            self.phase[mask] = new
            self.t_ph[mask] = 0.0
            self.s[mask] = 0.0

        # WAIT → TO_VIA
        m = (ph == P_WAIT) & (self.t_ph >= WAIT_S)
        self.q_start[m] = q_meas[m, :5]
        to(P_TO_VIA, m)
        # TO_VIA: 관절 보간 (smoothstep)
        m = ph == P_TO_VIA
        if m.any():
            u = torch.clamp(self.t_ph / self.via_T, 0.0, 1.0)
            u = u * u * (3.0 - 2.0 * u)
            qv = self.q_start + (self.q_via - self.q_start) * u.unsqueeze(1)
            self.q_cmd[m, :5] = qv[m]
            done = m & (self.t_ph >= self.via_T) & (track_err < 0.05)
            to(P_DESCEND, done)
        # DESCEND / APPROACH / LIFT: 직선 + IK
        for phase, a, b, spd, nxt in ((P_DESCEND, via, pre, V_DESCEND, P_APPROACH),
                                      (P_APPROACH, pre, g, V_APPROACH, P_CLOSE),
                                      (P_LIFT, g, lift, V_LIFT, P_HOLD)):
            m = self.phase == phase
            if not m.any():
                continue
            fin, q, ok = self._seg(a, b, spd, track_ok)
            good = m & ok
            self.q_cmd[good, :5] = q[good]
            bad = m & ~ok                                                        # IK 실패: 그 자리 정지 + SKIP
            self.phase[bad] = P_SKIP
            self.n_ikfail += int(bad.sum())
            fin_m = m & fin & ok & (track_err < 0.03)
            to(nxt, fin_m)
        # CLOSE: 그리퍼 닫기 (선형), 끝나면 LIFT
        m = self.phase == P_CLOSE
        if m.any():
            span = float(self._hi[5] - self._lo[5])
            self.g_cmd = torch.where(m, torch.clamp(self.g_cmd - span * dt / CLOSE_S, min=float(self._lo[5])), self.g_cmd)
            to(P_LIFT, m & (self.t_ph >= CLOSE_S))
        # HOLD → DONE
        to(P_DONE, (self.phase == P_HOLD) & (self.t_ph >= HOLD_S))
        self.q_cmd[:, 5] = self.g_cmd

    def _grip_ok_update(self):
        """들어올림·유지 중 그리퍼가 퍽 폭에서 멈춰 있는지 (완전히 닫히면 헛잡기, 허용 범위 밖이면 놓침)"""
        g = self._q_meas()[:, 5]
        closed_fully = g <= float(self._lo[5]) + 0.01
        carrying = (self.phase == P_LIFT) | (self.phase == P_HOLD) | (self.phase == P_DONE)
        # 한 번이라도 완전히 닫히면 실패로 기록 (grip_ok 는 '들고 있는 동안 계속 참'이어야 함)
        self.grip_ok = torch.where(self.phase == P_CLOSE, torch.ones_like(self.grip_ok), self.grip_ok & ~(carrying & closed_fully))

    # ------------------------------------------------------------------
    # 행동 처리
    # ------------------------------------------------------------------
    def process_actions(self, actions: torch.Tensor):
        env = self._env_ref
        if not self._checked:
            self._fk_self_check()
        new_ep = env.episode_length_buf == 0
        if bool(new_ep.any()):
            self._flush(new_ep.nonzero().squeeze(-1))
            self._plan(new_ep.nonzero().squeeze(-1))
        self._step_machine()
        self._grip_ok_update()
        # 처짐 보상: 실제 각이 목표보다 처지는 만큼 천천히 적분해 목표에 더함
        q_meas = self._q_meas()
        moving = ((self.phase >= P_TO_VIA) & (self.phase <= P_HOLD)).unsqueeze(1)
        err = self.q_cmd[:, :5] - q_meas[:, :5]
        self.q_int = torch.clamp(self.q_int + SAG_KI * err * self._dt * moving, -SAG_MAX, SAG_MAX)
        goal = self.q_cmd.clone()
        goal[:, :5] += self.q_int
        goal = torch.maximum(torch.minimum(goal, self._hi), self._lo)
        a = torch.clamp((goal - self._prev_target) / self._max_step, -1.0, 1.0)
        stop = (self.phase == P_DONE) | (self.phase == P_SKIP)
        a = torch.where(stop.unsqueeze(1), torch.zeros_like(a), a)
        self.last_a = a
        self._append(a)
        super().process_actions(a)

    # ------------------------------------------------------------------
    # 기록
    # ------------------------------------------------------------------
    def _append(self, a: torch.Tensor):
        if not self.record:
            return
        env = self._env_ref
        obs = getattr(env, "obs_buf", None)
        if obs is None or "wrist_cam" not in obs:
            return
        step = env.episode_length_buf
        take = ((step % self.sub) == 0) & (self.phase != P_SKIP)
        if not bool(take.any()):
            return
        ids = take.nonzero().squeeze(-1)
        pol = obs["policy"][ids].detach().float().cpu()
        wr = obs["wrist_cam"][ids].detach().half().cpu()
        tp = obs["top_cam"][ids].detach().half().cpu()
        act = a[ids].detach().float().cpu()
        phs = self.phase[ids].cpu()
        for j, e in enumerate(ids.tolist()):
            b = self.buf[e]
            if b is None:
                b = {"policy": [], "wrist": [], "top": [], "action": [], "phase": []}
                self.buf[e] = b
            b["policy"].append(pol[j]); b["wrist"].append(wr[j]); b["top"].append(tp[j])
            b["action"].append(act[j]); b["phase"].append(phs[j])

    def _flush(self, ids: torch.Tensor):
        """끝난 에피소드 정리: 성공(환경 success 종료 + 열림 폭 검사)만 파일로. ids = 방금 리셋된 환경"""
        env = self._env_ref
        try:
            succ = env.termination_manager.get_term("success")
        except Exception:
            succ = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        st = VC.vp(env)
        for e in ids.tolist():
            b = self.buf[e]
            self.buf[e] = None
            if not bool(self.active[e]):                       # 첫 리셋 (끝난 에피소드 없음)
                continue
            self.active[e] = False
            ok = bool(succ[e])
            self.n_done += 1
            self.n_ok += int(ok)
            if self.record and ok and b is not None and bool(self.grip_ok[e]) and len(b["action"]) > 3:
                self._save(b, e, st)
        if self.n_done and self.n_done % 64 < len(ids):
            print(f"[NZ2-EXPERT] 에피소드 {self.n_done}: 성공 {self.n_ok} ({100 * self.n_ok / self.n_done:.0f} %) / "
                  f"도달불가 {self.n_skip} / 중간 IK 실패 {self.n_ikfail} / 저장 {self.n_saved}")
        if self.n_done >= int(os.environ.get("NZ2_EXPERT_MAX_EPS", "100000")) or (self.record and self.n_saved >= self.target_eps):
            self._finish()

    def _save(self, b: dict, e: int, st):
        ep = {k: torch.stack(v) for k, v in b.items()}
        ep["color"] = int(st.color[e]); ep["lying"] = bool(st.lying[e, int(st.color[e])]); ep["stage"] = int(st.stage)
        ep["pitch_deg"] = float(math.degrees(float(self.pitch[e]))); ep["sub"] = self.sub
        torch.save(ep, os.path.join(self.out_dir, f"ep_{self.n_saved:06d}.pt"))
        self.n_saved += 1

    def _finish(self):
        meta = {"episodes_saved": self.n_saved, "episodes_total": self.n_done, "success": self.n_ok, "sub": self.sub,
                "obs_shapes": {"wrist": [4, G.OBS_H, G.OBS_W], "top": [4, G.OBS_H, G.OBS_W]}, "kin_j1_z": K.J1_Z}
        if self.record:
            with open(os.path.join(self.out_dir, "meta.json"), "w", encoding="utf-8") as f:
                json.dump(meta, f, ensure_ascii=False, indent=2)
        rate = 100 * self.n_ok / max(1, self.n_done)
        print(f"[NZ2-EXPERT] 완료: 전체 {self.n_done} / 성공 {self.n_ok} ({rate:.0f} %) / 도달불가 {self.n_skip} / 중간 IK 실패 {self.n_ikfail} / 저장 {self.n_saved}"
              + (f" → {os.path.abspath(self.out_dir)}" if self.record else " (기록 안 함)"))
        os._exit(0)

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        super().reset(env_ids)
        ids = torch.arange(self.num_envs, device=self.device) if env_ids is None else torch.as_tensor(env_ids, device=self.device, dtype=torch.long)
        self.need_plan[ids] = True
