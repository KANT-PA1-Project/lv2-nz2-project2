"""NZ2 스크립트 집기 행동 (해석적 IK 상태기계 + 선택적 잔차 RL) — 실행 시점에만 불러옴

주의: Isaac Lab 실행용 모듈(JointPositionAction → pxr)을 import 한다.
  설정 파일에서는 class_type 문자열 "arm_practice.tasks.nz2_scripted_action:NZ2ScriptedPickAction" 로만 가리킨다.
  (설정 단계에서 직접 import 하면 pxr 충돌 — 2026-11-03 교훈 6)

동작 (환경마다 독립, 30 Hz)
  0 WAIT 0.3 s → 1 TO_VIA 관절공간 동기 보간(접은 자세 → via) → 2 DESCEND 직선(via → pre)
  → 3 APPROACH 도구축 직선(pre → grasp) → 4 CLOSE 1.0 s → 5 LIFT 직선 40 mm → 6 HOLD 1.5 s → 7 DONE(기록)
  8 SKIP = 계획 실패(도달 불가) → 바로 DONE 으로 기록 / 9 IDLE = 할당된 시험을 모두 끝냄
  직선 구간은 매 스텝 해석적 IK (nz2_arm_kin) → 손끝이 직선으로 움직여 이웃 퍽·선반을 쓸지 않는다.
모드
  sweep    : 정책 행동 무시. 거리×하향각×모양 실험 기록 (CSV/JSON)
  residual : 목표각 = 스크립트 목표 + 잔차 (|잔차| ≤ residual_rad, 변화 ≤ residual_rate rad/s, DESCEND~CLOSE 만). 그리퍼는 스크립트
안전장치 (시뮬에서도 실기와 같은 규칙)
  - 관절 목표는 nz2_arm.json 한계 −2° 안쪽으로 자름, 관절 속도 ≤ max_velocity (팔 1 rad/s)
  - 직선 구간 IK 실패 시 그 자리 정지 + SKIP 기록 (엉뚱한 해로 튀지 않음)
  - 시작 시 FK 자기검사: 해석 FK 와 시뮬 arm_link5 위치 차이 > 20 mm 이면 중단 (부호·치수 불일치)
  - 앞 D435 충돌상자가 켜져 있으면 중단 (NZ2_ALLOW_D435=1 로만 허용)
"""
from __future__ import annotations

import csv
import json
import math
import os
import time
from collections.abc import Sequence

import torch

from isaaclab.envs.mdp.actions.joint_actions import JointPositionAction
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import math as math_utils

from arm_practice.tasks import nz2_arm_kin as K

P_WAIT, P_TO_VIA, P_DESCEND, P_APPROACH, P_CLOSE, P_LIFT, P_HOLD, P_DONE, P_SKIP, P_IDLE = range(10)
PHASE_NAMES = ["WAIT", "TO_VIA", "DESCEND", "APPROACH", "CLOSE", "LIFT", "HOLD", "DONE", "SKIP", "IDLE"]
NUM_PHASES = 10
D435_BOXES = ("chassis_vox_007", "chassis_vox_017", "chassis_vox_018",
              "chassis_vox_020", "chassis_vox_021", "chassis_vox_153")


class NZ2ScriptedPickAction(JointPositionAction):
    cfg: "object"

    def __init__(self, cfg: "object", env):
        super().__init__(cfg, env)
        if self.action_dim != 6:
            raise ValueError(f"관절 6개(arm_joint1~5, gripper_joint) 필요: {self._joint_names}")
        n, dev = self.num_envs, self.device
        self._env_ref = env
        self._dt = env.step_dt
        self._vmax = torch.tensor(cfg.joint_speed, device=dev)                    # (6,) rad/s
        self._lo = self._clip[0, :, 0].clone()
        self._hi = self._clip[0, :, 1].clone()
        self._home = self._asset.data.default_joint_pos.torch[0, self._joint_ids].clone()
        # 상태
        self.phase = torch.full((n,), P_IDLE, dtype=torch.long, device=dev)
        self.phase_t = torch.zeros(n, device=dev)                                 # 단계 경과 [s]
        self.seg_s = torch.zeros(n, device=dev)                                   # 직선 진행 0~1
        self.q_cmd = self._home.expand(n, 6).clone()
        self.q_from = self.q_cmd.clone()
        self.q_via = self.q_cmd.clone()
        self.via_T = torch.ones(n, device=dev)
        self.wps = torch.zeros(n, 4, 3, device=dev)                               # via, pre, grasp, lift
        self.pitch = torch.zeros(n, device=dev)
        self.plan_ok = torch.zeros(n, dtype=torch.bool, device=dev)
        self.plan_clear = torch.zeros(n, dtype=torch.bool, device=dev)
        self.done_flag = torch.zeros(n, dtype=torch.bool, device=dev)
        self.success_flag = torch.zeros(n, dtype=torch.bool, device=dev)
        # 기록용
        self.max_arm_f = torch.zeros(n, device=dev)
        self.max_fbad_f = torch.zeros(n, device=dev)
        self.worst_link = torch.full((n,), -1, dtype=torch.long, device=dev)
        self.pushed = torch.zeros(n, dtype=torch.bool, device=dev)
        self.tcp_err = torch.full((n,), float("nan"), device=dev)
        self.ik_fail = torch.zeros(n, dtype=torch.bool, device=dev)
        self.res = torch.zeros(n, 5, device=dev)                                   # 현재 잔차 [rad]
        self.q_int = torch.zeros(n, 5, device=dev)                                 # 처짐 보상 적분 [rad]
        self.fbad_ph = torch.zeros(n, NUM_PHASES, device=dev)                      # 단계별 손가락-선반/차체 최대 힘
        self.arm_ph = torch.zeros(n, NUM_PHASES, device=dev)                       # 단계별 팔 링크 최대 힘
        self.first_hit = torch.full((n,), -1, dtype=torch.long, device=dev)        # 처음 2 N 넘은 단계
        self.tcp_dz = torch.full((n,), float("nan"), device=dev)                   # grasp 순간 TCP 높이 오차 (실제−계획)
        self.records: list[dict] = []
        self._far_n, self._far_ok, self._far_next = 0, 0, 500
        self._written = False
        self._checked = False
        self._t0 = time.time()
        self._l5 = SceneEntityCfg("robot", body_names=["arm_link5"])
        self._l5.resolve(env.scene)
        self._check_d435()

    # ------------------------------------------------------------------
    # 시작 점검
    # ------------------------------------------------------------------
    def _check_d435(self):
        """앞 D435 충돌상자(URDF chassis_vox_007/017/018/020/021/153)가 꺼져 있는지 env_0 에서 확인"""
        if os.environ.get("NZ2_ALLOW_D435") == "1":
            print("[NZ2] NZ2_ALLOW_D435=1 → 앞 D435 충돌 검사 생략")
            return
        try:
            import isaaclab.sim as sim_utils
            from pxr import UsdPhysics
            stage = sim_utils.get_current_stage()
            root = stage.GetPrimAtPath("/World/envs/env_0/Robot")
            found, active = [], []
            from pxr import Usd
            for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
                if p.GetName() in D435_BOXES or any(p.GetName().endswith(b) for b in D435_BOXES):
                    found.append(p.GetName())
                    if p.HasAPI(UsdPhysics.CollisionAPI):
                        a = p.GetAttribute("physics:collisionEnabled")
                        if (not a) or a.Get() is None or bool(a.Get()):
                            active.append(str(p.GetPath()))
            if not found:
                print("[NZ2][경고] 앞 D435 충돌 prim 이름을 못 찾음 → disable_front_d435.py 결과(json usd_path)를 확인하세요")
            elif active:
                raise RuntimeError(
                    "앞 D435 충돌상자가 켜져 있습니다 (사용자 지시: 무시). R12 폴더에서 "
                    "`~/isaacsim/python.sh scripts/disable_front_d435.py ...` 실행 후 "
                    "ARM_CONFIG=.../isaac/nz2_arm_noD435.json 로 다시 실행하세요. 켜진 prim: " + ", ".join(active[:6]))
            else:
                print(f"[NZ2] 앞 D435 충돌상자 {len(found)}개 꺼짐 확인")
        except RuntimeError:
            raise
        except Exception as e:  # noqa: BLE001  (pxr 경로 차이 등은 경고만)
            print(f"[NZ2][경고] D435 검사 실패: {e}")

    def _fk_self_check(self):
        """해석 FK(arm_link5 원점) vs 시뮬 body 위치 (차체 기준). 부호·치수 불일치를 시작 때 잡는다"""
        r = self._asset.data
        q = r.joint_pos.torch[:, self._joint_ids][:, :5]
        pw = r.body_pos_w.torch[:, self._l5.body_ids[0]]
        p_rel, _ = math_utils.subtract_frame_transforms(r.root_pos_w.torch, r.root_quat_w.torch, pw)
        err = (K.fk_link5_origin(q) - p_rel).norm(dim=1)
        e = float(err.max())
        print(f"[NZ2] FK 자기검사: arm_link5 위치 오차 최대 {e * 1000:.1f} mm (q0={q[0].tolist()})")
        if e > 0.02:
            raise RuntimeError(f"해석 FK 와 시뮬 위치가 {e * 1000:.0f} mm 다름 → nz2_arm_kin 치수/부호 또는 팔 장착 위치 확인")
        self._checked = True

    # ------------------------------------------------------------------
    # 계획
    # ------------------------------------------------------------------
    def plan(self, env_ids: torch.Tensor):
        """mr_state 의 계획용 퍽 위치로 경유점·하향각을 정한다 (reset 에서 호출)"""
        from arm_practice.tasks import nz2_maxreach_cfg as M                    # 실행 시점 import
        st = M.mr_state(self._env_ref)
        ids = env_ids
        idle = st.finished[ids]
        puck = st.plan_puck[ids]
        gz = torch.full((len(ids),), M.GRASP_TCP_Z, device=self.device)
        pitch, wps, ok, clear = K.choose_plan(
            puck, gz, st.box_near[ids], st.box_far[ids], M.BOX_TOP, pitch_req=st.pitch_req[ids],
            pitch_max_deg=M.PITCH_MAX_DEG, step_deg=M.PITCH_STEP_DEG)
        self.pitch[ids] = torch.nan_to_num(pitch, nan=0.0)
        self.wps[ids] = wps
        self.plan_ok[ids] = ok
        self.plan_clear[ids] = clear
        if ok.any():
            q_via, okv = K.ik(wps[:, 0], self.pitch[ids])
            qv = self._home.expand(len(ids), 6).clone()
            qv[:, :5] = q_via
            qv[:, 5] = self._g_open()
            self.q_via[ids] = qv
            dq = (qv - self._home).abs()[:, :5].amax(dim=1)
            self.via_T[ids] = torch.clamp(dq / (0.6 * float(self._vmax[:5].min())), min=0.8)
        ph = torch.where(ok, torch.full_like(ids, P_WAIT), torch.full_like(ids, P_SKIP))
        self.phase[ids] = torch.where(idle, torch.full_like(ids, P_IDLE), ph)

    def _g_open(self):
        return float(self._hi[5])

    def _g_close(self):
        return float(self._lo[5])

    # ------------------------------------------------------------------
    # 매 스텝
    # ------------------------------------------------------------------
    def process_actions(self, actions: torch.Tensor):
        self._raw_actions[:] = actions
        if not self._checked:
            self._fk_self_check()
        self._step_machine()
        target = self.q_cmd.clone()
        # 처짐 보상(관절 I 제어): 중력으로 실제 각이 목표보다 처지면 그 차이를 천천히 적분해 목표에 더한다.
        #   시뮬 PD(강성 60 N·m/rad)·실기 다이나믹셀(P 만 쓰면 정상상태 오차) 모두 팔을 뻗으면 TCP 가 10~20 mm 처짐
        #   → 손끝이 박스 윗면(15 mm 여유)을 누름. 실기에서는 같은 로직을 arm_bridge 에 두거나 Position I 게인 사용
        if self.cfg.sag_ki > 0.0:
            moving = ((self.phase >= P_TO_VIA) & (self.phase <= P_HOLD)).unsqueeze(1)
            q_meas = self._asset.data.joint_pos.torch[:, self._joint_ids][:, :5]
            err = self.q_cmd[:, :5] - q_meas
            self.q_int = torch.clamp(self.q_int + self.cfg.sag_ki * err * self._dt * moving,
                                     -self.cfg.sag_max, self.cfg.sag_max)
            target[:, :5] = target[:, :5] + self.q_int
        if self.cfg.mode == "residual":
            # 잔차도 떨림 방지: 목표 잔차를 스텝당 residual_rate·dt 이하로만 따라감 (±1 반복 출력 → 진동 방지)
            active = ((self.phase <= P_CLOSE) & (self.phase >= P_DESCEND)).unsqueeze(1)
            want = torch.clamp(actions[:, :5], -1.0, 1.0) * self.cfg.residual_rad * active
            step = self.cfg.residual_rate * self._dt
            self.res = self.res + torch.clamp(want - self.res, -step, step)
            target[:, :5] = target[:, :5] + self.res
        target = torch.clamp(target, self._lo, self._hi)
        self._processed_actions = target

    def _cart_segment(self, mask, a_idx, b_idx, speed, next_phase):
        """mask 환경을 경유점 a→b 직선으로 speed[m/s] 로 이동. 끝나면 next_phase"""
        if not mask.any():
            return
        ids = mask.nonzero(as_tuple=False).squeeze(-1)
        a, b = self.wps[ids, a_idx], self.wps[ids, b_idx]
        L = (b - a).norm(dim=1).clamp(min=1e-4)
        s = torch.clamp(self.seg_s[ids] + speed * self._dt / L, max=1.0)
        p = a + (b - a) * s.unsqueeze(1)
        q, ok = K.ik(p, self.pitch[ids])
        good = ok
        qn = self.q_cmd[ids].clone()
        qn[good, :5] = q[good]
        self.q_cmd[ids] = qn
        self.seg_s[ids] = s
        # IK 실패: 그 자리 정지 + SKIP
        bad = ids[~good]
        if len(bad):
            self.ik_fail[bad] = True
            self.phase[bad] = P_DONE
        fin = ids[good & (s >= 1.0)]
        if len(fin):
            self.phase[fin] = next_phase
            self.phase_t[fin] = 0.0
            self.seg_s[fin] = 0.0

    def _step_machine(self):
        from arm_practice.tasks import nz2_maxreach_cfg as M
        env = self._env_ref
        dt = self._dt
        self.done_flag[:] = False
        self.phase_t += dt
        ph = self.phase
        # 접촉 최대값 추적 (시도 중인 환경만)
        running = (ph >= P_TO_VIA) & (ph <= P_HOLD)
        arm_f, worst = M.arm_contact_force(env)
        fb = M.finger_bad_force(env)
        upd = running & (arm_f > self.max_arm_f)
        self.worst_link[upd] = worst[upd]
        self.max_arm_f = torch.where(running, torch.maximum(self.max_arm_f, arm_f), self.max_arm_f)
        self.max_fbad_f = torch.where(running, torch.maximum(self.max_fbad_f, fb), self.max_fbad_f)
        # 진단: 어느 단계에서 닿았는지
        idx = ph.clamp(0, NUM_PHASES - 1).unsqueeze(1)
        self.fbad_ph.scatter_(1, idx, torch.maximum(self.fbad_ph.gather(1, idx), (fb * running).unsqueeze(1)))
        self.arm_ph.scatter_(1, idx, torch.maximum(self.arm_ph.gather(1, idx), (arm_f * running).unsqueeze(1)))
        hit = running & (self.first_hit < 0) & ((fb >= M.CONTACT_FAIL_N) | (arm_f >= M.CONTACT_FAIL_N))
        self.first_hit[hit] = ph[hit]
        # 0 WAIT
        m = (ph == P_WAIT) & (self.phase_t >= 0.3)
        self.q_from[m] = self.q_cmd[m]
        self.phase[m] = P_TO_VIA
        self.phase_t[m] = 0.0
        # 1 TO_VIA: 관절공간 동기 보간 (최소 저크 s = 3t²−2t³)
        m = self.phase == P_TO_VIA
        if m.any():
            tau = torch.clamp(self.phase_t / self.via_T, max=1.0)
            s = (3 * tau ** 2 - 2 * tau ** 3).unsqueeze(1)
            q = self.q_from + (self.q_via - self.q_from) * s
            self.q_cmd[m] = q[m]
            fin = m & (tau >= 1.0)
            self.phase[fin] = P_DESCEND
            self.phase_t[fin] = 0.0
            self.seg_s[fin] = 0.0
        # 2 DESCEND, 3 APPROACH
        self._cart_segment(self.phase == P_DESCEND, 0, 1, M.SPEED_DESCEND, P_APPROACH)
        before = self.phase.clone()
        self._cart_segment(self.phase == P_APPROACH, 1, 2, M.SPEED_APPROACH, P_CLOSE)
        # APPROACH 끝(= CLOSE 시작) 순간: 퍽 밀림·추종 오차 기록
        just = (before == P_APPROACH) & (self.phase == P_CLOSE)
        if just.any():
            self.pushed[just] = M.puck_disturbed(env)[just]
            tcp = M.tcp_in_root(env)
            self.tcp_err[just] = (tcp - self.wps[:, 2]).norm(dim=1)[just]
            self.tcp_dz[just] = (tcp[:, 2] - self.wps[:, 2, 2])[just]
        # 4 CLOSE
        m = self.phase == P_CLOSE
        self.q_cmd[m, 5] = self._g_close()
        fin = m & (self.phase_t >= M.CLOSE_S)
        self.phase[fin] = P_LIFT
        self.phase_t[fin] = 0.0
        self.seg_s[fin] = 0.0
        # 5 LIFT
        self._cart_segment(self.phase == P_LIFT, 2, 3, M.SPEED_LIFT, P_HOLD)
        # 6 HOLD
        fin = (self.phase == P_HOLD) & (self.phase_t >= M.HOLD_S)
        self.phase[fin] = P_DONE
        # 8 SKIP → DONE (계획 실패도 결과로 기록)
        sk = self.phase == P_SKIP
        self.phase[sk] = P_DONE
        # 그리퍼: CLOSE 전에는 열림
        early = self.phase <= P_APPROACH
        self.q_cmd[early, 5] = self._g_open()
        # 7 DONE: 기록 → 종료 신호
        d = self.phase == P_DONE
        if d.any():
            self._record(d.nonzero(as_tuple=False).squeeze(-1))
            self.done_flag[d] = True
            self.phase[d] = P_IDLE

    # ------------------------------------------------------------------
    # 기록
    # ------------------------------------------------------------------
    def _record(self, ids: torch.Tensor):
        from arm_practice.tasks import nz2_maxreach_cfg as M
        env = self._env_ref
        st = M.mr_state(env)
        lift = M.puck_lift(env)[ids]
        grasped = M.grasped_now(env)[ids]
        ok = self.plan_ok[ids]
        succ = (ok & ~self.ik_fail[ids] & grasped & (lift >= M.LIFT_OK_M) & ~self.pushed[ids]
                & (self.max_arm_f[ids] < M.CONTACT_FAIL_N) & (self.max_fbad_f[ids] < M.CONTACT_FAIL_N))
        self.success_flag[ids] = succ
        if self.cfg.mode != "sweep":          # 학습 중에는 보상용 성공 플래그만 (환경별 기록 생략 → 속도)
            self._far_n += len(ids)
            self._far_ok += int(succ.sum())
            if self._far_n >= self._far_next:
                print(f"[NZ2] PickFar 누적 시도 {self._far_n}, 성공률 {self._far_ok / self._far_n:.3f} "
                      f"(잔차 {self.cfg.residual_rad:.3f} rad)")
                self._far_next += 500
            return
        for j, e in enumerate(ids.tolist()):
            spec = st.spec_of(e)
            if spec is None:
                continue
            if not bool(ok[j]):
                reason = "도달불가(IK/한계/충돌근사)"
            elif bool(self.ik_fail[e]):
                reason = "직선구간 IK 실패"
            elif bool(self.pushed[e]):
                reason = "닫기 전 퍽 밀림/기울어짐"
            elif float(self.max_arm_f[e]) >= M.CONTACT_FAIL_N:
                reason = f"팔 링크 접촉({M.ARM_LINKS[int(self.worst_link[e])] if int(self.worst_link[e]) >= 0 else '?'})"
            elif float(self.max_fbad_f[e]) >= M.CONTACT_FAIL_N:
                reason = "손가락-선반/차체 접촉"
            elif not bool(grasped[j]):
                reason = "양손가락 접촉 없음"
            elif float(lift[j]) < M.LIFT_OK_M:
                reason = "들어올리기 실패(미끄러짐)"
            else:
                reason = "성공"
            rec = dict(spec)
            rec.update(dict(
                env=e, trial=int(st.trial_idx[e]), success=bool(succ[j]), reason=reason,
                plan_ok=bool(ok[j]), approx_clear=bool(self.plan_clear[e]),
                pitch_deg=round(math.degrees(float(self.pitch[e])), 1) if bool(ok[j]) else None,
                tcp_err_mm=round(float(self.tcp_err[e]) * 1000, 1) if not math.isnan(float(self.tcp_err[e])) else None,
                lift_mm=round(float(lift[j]) * 1000, 1),
                max_arm_contact_N=round(float(self.max_arm_f[e]), 2),
                max_finger_bad_N=round(float(self.max_fbad_f[e]), 2),
                tcp_dz_mm=round(float(self.tcp_dz[e]) * 1000, 1) if not math.isnan(float(self.tcp_dz[e])) else None,
                first_hit=PHASE_NAMES[int(self.first_hit[e])] if int(self.first_hit[e]) >= 0 else "-",
                fbad_move_N=round(float(self.fbad_ph[e, P_TO_VIA:P_APPROACH].max()), 1),
                fbad_approach_N=round(float(self.fbad_ph[e, P_APPROACH]), 1),
                fbad_close_N=round(float(self.fbad_ph[e, P_CLOSE]), 1),
                fbad_lift_N=round(float(self.fbad_ph[e, P_LIFT:P_HOLD + 1].max()), 1),
                arm_move_N=round(float(self.arm_ph[e, P_TO_VIA:P_APPROACH + 1].max()), 1),
                arm_grasp_N=round(float(self.arm_ph[e, P_CLOSE:P_HOLD + 1].max()), 1),
                sag_ki=self.cfg.sag_ki))
            self.records.append(rec)
            st.mark_done(e)
        if self.cfg.mode == "sweep" and not self._written and st.all_done():
            self._write()

    def _write(self):
        from arm_practice.tasks import nz2_maxreach_cfg as M
        st = M.mr_state(self._env_ref)
        out = os.path.join(os.environ.get("NZ2_OUT", os.path.join(os.getcwd(), "logs", "nz2_maxreach")),
                           f"{st.site}_{time.strftime('%Y-%m-%d_%H-%M-%S')}")
        os.makedirs(out, exist_ok=True)
        keys = sorted({k for r in self.records for k in r})
        with open(os.path.join(out, "trials.csv"), "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(self.records)
        summary = M.summarize(self.records)
        with open(os.path.join(out, "summary.json"), "w", encoding="utf-8") as f:
            json.dump(dict(site=st.site, env=M.run_settings(), elapsed_s=round(time.time() - self._t0, 1),
                           groups=summary), f, ensure_ascii=False, indent=1)
        print("\n" + M.summary_table(st.site, summary))
        print(f"\n[NZ2] 결과 저장: {out}/trials.csv, summary.json  — 모든 시험 완료. Ctrl+C 로 종료하세요")
        self._written = True
        if os.environ.get("NZ2_EXIT_WHEN_DONE") == "1":   # 여러 장소를 연달아 돌릴 때 (결과는 이미 저장됨)
            print("[NZ2] NZ2_EXIT_WHEN_DONE=1 → 종료")
            os._exit(0)

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        super().reset(env_ids)
        if env_ids is None or isinstance(env_ids, slice):
            ids = torch.arange(self.num_envs, device=self.device)
        else:
            ids = torch.as_tensor(env_ids, device=self.device, dtype=torch.long)
        self.q_cmd[ids] = self._home
        self.q_from[ids] = self._home
        self.phase_t[ids] = 0.0
        self.seg_s[ids] = 0.0
        self.max_arm_f[ids] = 0.0
        self.max_fbad_f[ids] = 0.0
        self.worst_link[ids] = -1
        self.pushed[ids] = False
        self.tcp_err[ids] = float("nan")
        self.ik_fail[ids] = False
        self.success_flag[ids] = False
        self.res[ids] = 0.0
        self.q_int[ids] = 0.0
        self.fbad_ph[ids] = 0.0
        self.arm_ph[ids] = 0.0
        self.first_hit[ids] = -1
        self.tcp_dz[ids] = float("nan")
        self.plan(ids)
