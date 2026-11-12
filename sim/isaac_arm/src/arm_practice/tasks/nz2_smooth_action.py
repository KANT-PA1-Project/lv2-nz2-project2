"""NZ2 떨림 방지 행동 클래스 (실행 시점에만 불러옴)

주의: 이 파일은 Isaac Lab 실행용 모듈(Articulation → pxr)을 import 한다.
  설정 파일(nz2_common / nz2_reach_cfg / nz2_pick_cfg)에서 직접 import 하면 Isaac Sim 이 뜨기 전에
  다른 버전의 pxr 가 먼저 올라가 omni.physx 로딩이 실패한다 (2026-11-03 확인).
  → 설정에서는 class_type 을 문자열 "arm_practice.tasks.nz2_smooth_action:NZ2SmoothJointPositionAction" 로만 가리킨다.

v3 (2026-11-03): mode="delta" 추가 (기본값)
  v2 의 절대 목표 방식(home + 1 rad·a)은 결정적 재생에서도 손끝이 떨렸다.
  원인: '정책(높은 이득) → EMA(지연) → 속도 제한(포화)' 고리가 목표 근처에서 리밋 사이클(자기 진동)을 만든다.
  delta 방식은 행동 = 관절 속도 명령이라 a = 0 이면 그 자리에 정확히 멈춘다.
  action_l2 벌점 = 속도 벌점, action_rate 벌점 = 가속도 벌점이 되어 '멈춰 있기'가 자연스럽게 가장 싸다.
"""
from __future__ import annotations

import math
from collections.abc import Sequence

import torch

import isaaclab.utils.string as string_utils
from isaaclab.envs.mdp.actions.joint_actions import JointPositionAction


class NZ2SmoothJointPositionAction(JointPositionAction):
    """관절 위치 행동 + 떨림 방지.

    mode="delta" (v3 기본)
        a_c   = clamp(a, −1, 1)                               (1 = 관절 최대 속도)
        cmd   = α·a_c + (1−α)·이전cmd                         (가속도 완화)
        목표  = 이전목표 + cmd · v_max · step_dt
        목표  = clamp(목표, 현재각 ± max_lead)                (막혔을 때 목표가 멀리 달아나는 와인드업 방지)
        목표  = clamp(목표, 관절 한계 안쪽)
    mode="absolute" (v2, 비교용)
        목표_raw = clip(home + scale·a) → EMA → 스텝당 변화 ≤ v_max·step_dt
    """

    cfg: "object"

    def __init__(self, cfg: "object", env):
        super().__init__(cfg, env)
        if cfg.mode not in ("delta", "absolute"):
            raise ValueError(f"mode 는 'delta' 또는 'absolute': {cfg.mode}")
        if not 0.0 < cfg.alpha <= 1.0:
            raise ValueError(f"alpha 는 (0, 1] 이어야 함: {cfg.alpha}")
        if not (math.isfinite(cfg.max_lead) and cfg.max_lead > 0):
            raise ValueError(f"max_lead 는 양수여야 함: {cfg.max_lead}")
        self._mode = cfg.mode
        self._alpha = float(cfg.alpha)
        self._max_lead = float(cfg.max_lead)
        # 스텝당 최대 변화량 [rad]
        self._max_step = torch.full((self.num_envs, self.action_dim), float("inf"), device=self.device)
        idx, names, vals = string_utils.resolve_matching_names_values(cfg.max_velocity, self._joint_names)
        if len(idx) != self.action_dim:
            raise ValueError(f"max_velocity 가 모든 관절을 덮지 않음: {names} / {self._joint_names}")
        if any((not math.isfinite(v)) or v <= 0 for v in vals):
            raise ValueError(f"max_velocity 는 양수여야 함: {vals}")
        self._max_step[:, idx] = torch.tensor(vals, device=self.device) * env.step_dt
        # 이전 목표 = 현재 관절 위치, 이전 속도 명령 = 0 에서 시작
        self._prev_target = self._asset.data.joint_pos.torch[:, self._joint_ids].clone()
        self._prev_cmd = torch.zeros_like(self._prev_target)

    def _clip_limits(self, target: torch.Tensor) -> torch.Tensor:
        if self.cfg.clip is not None:                          # 관절 한계 안쪽 유지
            target = torch.clamp(target, min=self._clip[:, :, 0], max=self._clip[:, :, 1])
        return target

    def process_actions(self, actions: torch.Tensor):
        super().process_actions(actions)                       # self._raw_actions 저장 (+ 절대 목표 계산)
        if self._mode == "delta":
            a = torch.clamp(self._raw_actions, -1.0, 1.0)
            cmd = self._alpha * a + (1.0 - self._alpha) * self._prev_cmd
            target = self._prev_target + cmd * self._max_step
            q = self._asset.data.joint_pos.torch[:, self._joint_ids]
            target = torch.clamp(target, q - self._max_lead, q + self._max_lead)
            self._prev_cmd[:] = cmd
        else:
            goal = self._alpha * self._processed_actions + (1.0 - self._alpha) * self._prev_target
            step = torch.clamp(goal - self._prev_target, -self._max_step, self._max_step)
            target = self._prev_target + step
        target = self._clip_limits(target)
        self._processed_actions = target
        self._prev_target[:] = target

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        super().reset(env_ids)
        q = self._asset.data.joint_pos.torch[:, self._joint_ids]
        if env_ids is None:
            self._prev_target[:] = q
            self._prev_cmd[:] = 0.0
        else:
            ids = torch.as_tensor(env_ids, device=self.device, dtype=torch.long)
            self._prev_target[ids] = q[ids]
            self._prev_cmd[ids] = 0.0
