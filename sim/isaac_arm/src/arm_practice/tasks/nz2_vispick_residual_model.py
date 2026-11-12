"""NZ2 잔차 RL 정책 모델 — rsl_rl CNNModel + 마지막 층 0 초기화 + 비평가 워밍업(행동자 고정)

설치 위치: ~/manipulator-isaac61-lyrical-practice/src/arm_practice/tasks/nz2_vispick_residual_model.py
쓰는 곳: nz2_vispick_cfg.NZ2VisPickResidualPPORunnerCfg 의 actor.class_name
         = "arm_practice.tasks.nz2_vispick_residual_model:ZeroLastCNNModel" (rsl_rl resolve_callable 이 불러옴)
Isaac 모듈을 import 하지 않는다 (rsl_rl·torch 만).

왜
  - 마지막 Linear 의 가중치·편향 = 0 → 시작 출력 π = 0 → tanh(0) = 0 → 잔차 0 = IK 단독과 같은 성능에서 출발
    (그래디언트는 마지막 층부터 흐르므로 학습은 정상적으로 시작됨)
  - 비평가 워밍업: 처음 NZ2_RES_WARMUP_ITERS(기본 50) 반복 동안 행동자 그래디언트를 0 으로 만들어 고정.
    비평가는 무작위에서 시작하므로 가치 추정이 엉망인 동안 행동자가 끌려가지 않게 한다.
    반복당 역전파 횟수 = num_learning_epochs × num_mini_batches (NZ2_RES_UPDATES_PER_ITER, 기본 5×4 = 20)
    주의: 학습률 schedule 이 'adaptive' 면 고정 중 KL≈0 → 학습률이 1e-2 까지 올라감 → 반드시 'fixed'
"""
from __future__ import annotations

import os

import torch
import torch.nn as nn

from rsl_rl.models import CNNModel


class ZeroLastCNNModel(CNNModel):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        last = None
        for m in self.mlp.modules():
            if isinstance(m, nn.Linear):
                last = m
        if last is None:
            raise RuntimeError("[NZ2-RES] MLP 에 Linear 층이 없음")
        with torch.no_grad():
            last.weight.zero_()
            last.bias.zero_()
        # 비평가 워밍업: 행동자 전체(CNN·MLP·std) 그래디언트를 일정 횟수 동안 0 으로
        iters = int(os.environ.get("NZ2_RES_WARMUP_ITERS", "50"))
        per = int(os.environ.get("NZ2_RES_UPDATES_PER_ITER", "20"))
        self._freeze_left = max(0, iters * per)
        self._hooked = []
        if self._freeze_left > 0:
            params = [p for p in self.parameters() if p.requires_grad]
            for i, p in enumerate(params):
                self._hooked.append(p.register_hook(self._make_hook(i == 0)))
        print(f"[NZ2-RES] 행동자: 마지막 층 0 초기화 ({last.in_features}→{last.out_features}), "
              f"워밍업 고정 {iters} 반복 (역전파 {self._freeze_left}회)")

    def _make_hook(self, counter: bool):
        def hook(grad):
            if self._freeze_left <= 0:
                return grad
            if counter:                                     # 역전파 1회당 한 번만 센다
                self._freeze_left -= 1
                if self._freeze_left == 0:
                    print("[NZ2-RES] 비평가 워밍업 끝 → 행동자 학습 시작")
            return torch.zeros_like(grad)
        return hook
