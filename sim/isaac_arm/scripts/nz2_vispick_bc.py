#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
nz2_vispick_bc.py — 전문가 시연(영상 + 상태 → 6차원 행동)으로 CNN 정책을 모방학습(BC)하고 rsl_rl 체크포인트로 저장

설치 위치: ~/manipulator-isaac61-lyrical-practice/scripts/nz2_vispick_bc.py   (Isaac 불필요, 일반 파이썬 + torch + rsl_rl)
실행 위치: [개발 PC]  bash nz2_vispick_run.sh bc [시연폴더]   (내부에서 아래를 실행)
    uv run --no-sync python scripts/nz2_vispick_bc.py --demo logs/nz2_demo/<시각> --template logs/rsl_rl/nz2_vispick/<실행>/model_<n>.pt

왜 템플릿 체크포인트가 필요한가
    같은 폴더 `bash nz2_vispick_run.sh check` (영상 RL 5회 점검) 가 만든 model_*.pt 를 받아
    ① 행동자(actor) 구조를 이 스크립트가 만든 것과 strict 로 맞춰 보고 (CNN·MLP 크기가 다르면 여기서 바로 실패)
    ② 비평가(critic)·옵티마이저 구조를 그대로 가져온다 (옵티마이저 상태는 비움). 행동자 가중치만 BC 결과로 교체.
    → 저장한 체크포인트를 `--checkpoint` 로 PPO 가 그대로 이어받는다 (init_std 도 BC 값 0.10 으로 시작 = 잔차처럼 작은 탐색)

학습 방식
    입력: 손목·상단 RGB-D (4×96×128 각각, 기록 때 이미 증강됨) + policy 벡터 (관절·실제목표·직전행동·명령색 …)
    출력: 행동 평균 (6) ← 전문가 행동과 MSE.  무작위 이동(±4 px) 증강, AdamW, cosine 감쇠, 검증 10 % 에피소드
    1D 관측 정규화: 시연 전체로 평균·표준편차를 맞춰 둠 (PPO 가 이어서 갱신)
안전장치: 시연 폴더·템플릿 없음/구조 불일치 → 즉시 종료(코드 2), NaN 손실 → 종료(코드 3), 저장 전 strict 재로드 검사
"""
from __future__ import annotations

import argparse
import copy
import glob
import json
import math
import os
import sys
import time

import torch
import torch.nn.functional as F

# ===== 상수 (한곳에서 바꿈) =====
CNN_CFG = dict(output_channels=[16, 32, 32, 32], kernel_size=[5, 3, 3, 3], stride=[2, 2, 2, 2], activation="relu")   # nz2_vispick_cfg 와 같아야 함
HIDDEN = [256, 128]
ACTIVATION = "elu"
INIT_STD = 0.10
ACTOR_GROUPS = ["policy", "wrist_cam", "top_cam"]


def build_actor(policy_dim: int, device: str):
    from tensordict import TensorDict
    from rsl_rl.models import CNNModel
    obs = TensorDict({"policy": torch.zeros(2, policy_dim), "wrist_cam": torch.zeros(2, 4, 96, 128),
                      "top_cam": torch.zeros(2, 4, 96, 128)}, batch_size=[2])
    actor = CNNModel(obs, {"actor": ACTOR_GROUPS}, "actor", 6, hidden_dims=HIDDEN, activation=ACTIVATION,
                     obs_normalization=True, cnn_cfg=copy.deepcopy(CNN_CFG),
                     distribution_cfg={"class_name": "GaussianDistribution", "init_std": INIT_STD}).to(device)
    return actor


def load_demos(folder: str, max_eps: int):
    files = sorted(glob.glob(os.path.join(folder, "ep_*.pt")))[:max_eps]
    if not files:
        print(f"[오류] {folder} 에 ep_*.pt 없음 → bash nz2_vispick_run.sh expert 로 시연 수집", file=sys.stderr)
        sys.exit(2)
    eps = []
    for f in files:
        d = torch.load(f, weights_only=False, map_location="cpu")
        eps.append({"policy": d["policy"].float(), "wrist": d["wrist"].half(), "top": d["top"].half(),
                    "action": d["action"].float().clamp(-1, 1), "phase": d["phase"].long(), "lying": d.get("lying", False)})
    return eps


def cat(eps, key):
    return torch.cat([e[key] for e in eps], dim=0)


def shift(img: torch.Tensor, pad: int = 4) -> torch.Tensor:
    """무작위 이동 증강 (가장자리 복제 패딩 후 원래 크기로 자름, 배치 전체 같은 이동이 아니라 샘플별)"""
    n, c, h, w = img.shape
    padded = F.pad(img, (pad, pad, pad, pad), mode="replicate")
    dx = torch.randint(0, 2 * pad + 1, (n,), device=img.device)
    dy = torch.randint(0, 2 * pad + 1, (n,), device=img.device)
    ys = (dy.view(n, 1) + torch.arange(h, device=img.device).view(1, h))
    xs = (dx.view(n, 1) + torch.arange(w, device=img.device).view(1, w))
    out = padded[torch.arange(n, device=img.device).view(n, 1, 1, 1), torch.arange(c, device=img.device).view(1, c, 1, 1),
                 ys.view(n, 1, h, 1), xs.view(n, 1, 1, w)]
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--demo", required=True, help="시연 폴더 (ep_*.pt)")
    ap.add_argument("--template", required=True, help="영상 RL 점검(check) 이 만든 model_*.pt (critic·optimizer 구조 출처)")
    ap.add_argument("--out", default=None, help="저장 경로 (기본: logs/rsl_rl/nz2_vispick/<시각>_bc/model_0.pt)")
    ap.add_argument("--epochs", type=int, default=20)
    ap.add_argument("--batch", type=int, default=64)
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--max_eps", type=int, default=100000)
    ap.add_argument("--val_frac", type=float, default=0.1)
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    a = ap.parse_args()
    torch.manual_seed(0)
    dev = a.device

    if not os.path.isfile(a.template):
        print(f"[오류] 템플릿 {a.template} 없음 → bash nz2_vispick_run.sh check 먼저", file=sys.stderr)
        sys.exit(2)
    tmpl = torch.load(a.template, weights_only=False, map_location="cpu")

    eps = load_demos(a.demo, a.max_eps)
    n_val = max(1, int(len(eps) * a.val_frac))
    val_eps, tr_eps = eps[:n_val], eps[n_val:]
    pol_dim = eps[0]["policy"].shape[1]
    print(f"[BC] 에피소드 {len(eps)} (학습 {len(tr_eps)} / 검증 {len(val_eps)}), 스텝 {sum(len(e['action']) for e in eps)}, policy 차원 {pol_dim}")

    actor = build_actor(pol_dim, dev)
    try:                                                    # 구조 검사: 템플릿 행동자와 키·모양이 같아야 함
        actor.load_state_dict(tmpl["actor_state_dict"], strict=True)
    except RuntimeError as e:
        print("[오류] 행동자 구조가 템플릿과 다름 → CNN_CFG / HIDDEN / ACTIVATION 을 nz2_vispick_cfg.py 와 맞추세요\n", str(e)[:1500], file=sys.stderr)
        sys.exit(2)
    actor = build_actor(pol_dim, dev)                       # 템플릿 가중치가 아니라 새 초기화에서 시작
    print("[BC] 행동자 구조: 템플릿과 일치")

    # 1D 관측 정규화를 시연 전체로 맞춤
    actor.train()
    pol_all = cat(tr_eps, "policy").to(dev)
    for chunk in torch.split(pol_all, 4096):
        actor.obs_normalizer.update(chunk)
    del pol_all

    X = {k: cat(tr_eps, k) for k in ("policy", "wrist", "top", "action", "phase")}
    V = {k: cat(val_eps, k) for k in ("policy", "wrist", "top", "action", "phase")}
    n_tr = X["action"].shape[0]
    opt = torch.optim.AdamW([p for n, p in actor.named_parameters() if "std" not in n], lr=a.lr, weight_decay=1e-4)
    steps_per_epoch = math.ceil(n_tr / a.batch)
    total = a.epochs * steps_per_epoch
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=total)

    def batch_obs(S, idx, aug):
        wr, tp = S["wrist"][idx].to(dev).float(), S["top"][idx].to(dev).float()
        if aug:
            wr, tp = shift(wr), shift(tp)
        return {"policy": S["policy"][idx].to(dev), "wrist_cam": wr, "top_cam": tp}

    @torch.no_grad()
    def evaluate():
        actor.eval()
        err, ph_err = [], {}
        for i in range(0, V["action"].shape[0], 128):
            idx = torch.arange(i, min(i + 128, V["action"].shape[0]))
            pred = actor(batch_obs(V, idx, False))
            e = ((pred - V["action"][idx].to(dev)) ** 2).cpu()
            err.append(e)
        e = torch.cat(err)
        for p in range(9):
            m = V["phase"] == p
            if m.any():
                ph_err[p] = float(e[m].mean())
        actor.train()
        return float(e.mean()), e.mean(0).tolist(), ph_err

    t0 = time.time()
    best = (1e9, None)
    for ep in range(a.epochs):
        perm = torch.randperm(n_tr)
        run = 0.0
        for k in range(steps_per_epoch):
            idx = perm[k * a.batch:(k + 1) * a.batch]
            pred = actor(batch_obs(X, idx, True))
            loss = F.mse_loss(pred, X["action"][idx].to(dev))
            if not torch.isfinite(loss):
                print("[오류] 손실이 NaN/inf → 학습률·데이터 확인", file=sys.stderr)
                sys.exit(3)
            opt.zero_grad(set_to_none=True)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(actor.parameters(), 1.0)
            opt.step()
            sched.step()
            run += float(loss.detach())
        v, per_dim, per_phase = evaluate()
        print(f"[BC] epoch {ep + 1}/{a.epochs}  train {run / steps_per_epoch:.4f}  val {v:.4f}  ({time.time() - t0:.0f} s)")
        if v < best[0]:
            best = (v, copy.deepcopy(actor.state_dict()))
    actor.load_state_dict(best[1])
    v, per_dim, per_phase = evaluate()
    names = ["j1", "j2", "j3", "j4", "j5", "grip"]
    pn = ["WAIT", "TO_VIA", "DESCEND", "APPROACH", "CLOSE", "LIFT", "HOLD", "DONE", "SKIP"]
    print(f"[BC] 최고 검증 MSE {v:.4f} | 관절별 " + ", ".join(f"{n} {x:.3f}" for n, x in zip(names, per_dim)))
    print("[BC] 단계별 MSE: " + ", ".join(f"{pn[p]} {x:.3f}" for p, x in per_phase.items()))

    out = a.out or os.path.join("logs", "rsl_rl", "nz2_vispick", time.strftime("%Y-%m-%d_%H-%M-%S") + "_bc", "model_0.pt")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    opt_sd = copy.deepcopy(tmpl["optimizer_state_dict"])
    opt_sd["state"] = {}                                     # 템플릿의 Adam 모멘트는 버림 (구조·학습률만 유지)
    ck = {"actor_state_dict": actor.state_dict(), "critic_state_dict": tmpl["critic_state_dict"],
          "optimizer_state_dict": opt_sd, "iter": 0, "infos": None}
    torch.save(ck, out)
    chk = build_actor(pol_dim, "cpu")                        # 저장 후 strict 재로드 검사
    chk.load_state_dict(torch.load(out, weights_only=False, map_location="cpu")["actor_state_dict"], strict=True)
    with open(os.path.splitext(out)[0] + "_report.json", "w", encoding="utf-8") as f:
        json.dump({"val_mse": v, "per_dim": dict(zip(names, per_dim)), "per_phase": {pn[p]: x for p, x in per_phase.items()},
                   "episodes": len(eps), "epochs": a.epochs, "init_std": INIT_STD, "demo": os.path.abspath(a.demo)}, f, ensure_ascii=False, indent=2)
    print(f"[BC] 저장: {os.path.abspath(out)}\n[BC] 다음: bash nz2_vispick_run.sh bc-rl {out}")


if __name__ == "__main__":
    main()
