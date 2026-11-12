"""NZ2 영상 집기 실험 — 카메라 관측(RGB-D 묶음·자르기·증강)과 카메라 점검(영상 저장·퍽 투영 검사)

설치 위치: ~/manipulator-isaac61-lyrical-practice/src/arm_practice/tasks/nz2_vispick_obs.py
쓰는 곳: nz2_vispick_cfg.py

정책 입력 영상 형식 (실기 Jetson 에서도 똑같이 만들어야 함 — 'sim2real 계약')
  채널 4 × 96 × 128, float32
    0~2: RGB / 255 − 0.5
    3  : 깊이 [m] / DEPTH_MAX_M (0~1). 무효(0, inf, DEPTH_MIN_M 보다 가까움) = 0  ← D435 의 '깊이 0 = 없음' 과 같게
  손목: 640×480 → 128×96 로 축소 (area 평균)
  상단: 640×480 에서 (u0, v0, 320, 240) × 2.5 영역 = 렌더 256×192 의 (u0, v0, 128, 96) 를 잘라 128×96 로 축소
        u0, v0 는 nz2_vispick_geom.top_crop() (하향각 65° 기본: 렌더 기준 (64, 80) → 실기 (160, 200, 320, 240))
  컬러·깊이 정렬: 실기는 rs2::align 으로 깊이를 컬러 화면에 맞춘 뒤 위 처리 [실기 확인 필요]
"""
from __future__ import annotations

import json
import math
import os
import time

import torch

from arm_practice.tasks import nz2_vispick_geom as G

# ===== 증강 (학습 때만, NZ2_PLAY=1 이면 끔) [가정] =====
AUG_BRIGHT = (0.70, 1.30)                 # 밝기 배율 (환경·에피소드마다)
AUG_CONTRAST = (0.80, 1.20)
AUG_CH_GAIN = (0.92, 1.08)                # 채널별 배율 (조명 색온도). R/G 판별이 뒤집히지 않을 만큼만
AUG_RGB_NOISE = 0.02                      # 화소 잡음 σ (0~1 단위)
AUG_DEPTH_NOISE_M = 0.004                 # 깊이 잡음 σ @1 m (거리² 비례)
AUG_DEPTH_DROP = 0.02                     # 깊이 화소 무작위 0 처리 비율 (D435 구멍 흉내)


def _t(x):
    """Isaac Lab 3.0 데이터(ProxyArray) → torch"""
    return x.torch if hasattr(x, "torch") else x


def _aug_state(env, key: str):
    st = getattr(env, "_nz2_aug", None)
    if st is None:
        st = {}
        env._nz2_aug = st
    if key not in st:
        n, dev = env.num_envs, env.device
        st[key] = {"bright": torch.ones(n, 1, 1, 1, device=dev), "contrast": torch.ones(n, 1, 1, 1, device=dev),
                   "gain": torch.ones(n, 3, 1, 1, device=dev), "fresh": torch.ones(n, dtype=torch.bool, device=dev)}
    return st[key]


def _resample_aug(env, a):
    """에피소드 시작한 환경만 증강 계수를 새로 뽑는다 (한 에피소드 동안은 같은 조명)"""
    buf = getattr(env, "episode_length_buf", None)
    new = a["fresh"] | ((buf == 0) if buf is not None else torch.zeros_like(a["fresh"]))
    if not bool(new.any()):
        return
    k = int(new.sum())
    dev = env.device
    U = lambda lo, hi, *shape: lo + (hi - lo) * torch.rand(k, *shape, device=dev)
    a["bright"][new] = U(*AUG_BRIGHT, 1, 1, 1)
    a["contrast"][new] = U(*AUG_CONTRAST, 1, 1, 1)
    a["gain"][new] = U(*AUG_CH_GAIN, 3, 1, 1)
    a["fresh"][:] = False


def rgbd_tensor(env, sensor_name: str, crop: tuple[int, int] | None = None) -> torch.Tensor:
    """카메라 → (N, 4, OBS_H, OBS_W). 증강 전 '깨끗한' 영상 (RGB 0~1, 깊이 0~1)"""
    cam = env.scene.sensors[sensor_name]
    out = cam.data.output
    rgb = _t(out["rgb"])[..., :3].float() / 255.0                       # (N, H, W, 3)
    d = _t(out["distance_to_image_plane"]).float()
    if d.dim() == 4:
        d = d[..., 0]
    valid = torch.isfinite(d) & (d >= G.DEPTH_MIN_M)
    d = torch.where(valid, d, torch.zeros_like(d))
    dn = (d / G.DEPTH_MAX_M).clamp(0.0, 1.0)
    img = torch.cat([rgb.permute(0, 3, 1, 2), dn.unsqueeze(1)], dim=1)   # (N, 4, H, W)
    if crop is not None:
        u0, v0 = crop
        img = img[:, :, v0:v0 + G.OBS_H, u0:u0 + G.OBS_W]
    if crop is None and img.shape[-2:] != (G.OBS_H, G.OBS_W):
        # (11/11) 640×480 등 큰 렌더 → 관측 크기로 줄임. RGB 는 평균, 깊이는 최근접(무효 0 이 섞이지 않게)
        import torch.nn.functional as F
        rgb_s = F.interpolate(img[:, :3], size=(G.OBS_H, G.OBS_W), mode="area")
        dep_s = F.interpolate(img[:, 3:], size=(G.OBS_H, G.OBS_W), mode="nearest")
        img = torch.cat([rgb_s, dep_s], dim=1)
    if img.shape[-2:] != (G.OBS_H, G.OBS_W):
        raise RuntimeError(f"[NZ2] {sensor_name} 영상 크기 {tuple(img.shape[-2:])} ≠ {(G.OBS_H, G.OBS_W)} (렌더 크기·자르기 확인)")
    return img


def obs_cam_rgbd(env, sensor_name: str, crop: tuple[int, int] | None = None, augment: bool = True) -> torch.Tensor:
    """관측 항: 정책 입력 영상 (N, 4, 96, 128). RGB 는 −0.5~0.5 로 가운데 맞춤"""
    img = rgbd_tensor(env, sensor_name, crop).clone()
    rgb, dep = img[:, :3], img[:, 3:]
    if augment and os.environ.get("NZ2_PLAY") != "1":
        a = _aug_state(env, sensor_name)
        _resample_aug(env, a)
        mean = rgb.mean(dim=(1, 2, 3), keepdim=True)
        rgb = ((rgb - mean) * a["contrast"] + mean) * a["bright"] * a["gain"]
        rgb = rgb + AUG_RGB_NOISE * torch.randn_like(rgb)
        dm = dep * G.DEPTH_MAX_M
        dm = dm + AUG_DEPTH_NOISE_M * dm * dm * torch.randn_like(dm)
        drop = torch.rand_like(dm) < AUG_DEPTH_DROP
        dm = torch.where(drop | (dep <= 0), torch.zeros_like(dm), dm)
        dep = (dm / G.DEPTH_MAX_M).clamp(0.0, 1.0)
        rgb = rgb.clamp(0.0, 1.0)
    return torch.cat([rgb - 0.5, dep], dim=1).contiguous()


# ---------------------------------------------------------------------------
# 카메라 점검 (NZ2_CAM_CHECK=1): 영상 저장 + 내용 검사 + 목표 퍽 투영 검사
# ---------------------------------------------------------------------------
def _project_ros(env, sensor_name: str, p_w: torch.Tensor):
    """월드 점 (N,3) → 화소 (u, v), 광축 거리 z. 카메라 자세는 data.pos_w / quat_w_ros (ROS: z 앞, x 오른쪽, y 아래)"""
    from isaaclab.utils.math import quat_apply_inverse      # 실행 시점 import
    cam = env.scene.sensors[sensor_name]
    pos = _t(cam.data.pos_w).float()
    quat = _t(cam.data.quat_w_ros).float()
    k = _t(cam.data.intrinsic_matrices).float()
    pc = quat_apply_inverse(quat, p_w - pos)
    z = pc[:, 2]
    u = k[:, 0, 0] * pc[:, 0] / z + k[:, 0, 2]
    v = k[:, 1, 1] * pc[:, 1] / z + k[:, 1, 2]
    return u, v, z


def _save_png(path: str, arr):
    import numpy as np
    try:
        from PIL import Image
        Image.fromarray(arr).save(path)
    except Exception:                                            # PIL 이 없으면 npy
        np.save(os.path.splitext(path)[0] + ".npy", arr)


def cam_check_event(env, env_ids, cams: dict, target_fn, n_checks: int = 4):
    """interval 이벤트 (1 s 마다). cams = {센서이름: 자르기(None 또는 (u0, v0))}, target_fn(env) → 목표 퍽 월드 위치 (N,3)
    검사: ① 영상이 단색이 아님(표준편차 > 0.01) ② 깊이 유효 비율 ③ 목표 퍽 중심이 화면(자르기 창) 안 ④ 그 화소 깊이 ≈ 퍽까지 거리(±40 mm)
    n_checks 번 뒤 summary.json 저장. NZ2_EXIT_WHEN_DONE=1 이면 종료 (모두 통과 0, 아니면 3)"""
    import numpy as np
    st = getattr(env, "_nz2_camchk", None)
    if st is None:
        root = os.path.join("logs", "nz2_vispick", "camcheck_" + time.strftime("%Y%m%d_%H%M%S"))
        os.makedirs(root, exist_ok=True)
        st = {"k": 0, "root": root, "rows": []}
        env._nz2_camchk = st
        print(f"[NZ2-CAM] 카메라 점검 시작 → {os.path.abspath(root)}")
    st["k"] += 1
    p_t = target_fn(env)
    n_show = min(4, env.num_envs)
    for name, crop in cams.items():
        img = rgbd_tensor(env, name, crop)                      # 증강 없음
        rgb, dep = img[:, :3], img[:, 3]
        std = rgb.flatten(1).std(dim=1)
        nonflat = (std > 0.01).float().mean().item()
        dvalid = (dep > 0).float().mean().item()
        u, v, z = _project_ros(env, name, p_t)
        if crop is not None:
            u, v = u - crop[0], v - crop[1]
        else:                                                   # 렌더가 관측보다 크면 (640×480) 관측 화소로 환산
            sc = G.OBS_W / float(env.scene.sensors[name].cfg.width)
            u, v = u * sc, v * sc
        # 렌더 크기 → 관측 크기 (손목은 같음, 상단은 자른 뒤 같음)
        inside = (u >= 0) & (u < G.OBS_W) & (v >= 0) & (v < G.OBS_H) & (z > 0)
        ui = u.clamp(0, G.OBS_W - 1).long()
        vi = v.clamp(0, G.OBS_H - 1).long()
        d_pix = dep[torch.arange(env.num_envs, device=env.device), vi, ui] * G.DEPTH_MAX_M
        depth_ok = inside & ((d_pix - z).abs() < 0.04)
        row = {"check": st["k"], "camera": name, "non_flat": round(nonflat, 3), "depth_valid": round(dvalid, 3),
               "puck_in_view": round(inside.float().mean().item(), 3),
               "puck_depth_match": round(depth_ok.float().mean().item(), 3),
               "puck_px_env0": (round(u[0].item(), 1), round(v[0].item(), 1)), "puck_z_env0_m": round(z[0].item(), 3)}
        st["rows"].append(row)
        print(f"[NZ2-CAM] #{st['k']} {name:9s} 단색아님 {row['non_flat']:.2f} 깊이유효 {row['depth_valid']:.2f} "
              f"퍽화면안 {row['puck_in_view']:.2f} 퍽깊이일치 {row['puck_depth_match']:.2f} env0 퍽 (u,v)={row['puck_px_env0']}")
        if st["k"] in (1, n_checks):
            for e in range(n_show):
                a = (rgb[e].permute(1, 2, 0).clamp(0, 1) * 255).byte().cpu().numpy()
                if bool(inside[e]):                               # 목표 퍽 투영 위치에 십자 표시
                    uu, vv = int(ui[e]), int(vi[e])
                    a[max(vv - 3, 0):vv + 4, uu] = (255, 255, 0)
                    a[vv, max(uu - 3, 0):uu + 4] = (255, 255, 0)
                _save_png(os.path.join(st["root"], f"c{st['k']}_{name}_env{e}_rgb.png"), a)
                dd = (dep[e].clamp(0, 1) * 255).byte().cpu().numpy()
                _save_png(os.path.join(st["root"], f"c{st['k']}_{name}_env{e}_depth.png"), np.ascontiguousarray(dd))
    if st["k"] >= n_checks:
        last = [r for r in st["rows"] if r["check"] == n_checks]
        ok = all(r["non_flat"] >= 0.99 and r["depth_valid"] >= 0.5 and r["puck_in_view"] >= 0.95
                 and r["puck_depth_match"] >= 0.8 for r in last)
        summary = {"ok": ok, "rows": st["rows"], "geom": G.report(float(os.environ.get("NZ2_TOPCAM_PITCH_DEG", G.TOP_CAM_PITCH_DEG_DEFAULT))),
                   "settings": {k: v for k, v in os.environ.items() if k.startswith("NZ2_") or k == "ARM_CONFIG"}}
        with open(os.path.join(st["root"], "summary.json"), "w", encoding="utf-8") as f:
            json.dump(summary, f, ensure_ascii=False, indent=2)
        print(f"[NZ2-CAM] 결과 {'통과' if ok else '실패'} → {os.path.abspath(st['root'])}/summary.json")
        print("[NZ2-CAM] 기준: 단색아님 ≥ 0.99, 깊이유효 ≥ 0.5, 퍽화면안 ≥ 0.95, 퍽깊이일치 ≥ 0.8 (마지막 점검 기준)")
        if os.environ.get("NZ2_EXIT_WHEN_DONE") == "1":
            os._exit(0 if ok else 3)
