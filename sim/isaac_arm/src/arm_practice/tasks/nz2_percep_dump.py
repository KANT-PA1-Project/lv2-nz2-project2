"""NZ2 퍽 인식 실험용 영상 저장 (NZ2_PERCEP_DUMP=<이름> 일 때만 동작)

설치 위치: ~/manipulator-isaac61-lyrical-practice/src/arm_practice/tasks/nz2_percep_dump.py
쓰는 곳: nz2_vispick_residual.py (상태기계가 정지한 순간에 dump 호출 = 예약),
        nz2_vispick_cfg.py (percep_cam 추가 + 매 스텝 interval 이벤트 percep_flush_event = 실제 저장)

손목 D435 자리에 실기 해상도(640×480) 카메라 percep_cam 을 하나 더 달고, 아래 세 순간에 영상과 정답을 저장한다
  look : WAIT 끝 (관측 자세, 박스 전체를 봄)
  via  : TO_VIA 끝, 정지 확인 후 (퍽 위 높은 곳)
  pre  : DESCEND 끝, 정지 확인 후 (퍽 위 근접 = 재관측 자리)
저장: ~/manipulator-isaac61-lyrical-practice/agent/percep/<이름>/<순간>_<번호>.npz
  rgb (480,640,3) uint8, depth (480,640) float32 [m] (시뮬 원값, D435 최소거리 처리는 평가 때), K (3,3),
  R_ac·t_ac (ROS 광학 → 팔 좌표), box_top, 퍽 4개 위치·자세(팔 좌표), 색별 모양·누움·박스 위 여부, 명령 색
팔 좌표 = 환경 원점 기준 (IK nz2_arm_kin 과 같은 좌표)

(11/11 v2) 029·031 은 행동 처리(process_actions, 렌더 전) 안에서 카메라를 읽어 빈 영상(RGB 단색, 깊이 inf/0).
  카메라 점검(cam_check, interval 이벤트 = 렌더 뒤)은 같은 카메라가 정상 → 읽는 시점을 이벤트로 옮김.
  dump() 는 환경별 '예약'만 하고, 같은 스텝의 interval 이벤트 flush() 가 영상·카메라 자세·퍽 정답을 함께 읽어 저장.
  look 은 예약 직후 TO_VIA 가 시작돼 팔이 한 스텝(33 ms) 움직이지만 카메라 자세도 같은 순간 값으로 저장하므로 정답은 일치.
빈 영상 검사: 빈 장은 저장하지 않고 건너뜀. 처음 BLANK_ABORT 장이 연속으로 비면 중단 (rc 3).
"""
from __future__ import annotations

import os

import numpy as np
import torch

from isaaclab.utils import math as math_utils

NAME = os.environ.get("NZ2_PERCEP_DUMP", "")
ENABLED = bool(NAME)
MAX_PER_TAG = int(os.environ.get("NZ2_PERCEP_MAX", "64"))
ROOT = os.path.expanduser(os.path.join("~/manipulator-isaac61-lyrical-practice/agent/percep", NAME)) if ENABLED else ""
SENSOR = os.environ.get("NZ2_PERCEP_SENSOR", "wrist_cam" if os.environ.get("NZ2_WRIST_FULLRES") == "1" else "percep_cam")
TAGS = ("look", "via", "pre")
BLANK_STD = float(os.environ.get("NZ2_PERCEP_BLANK_STD", "1.0"))      # RGB 표준편차(0~255) 이하면 단색
BLANK_FIN = float(os.environ.get("NZ2_PERCEP_BLANK_FIN", "0.05"))     # 깊이 유효(유한·양수) 비율 이하면 빈 장면
BLANK_ABORT = int(os.environ.get("NZ2_PERCEP_BLANK_ABORT", "8"))      # 저장 0장인 채로 빈 장이 이만큼 연속이면 중단

_count: dict[str, int] = {}
_pending: dict[str, torch.Tensor] = {}
_blank = {"run": 0, "total": 0, "saved": 0}


def _t(x):
    return x.torch if hasattr(x, "torch") else x


def dump(env, mask: torch.Tensor, tag: str):
    """예약: mask (N,) bool 인 환경을 이번 스텝 렌더 뒤(flush)에 저장"""
    if not ENABLED or not bool(mask.any()) or _count.get(tag, 0) >= MAX_PER_TAG:
        return
    p = _pending.get(tag)
    _pending[tag] = mask.clone() if p is None else (p | mask)


def flush_event(env, env_ids=None):
    """interval 이벤트 (매 스텝, 렌더 뒤). 예약된 환경의 영상 + 정답 저장"""
    if not ENABLED or not _pending:
        return
    from arm_practice.tasks import nz2_vispick_cfg as VC
    from arm_practice.tasks import nz2_vispick_geom as G
    cam = env.scene.sensors[SENSOR]
    out = cam.data.output
    rgb = _t(out["rgb"])[..., :3]
    dep = _t(out["distance_to_image_plane"])
    if dep.dim() == 4:
        dep = dep[..., 0]
    org = env.scene.env_origins
    pos = _t(cam.data.pos_w) - org
    R = math_utils.matrix_from_quat(_t(cam.data.quat_w_ros))
    Kmat = _t(cam.data.intrinsic_matrices)
    ppos, pquat = VC._puck_all(env)
    ppos = ppos - org.unsqueeze(1)
    st = VC.vp(env)
    os.makedirs(ROOT, exist_ok=True)
    for tag in TAGS:
        mask = _pending.pop(tag, None)
        if mask is None:
            continue
        n_done = _count.get(tag, 0)
        for e in mask.nonzero().squeeze(-1).tolist():
            if n_done >= MAX_PER_TAG:
                break
            std = float(rgb[e].float().std())
            fin = float((torch.isfinite(dep[e]) & (dep[e] > 0)).float().mean())
            if std < BLANK_STD or fin < BLANK_FIN:
                _blank["run"] += 1
                _blank["total"] += 1
                print(f"[NZ2-PERCEP][경고] 빈 영상 건너뜀 ({tag}, env {e}): RGB 표준편차 {std:.2f}, 깊이 유효 {fin:.3f} "
                      f"(연속 {_blank['run']}, 누적 {_blank['total']})", flush=True)
                if _blank["saved"] == 0 and _blank["run"] >= BLANK_ABORT:
                    print(f"[NZ2-PERCEP][오류] 처음 {BLANK_ABORT}장 연속 빈 영상 → 카메라 렌더 문제, 중단 (센서 {SENSOR})", flush=True)
                    os._exit(3)
                continue
            _blank["run"] = 0
            _blank["saved"] += 1
            path = os.path.join(ROOT, f"{tag}_{n_done:04d}.npz")
            np.savez_compressed(
                path,
                rgb=rgb[e].cpu().numpy().astype(np.uint8),
                depth=dep[e].float().cpu().numpy(),
                K=Kmat[e].cpu().numpy().astype(np.float64),
                R_ac=R[e].cpu().numpy().astype(np.float64), t_ac=pos[e].cpu().numpy().astype(np.float64),
                box_top=float(G.BOX[2]),
                puck_pos=ppos[e].cpu().numpy(), puck_quat=pquat[e].cpu().numpy(),       # 순서 VC.PUCKS: r_box, r_cyl, g_box, g_cyl
                shape=st.shape[e].cpu().numpy(), lying=st.lying[e].cpu().numpy(), on_box=st.on_box[e].cpu().numpy(),
                target_color=int(st.color[e]), env_id=e, tag=tag, sensor=SENSOR)
            if _blank["saved"] == 1:
                print(f"[NZ2-PERCEP] 첫 장 저장 OK ({tag}, env {e}, 센서 {SENSOR}): RGB 표준편차 {std:.1f}, 깊이 유효 {fin:.3f}",
                      flush=True)
            n_done += 1
        _count[tag] = n_done
        if n_done >= MAX_PER_TAG and _count.get("_said_" + tag) is None:
            _count["_said_" + tag] = 1
            print(f"[NZ2-PERCEP] {tag} {MAX_PER_TAG}장 저장 완료 → {ROOT} (빈 영상 건너뜀 누적 {_blank['total']})", flush=True)
