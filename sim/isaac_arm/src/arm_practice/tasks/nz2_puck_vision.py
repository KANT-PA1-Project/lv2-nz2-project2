"""NZ2 퍽 인식 핵심 (ROS·Isaac 둘 다 쓰는 순수 numpy + OpenCV 모듈)

설치 위치
  시뮬: ~/manipulator-isaac61-lyrical-practice/src/arm_practice/tasks/nz2_puck_vision.py
  실기: nz2_perception 패키지에 같은 파일을 복사 (ROS 노드는 이 함수만 부른다 → 시뮬에서 잰 오차가 실기와 같은 코드)

입력: 컬러 RGB(H,W,3 uint8), 컬러에 정렬된 깊이(H,W, m, 0 = 무효), 내부행렬 K(3×3),
      카메라 자세 R_ac, t_ac (ROS 광학 좌표 z 앞·x 오른쪽·y 아래 → 팔 베이스 좌표)
      박스 윗면 높이 box_top_z [m] (팔 좌표). mission v5: 바닥 위 70 mm
출력: 퍽 목록 (색·세움/누움·모양·중심 xy·yaw·윗면 z)

방법
  1. HSV 로 빨강/초록 분할 → 연결 성분 (색마다 여러 개 가능)
  2. 성분 화소 → 깊이로 3D 점 (팔 좌표). 박스 윗면 −5 ~ +75 mm 밖 점은 버림
  3. 높이(상위 2 % z − 박스 윗면) < 45 mm 면 누움, 아니면 세움 (퍽 60 / 30 mm)
  4. 세움: 윗면 띠(최고점 −6 mm) 점들의 최소외접사각형 중심 = 퍽 축.
          모양 = 볼록껍질 넓이 / 사각형 넓이 (원 π/4 = 0.79, 정사각형 1.0) → 0.88 기준
     누움: 능선 띠(최고점 −4 mm) = 축 바로 위 → 그 사각형 중심 = 축 xy. 원기둥 능선 폭 약 20 mm, 각기둥 윗면 30 mm → 25 mm 기준
          yaw = 박스 윗면 +10 mm 위 점들의 사각형 긴 변 방향
  5. 윗면 z 는 측정 대신 박스 높이 + 퍽 치수(세움 60 / 누움 30 mm)로 정함 (D435 깊이 오차가 z 에 들어가지 않게)
모든 임계값은 PARAMS 한곳에서 바꿈. 실물 HSV 는 조명 아래에서 다시 재야 함 [가정]
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import cv2
import numpy as np

PARAMS = dict(
    # HSV (OpenCV: H 0~179, S·V 0~255) [가정: 시뮬 퍽 색 빨강 (0.85,0.10,0.10), 초록 (0.10,0.65,0.20)]
    red_h=((0, 10), (170, 179)), green_h=((40, 90),),
    s_min=110, v_min=40,
    min_area_px=150,               # 640×480 기준. 0.5 m 에서 퍽 옆면 약 37×74 px
    open_px=3, close_px=5,
    depth_min_m=0.175, depth_max_m=1.2,   # D435 640×480 최소 거리 [가정]
    z_lo_m=-0.005, z_hi_m=0.075,          # 박스 윗면 기준 점 높이 범위
    lie_h_m=0.045,                         # 이보다 낮으면 누움
    stand_band_m=0.006, ridge_band_m=0.004, lie_upper_m=0.010,
    stand_round_ratio=0.88,                # 볼록껍질/사각형 넓이 < 이면 원기둥
    lie_ridge_w_m=0.025,                   # 능선 폭 < 이면 원기둥
    min_pts=40, min_valid_frac=0.3,
    puck_h_m=0.060, puck_w_m=0.030,
)


@dataclass
class Puck:
    color: str                 # "r" | "g"
    lying: bool
    shape: str                 # "box" | "cyl"
    xy: np.ndarray             # 퍽 축 중심 (팔 좌표) [m]
    top_z: float               # 윗면 z (박스 높이 + 치수) [m]
    yaw: float                 # 누운 퍽 축 방향 [rad] (−π/2~π/2). 세운 퍽: 각기둥 면 방향 (−π/4~π/4), 원기둥 0
    h_meas: float              # 잰 높이 (박스 위) [m]
    n_px: int
    valid_frac: float
    info: dict = field(default_factory=dict)


def color_masks(rgb: np.ndarray, p: dict = PARAMS) -> dict[str, np.ndarray]:
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    sv = (s >= p["s_min"]) & (v >= p["v_min"])
    out = {}
    for name, key in (("r", "red_h"), ("g", "green_h")):
        m = np.zeros(h.shape, bool)
        for lo, hi in p[key]:
            m |= (h >= lo) & (h <= hi)
        m = (m & sv).astype(np.uint8)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((p["open_px"],) * 2, np.uint8))
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((p["close_px"],) * 2, np.uint8))
        out[name] = m
    return out


def backproject(us, vs, z, K, R_ac, t_ac):
    """화소 중심(u+0.5, v+0.5)·깊이 → 팔 좌표 점 (n,3)"""
    x = (us + 0.5 - K[0, 2]) * z / K[0, 0]
    y = (vs + 0.5 - K[1, 2]) * z / K[1, 1]
    pc = np.stack([x, y, z], axis=1)
    return pc @ R_ac.T + t_ac


def _rect(xy: np.ndarray):
    """xy (n,2) [m] → 최소외접사각형 (중심, (w,h), 각도 deg), mm 단위로 계산"""
    pts = (xy * 1000.0).astype(np.float32)
    (cx, cy), (w, h), a = cv2.minAreaRect(pts)
    return np.array([cx, cy]) / 1000.0, (w / 1000.0, h / 1000.0), a, pts


def _long_axis_yaw(size, ang_deg):
    w, h = size
    a = math.radians(ang_deg)
    if h > w:
        a += math.pi / 2
    return (a + math.pi / 2) % math.pi - math.pi / 2


def detect(rgb, depth, K, R_ac, t_ac, box_top_z: float, p: dict = PARAMS) -> list[Puck]:
    K = np.asarray(K, float)
    R_ac = np.asarray(R_ac, float)
    t_ac = np.asarray(t_ac, float)
    d = np.asarray(depth, float)
    d = np.where(np.isfinite(d) & (d >= p["depth_min_m"]) & (d <= p["depth_max_m"]), d, 0.0)
    out: list[Puck] = []
    for col, m in color_masks(rgb, p).items():
        n, lab, stats, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
        for i in range(1, n):
            area = int(stats[i, cv2.CC_STAT_AREA])
            if area < p["min_area_px"]:
                continue
            vs, us = np.nonzero(lab == i)
            z = d[vs, us]
            ok = z > 0
            vf = float(ok.mean())
            info = {"bbox": stats[i, :4].tolist()}
            if vf < p["min_valid_frac"] or ok.sum() < p["min_pts"]:
                info["reject"] = "depth_invalid"
                out.append(Puck(col, False, "?", np.full(2, np.nan), float("nan"), float("nan"), float("nan"), area, vf, info))
                continue
            P = backproject(us[ok].astype(float), vs[ok].astype(float), z[ok], K, R_ac, t_ac)
            rel = P[:, 2] - box_top_z
            keep = (rel >= p["z_lo_m"]) & (rel <= p["z_hi_m"])
            P, rel = P[keep], rel[keep]
            if len(P) < p["min_pts"]:
                info["reject"] = "few_points"
                out.append(Puck(col, False, "?", np.full(2, np.nan), float("nan"), float("nan"), float("nan"), area, vf, info))
                continue
            hmax = float(np.percentile(rel, 98))
            lying = hmax < p["lie_h_m"]
            if not lying:
                top = P[rel >= hmax - p["stand_band_m"]]
                c, size, ang, pts = _rect(top[:, :2])
                hull = cv2.convexHull(pts)
                ra = max(size[0] * size[1], 1e-9)
                ratio = float(cv2.contourArea(hull) / 1e6 / ra)
                shape = "cyl" if ratio < p["stand_round_ratio"] else "box"
                yaw = 0.0 if shape == "cyl" else ((math.radians(ang) + math.pi / 4) % (math.pi / 2) - math.pi / 4)
                top_z = box_top_z + p["puck_h_m"]
                info.update(round_ratio=round(ratio, 3), top_wh_mm=[round(size[0] * 1000, 1), round(size[1] * 1000, 1)])
            else:
                ridge = P[rel >= hmax - p["ridge_band_m"]]
                c, rsize, rang, _ = _rect(ridge[:, :2])
                upper = P[rel >= p["lie_upper_m"]]
                _, usize, uang, _ = _rect(upper[:, :2]) if len(upper) >= 10 else (None, rsize, rang, None)
                yaw = _long_axis_yaw(usize, uang)
                # 능선 폭 = 축에 직각 방향 폭
                ax = np.array([math.cos(yaw), math.sin(yaw)])
                nrm = np.array([-ax[1], ax[0]])
                q = ridge[:, :2] - c
                wid = float(np.percentile(q @ nrm, 95) - np.percentile(q @ nrm, 5))
                # 축 방향 중심: 위쪽 점들의 축 방향 양 끝 평균 (능선이 짧게 보일 때 대비)
                qa = (upper[:, :2] - c) @ ax if len(upper) >= 10 else q @ ax
                c = c + ax * 0.5 * (np.percentile(qa, 2) + np.percentile(qa, 98))
                shape = "cyl" if wid < p["lie_ridge_w_m"] else "box"
                top_z = box_top_z + p["puck_w_m"]
                info.update(ridge_w_mm=round(wid * 1000, 1), upper_len_mm=round(max(usize) * 1000, 1))
            out.append(Puck(col, bool(lying), shape, np.asarray(c, float), float(top_z), float(yaw), hmax, area, vf, info))
    return out


def best_per_color(pucks: list[Puck]) -> dict[str, Puck]:
    """색마다 화소가 가장 많은(유효) 것 하나"""
    best: dict[str, Puck] = {}
    for pk in pucks:
        if np.isnan(pk.xy).any():
            continue
        if pk.color not in best or pk.n_px > best[pk.color].n_px:
            best[pk.color] = pk
    return best
