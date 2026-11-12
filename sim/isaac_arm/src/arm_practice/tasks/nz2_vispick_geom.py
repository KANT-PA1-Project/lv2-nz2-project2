"""NZ2 바닥 고정 팔 + 카메라 영상 집기 실험 — 기하 상수와 카메라 시야 계산 (순수 Python/torch, Isaac 불필요)

설치 위치: ~/manipulator-isaac61-lyrical-practice/src/arm_practice/tasks/nz2_vispick_geom.py
쓰는 곳: nz2_vispick_cfg.py (환경 설정), scripts/nz2_vispick_geom_check.py (사전 점검)

좌표: base_link 원점 = 차체 중심 바닥(= 환경 원점). +x 앞, +y 왼쪽, +z 위. 로봇은 USD root_joint 로 바닥에 고정.
치수 출처
  - 팔: nz2_arm_kin (r12_arm_train.urdf, 팔 = 차체 중심, arm_joint1 높이 0.275 m)
  - 상단 정면 카메라: r12_arm_train.urdf r07_camera1 (0.075, 0, 0.875), 컬러 광학 중심 +0.0126 앞 [가정: 왼쪽 IR 위치 대신 컬러 중심을 x 로만 반영]
  - 손목 카메라: nz2_pa24_v46 문서 — arm_link5 기준 (42.63, 0, 52.44) mm, 30° 하향 (CAD OBB 판독)
  - D435 컬러 640×480 초점거리 fx ≈ 615 px [가정, 실기 rs-enumerate-devices -c 로 확인]
  - 박스 205×205×70 mm (mission.pdf v5), 차체 충돌상자 앞 끝 x 0.22 m
"""
from __future__ import annotations

import math
import os

# ===== 카메라 공통 =====
REAL_W, REAL_H = 640, 480                 # 실기 해상도 (사용자 결정: 모든 카메라 640×480)
FX_REAL = 615.0                           # [가정] D435 컬러 640×480 초점거리 [px]
HFOV_DEG = math.degrees(2 * math.atan(REAL_W / 2 / FX_REAL))   # ≈ 54.9°
VFOV_DEG = math.degrees(2 * math.atan(REAL_H / 2 / FX_REAL))   # ≈ 42.6°
H_APERTURE_MM = 20.955                    # Isaac PinholeCameraCfg 기본 가로 조리개
FOCAL_MM = H_APERTURE_MM * FX_REAL / REAL_W                    # 시야각이 D435 와 같도록 맞춘 초점거리
V_APERTURE_MM = H_APERTURE_MM * REAL_H / REAL_W
DEPTH_MIN_M = 0.175                       # [가정] D435 깊이 최소 거리(640×480). 이보다 가까우면 실기 깊이 = 0
DEPTH_MAX_M = 1.5                         # 정규화 상한 (이보다 먼 값은 1.0)
CLIP_RANGE = (0.03, 5.0)

# ===== 정책 입력 영상 =====
OBS_W, OBS_H = 128, 96                    # 정책 입력 (4:3 유지). 실기: 640×480 → 같은 방식으로 축소·자르기
WRIST_RENDER = (128, 96)                  # 손목: 전체 화면을 1/5 로 축소한 것과 같음
TOP_RENDER = (256, 192)                   # 상단: 1/2.5 로 렌더 후 박스 주변 128×96 만 잘라 씀 (퍽 약 9 px)

# ===== 손목 카메라 (arm_link5 기준) =====
WRIST_CAM_POS = (0.04263, 0.0, 0.05244)
WRIST_CAM_TILT_DEG = 30.0                 # arm_link5 +X 에서 아래로

# ===== 상단 정면 카메라 (base_link 기준) =====
TOP_CAM_POS = (0.075 + 0.0126, 0.0, 0.875)
TOP_CAM_PITCH_REAL_DEG = 30.0             # 현재 실기 (URDF). 이 각에서는 0.67 m 보다 가까운 곳이 안 보임
TOP_CAM_PITCH_DEG_DEFAULT = 65.0          # 이 실험 기본값 (사용자 결정: 같은 위치, 하향각만 키움). 60° 는 박스 앞 끝이 화면 아래 끝 25 px

# ===== 작업 장면 =====
BOX = (0.205, 0.205, 0.070)               # 선반(박스) 깊이, 폭, 높이 (바닥 위)
CHASSIS_FRONT_X = float(os.environ.get("NZ2_CHASSIS_FRONT_X", "0.22"))                    # 팔 베이스 → 박스 앞면 기준 거리 [m] (+간격 10~40 mm). 이름은 이전 실험과 맞춤
PUCK_BOX = (0.03, 0.03, 0.06)
PUCK_CYL_R, PUCK_CYL_H = 0.015, 0.06
PUCK_LOCAL_MAX = 0.065                    # 박스 중심에서 퍽 중심 최대 |x|,|y| (누운 퍽 외접 반경 33.5 mm + 여유 → 박스 끝까지 37.5 mm)
PUCK_MIN_SEP = 0.075                      # 두 퍽 중심 최소 거리 (누운 퍽 둘이 겹치지 않는 값 67 mm + 여유)

# ===== 팔 베이스 (차체 없음, 팔만 바닥 z = 0 에 고정) =====
# 원점 = 팔 베이스(arm_joint1) 바로 아래 바닥. 박스·카메라 위치는 모두 팔 베이스 기준
ARM_X = 0.0
ARM_Z = float(os.environ.get("NZ2_ARM_Z", "0.0"))             # 0 = 바닥 (사용자 결정)

# ===== 시작 자세 (관측 자세) =====
# 손목 카메라가 박스 전체(퍽 9칸 윗면·바닥 포함)를 보고, 팔이 상단 카메라 시야를 가리지 않는 자세.
# 2026-11-10 격자 탐색(0.04 rad) 결과: 손목 카메라 (r 0.056, z 0.531) m, 광축 57.5° 하향, 박스까지 0.46~0.60 m,
# 상단 카메라 광선-팔 최소 거리 64 mm. 관절 순서 arm_joint1~5 [rad]
# (팔 바닥 z = 0 기준, 2026-11-10 v4) 어깨를 세우고(j2 ≈ 0) 팔꿈치를 들어 손목 카메라가 박스를 59° 로 내려다보는 준비 자세.
# 손목 카메라 (0.16, 0.386) m → 박스 평균 0.35 m, 손끝 높이 0.30 m. 상단 카메라 시야를 손목이 일부 가림(−35 mm, 허용)
# v3 의 (0, −1.188, 0.093, 1.202, 0) 은 팔을 뒤로 68° 눕혀 바닥 가까이 깔리는 자세라 폐기
LOOK_POSE = (0.0, -0.008, -1.447, 1.962, 0.0)
GRIPPER_OPEN = -0.4569                    # gripper_joint 상한 = 최대 벌림 (약 70 mm)


def _rot_y(v, th):
    c, s = math.cos(th), math.sin(th)
    x, y, z = v
    return (x * c + z * s, y, -x * s + z * c)


def project_world_cam(cam_pos, pitch_rad, p, w=REAL_W, h=REAL_H, fx=None):
    """'world' 관례 카메라(앞 +X, 위 +Z, y 축 기준 pitch 만큼 아래로)에서 점 p 의 화소 (u, v), 깊이 (광축 거리)
    yaw = 0 (정면) 만 다룬다."""
    fx = FX_REAL * w / REAL_W if fx is None else fx
    d = (p[0] - cam_pos[0], p[1] - cam_pos[1], p[2] - cam_pos[2])
    c, s = math.cos(pitch_rad), math.sin(pitch_rad)
    fwd = d[0] * c - d[2] * s
    up = d[0] * s + d[2] * c
    left = d[1]
    if fwd <= 1e-6:
        return float("nan"), float("nan"), fwd
    return w / 2 - fx * left / fwd, h / 2 - fx * up / fwd, fwd


def top_crop(pitch_deg: float = TOP_CAM_PITCH_DEG_DEFAULT, gap: float = 0.02):
    """상단 카메라 렌더(TOP_RENDER)에서 박스 중심을 가운데 두는 OBS 크기 자르기 창 (u0, v0) [렌더 화소]
    실기 640×480 에서는 (u0, v0, OBS_W, OBS_H) × (640 / TOP_RENDER[0]) 영역을 잘라 128×96 으로 줄이면 같다"""
    rw, rh = TOP_RENDER
    cx = CHASSIS_FRONT_X + gap + BOX[0] / 2
    u, v, _ = project_world_cam(TOP_CAM_POS, math.radians(pitch_deg), (cx, 0.0, BOX[2]), w=rw, h=rh)
    u0 = int(round(min(max(u - OBS_W / 2, 0), rw - OBS_W)))
    v0 = int(round(min(max(v - OBS_H / 2, 0), rh - OBS_H)))
    return u0, v0


def wrist_cam_pose_planar(q, arm=(None, None)):
    """q = (j1..j5) 에서 손목 카메라 위치 (r, z) 와 광축 하향각 [rad] (j1 = j5 = 0 가정, 팔 평면)
    nz2_arm_kin 없이 계산 (점검 스크립트가 torch 없이도 돌도록)"""
    x0 = ARM_X if arm[0] is None else arm[0]
    z0 = ARM_Z if arm[1] is None else arm[1]
    j2, j3, j4 = q[1], q[2], q[3]
    shoulder_z = z0 + 0.0595
    l2a, l2b, l3 = 0.024, 0.1854, 0.149
    ax, az = _rot_y((l2a, 0, l2b), j2)[0], _rot_y((l2a, 0, l2b), j2)[2]
    j3p = (x0 + ax, shoulder_z + az)
    a23 = j2 + j3
    wr = (j3p[0] + l3 * math.cos(a23), j3p[1] - l3 * math.sin(a23))
    p = a23 + j4
    j5 = (wr[0] + 0.06635 * math.cos(p), wr[1] - 0.06635 * math.sin(p))
    ox, oz = WRIST_CAM_POS[0], WRIST_CAM_POS[2]
    cam = (j5[0] + ox * math.cos(p) + oz * math.sin(p), j5[1] - ox * math.sin(p) + oz * math.cos(p))
    tcp = (wr[0] + (0.06635 + 0.105) * math.cos(p), wr[1] - (0.06635 + 0.105) * math.sin(p))
    tip = (tcp[0] + 0.020 * math.cos(p), tcp[1] - 0.020 * math.sin(p))
    return {"cam": cam, "cam_pitch": p + math.radians(WRIST_CAM_TILT_DEG), "J3": j3p, "W": wr, "J5": j5,
            "TIP": tip, "S": (x0, shoulder_z)}


def scene_points(gap: float = 0.02, puck_top: float = BOX[2] + PUCK_BOX[2]):
    """박스 윗면 모서리 4개 + 퍽 영역 모서리(윗면·바닥) — 시야 점검용"""
    x0 = CHASSIS_FRONT_X + gap
    cx = x0 + BOX[0] / 2
    pts = [(x, y, BOX[2]) for x in (x0, x0 + BOX[0]) for y in (-BOX[1] / 2, BOX[1] / 2)]
    m = PUCK_LOCAL_MAX
    for x in (cx - m, cx + m):
        for y in (-m, m):
            pts += [(x, y, puck_top), (x, y, BOX[2])]
    return pts


def _seg_dist(p, a, b):
    dx, dz = b[0] - a[0], b[1] - a[1]
    t = ((p[0] - a[0]) * dx + (p[1] - a[1]) * dz) / (dx * dx + dz * dz + 1e-12)
    t = min(max(t, 0.0), 1.0)
    return math.hypot(a[0] + t * dx - p[0], a[1] + t * dz - p[1])


def top_occlusion_margin(q, gap: float = 0.02, arm_radius: float = 0.035, arm=(None, None)):
    """상단 카메라 → 퍽 영역 광선과 팔(어깨~손가락 끝 꺾은선) 사이 최소 거리 − 팔 반경 [m]. 음수면 가림"""
    k = wrist_cam_pose_planar(q, arm)
    segs = [(k["S"], k["J3"]), (k["J3"], k["W"]), (k["W"], k["TIP"])]
    cam = (TOP_CAM_POS[0], TOP_CAM_POS[2])
    worst = 9.0
    for (x, _, z) in scene_points(gap):
        for i in range(31):
            a = i / 30
            p = (cam[0] + (x - cam[0]) * a, cam[1] + (z - cam[1]) * a)
            for s0, s1 in segs:
                worst = min(worst, _seg_dist(p, s0, s1))
    return worst - arm_radius


def wrist_view_ok(q, gaps=(0.01, 0.04), arm=(None, None), margin=0.04):
    """손목 카메라가 모든 간격에서 박스·퍽 영역 전체를 보고, 깊이가 최소 거리보다 먼지. 반환 (ok, 평균 깊이, v 중심)"""
    k = wrist_cam_pose_planar(q, arm)
    if k["TIP"][1] < BOX[2] + 0.10:
        return False, 0.0, 0.0
    ds, vs = [], []
    for g in gaps:
        for p in scene_points(g):
            u, v, d = project_world_cam((k["cam"][0], 0.0, k["cam"][1]), k["cam_pitch"], p)
            if not (d > DEPTH_MIN_M and REAL_W * margin <= u <= REAL_W * (1 - margin)
                    and REAL_H * margin <= v <= REAL_H * (1 - margin)):
                return False, 0.0, 0.0
            ds.append(d)
            vs.append(v)
    return True, sum(ds) / len(ds), sum(vs) / len(vs)


def find_look_pose(arm=(None, None), step=0.06, occl_min=0.02):
    """관측 자세 격자 탐색 (j1 = j5 = 0). 조건: 손목 카메라가 박스 전체를 봄 + 팔이 상단 시야를 occl_min 이상 비킴.
    점수: 박스까지 평균 깊이 0.40 m 에 가깝게, 화면 세로 중앙, LOOK_POSE 에 가깝게. 반환 (q, 정보) 또는 (None, 이유)"""
    lo = (-1.6581, -1.9373, -1.7279)
    hi = (1.9722, 1.4312, 2.042)
    best, info = None, "손목 시야 조건을 만족하는 자세 없음"
    rng = lambda a, b: [a + 0.05 + i * step for i in range(int((b - a - 0.1) / step) + 1)]
    for j2 in rng(lo[0], hi[0]):
        for j3 in rng(lo[1], hi[1]):
            for j4 in rng(lo[2], hi[2]):
                q = (0.0, j2, j3, j4, 0.0)
                ok, dm, vc = wrist_view_ok(q, arm=arm)
                if not ok:
                    continue
                occ = min(top_occlusion_margin(q, g, arm=arm) for g in (0.01, 0.04))
                if occ < occl_min:
                    continue
                score = abs(dm - 0.40) * 3 + abs(vc - REAL_H / 2) / 200 + 0.1 * math.dist(q, LOOK_POSE)
                if best is None or score < best[0]:
                    best = (score, q, {"mean_depth_m": round(dm, 3), "occl_mm": round(occ * 1000, 1)})
    if best is None:
        return None, info
    return tuple(round(v, 3) for v in best[1]), best[2]


def look_pose():
    """환경에서 쓸 관측 자세: NZ2_LOOK_POSE="j1,j2,j3,j4,j5" > 기본 팔 위치면 LOOK_POSE > 탐색"""
    raw = os.environ.get("NZ2_LOOK_POSE")
    if raw:
        q = tuple(float(v) for v in raw.split(","))
        if len(q) != 5:
            raise ValueError("NZ2_LOOK_POSE 는 관절 5개")
        return q
    if abs(ARM_Z) < 1e-6:
        return LOOK_POSE
    q, info = find_look_pose()
    if q is None:
        raise RuntimeError(f"[NZ2] 팔 높이 z {ARM_Z} 에서 관측 자세 없음: {info} → NZ2_LOOK_POSE 직접 지정")
    print(f"[NZ2] 팔 높이 z {ARM_Z} 관측 자세 자동 탐색: {q} {info}")
    return q


def report(top_pitch_deg: float = TOP_CAM_PITCH_DEG_DEFAULT, gaps=(0.01, 0.02, 0.04), q=None):
    """사전 점검 결과 (dict). ok = 모든 간격에서 두 카메라가 박스·퍽 영역 전체를 보고 팔이 상단 시야를 안 가림"""
    q = look_pose() if q is None else q
    out = {"arm_xz": (ARM_X, ARM_Z), "look_pose": q, "top_pitch_deg": top_pitch_deg, "hfov_deg": round(HFOV_DEG, 1), "vfov_deg": round(VFOV_DEG, 1),
           "rows": [], "ok": True}
    k = wrist_cam_pose_planar(q)
    rw, rh = TOP_RENDER
    u0, v0 = top_crop(top_pitch_deg)                 # 자르기 창은 기준 간격 20 mm 로 고정 (실기와 같게)
    for g in gaps:
        pts = scene_points(g)
        # 상단: 렌더 화면 + 자르기 창 안
        top = [project_world_cam(TOP_CAM_POS, math.radians(top_pitch_deg), p, w=rw, h=rh) for p in pts]
        top_in = all(u0 + 2 <= u <= u0 + OBS_W - 2 and v0 + 2 <= v <= v0 + OBS_H - 2 for u, v, _ in top)
        # 실기 30° 그대로일 때
        real = [project_world_cam(TOP_CAM_POS, math.radians(TOP_CAM_PITCH_REAL_DEG), p) for p in pts]
        real_in = sum(0 <= u <= REAL_W and 0 <= v <= REAL_H for u, v, _ in real)
        # 손목
        wr = [project_world_cam((k["cam"][0], 0.0, k["cam"][1]), k["cam_pitch"], p) for p in pts]
        m = 0.04
        wr_in = all(REAL_W * m <= u <= REAL_W * (1 - m) and REAL_H * m <= v <= REAL_H * (1 - m) for u, v, _ in wr)
        wr_depth = min(d for _, _, d in wr)
        occ = top_occlusion_margin(q, g)
        row = {"gap_mm": round(g * 1000), "top_crop_uv0": (u0, v0), "top_all_in_crop": top_in,
               "top_v_range_render": (round(min(v for _, v, _ in top)), round(max(v for _, v, _ in top))),
               "real30_points_visible": f"{real_in}/{len(pts)}",
               "wrist_all_in_view": wr_in, "wrist_min_depth_m": round(wr_depth, 3),
               "wrist_v_range": (round(min(v for _, v, _ in wr)), round(max(v for _, v, _ in wr))),
               "top_occlusion_margin_mm": round(occ * 1000, 1)}
        row["top_partly_occluded_by_arm"] = occ <= 0.0     # 기록만 (준비 자세에서 손목이 상단 시야 일부를 가리는 것은 허용)
        row_ok = top_in and wr_in and wr_depth > DEPTH_MIN_M
        row["ok"] = row_ok
        out["ok"] &= row_ok
        out["rows"].append(row)
    # 퍽 화소 크기 (정책 입력 기준)
    cx = CHASSIS_FRONT_X + 0.02 + BOX[0] / 2
    d_top = math.dist(TOP_CAM_POS, (cx, 0.0, 0.1))
    d_wr = math.dist((k["cam"][0], 0.0, k["cam"][1]), (cx, 0.0, 0.1))
    out["puck_px_top_obs"] = round(FX_REAL * rw / REAL_W * PUCK_BOX[0] / d_top, 1)
    out["puck_px_wrist_obs_at_look"] = round(FX_REAL * OBS_W / REAL_W * PUCK_BOX[0] / d_wr, 1)
    out["wrist_cam_rz"] = (round(k["cam"][0], 3), round(k["cam"][1], 3))
    out["wrist_cam_pitch_deg"] = round(math.degrees(k["cam_pitch"]), 1)
    return out
