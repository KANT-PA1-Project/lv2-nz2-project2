"""NZ2 경기장(mission.pdf v5 '주사위 기억 로봇' 맵) 데이터 + Isaac Lab 장면 부품 생성

설치 위치: ~/manipulator-isaac61-lyrical-practice/src/arm_practice/tasks/nz2_arena.py
좌표: 맵 = 왼쪽 아래 기둥 중심 (0,0) [mm], +X 오른쪽, +Y 위. 바닥 Z=0.
출처
  - 벽 체인·외곽 피치: claude/nz2_sim_map.md (mission.pdf p.2 픽셀 측정 → 734/394 mm 격자 스냅, 설치 수량 일치 검증)
  - 장소: L1=(3670,950), L2=(2202,3700), L3=(3670,5354)  [mission.pdf p.2 본문]
  - 시작 박스 6곳: p.2 그림 판독 → 734 격자에 스냅 [그림, 실물 확인 필요]
  - 박스: 작은 205×205×70 (시작 6 + L1·L3), 큰 410×410×70 ×4 (중앙 820 받침대, 중심 (2202,3495))
  - 퍽 3×3 격자 ±60 mm (p.4), 시작 박스 퍽 2개 X±40 mm (p.2 그림의 점 2개)
  - 벽판 높이 380 mm (maze USD), 두께 10 mm [가정], 기둥 30×30×390 [가정: R3 연결기둥 근사]

장면에서는 '로봇 기준 좌표'로 바꿔 넣는다. 로봇 base_link 원점 = 환경 원점, 로봇 +X = 박스 쪽.
  로봇 중심 = 박스 중심 − (박스 반깊이 + 간격 + 차체 앞 끝 0.22 m)·접근 방향
"""
from __future__ import annotations

import math
import os

# ===== 치수 [m] =====
WALL_H, WALL_T = 0.380, 0.010                 # [가정] 두께
POST_W, POST_H = 0.030, 0.390
SMALL_BOX = (0.205, 0.205, 0.070)
BIG_BOX = (0.410, 0.410, 0.070)
CHASSIS_FRONT_X = 0.22                         # URDF 차체 충돌상자 앞 끝
NOMINAL_GAP = 0.02                             # 벽 배치 기준 간격 (박스·퍽은 환경마다 간격을 바꿔도 벽은 이 값 기준)
FIELD = (4.404, 6.304)                         # 외곽 기둥 중심 간

# ===== 벽 =====
_X = [0.367, 1.101, 1.835, 2.569, 3.303, 4.037]
WALL_CHAINS = [
    [(1.835, 4.987), (2.569, 4.987), (2.569, 5.721)],
    [(1.101, 4.987), (1.101, 5.721)],
    [(3.303, 4.987), (3.303, 5.721)],
    [(0.367, 4.253), (1.101, 4.253), (1.101, 3.519)],
    [(3.303, 4.253), (4.037, 4.253)],
    [(4.037, 3.519), (3.303, 3.519), (3.303, 2.051), (2.569, 2.051)],
    [(1.101, 2.785), (1.101, 2.051), (1.835, 2.051), (1.835, 1.317), (1.101, 1.317), (1.101, 0.583)],
    [(2.569, 1.317), (4.037, 1.317)],
]
# 외곽: X 피치 0.734 (긴 벽판), Y 피치 0.394 (짧은 벽판)
OUTER_X = [round(0.734 * i, 3) for i in range(7)]          # 0 … 4.404
OUTER_Y = [round(0.394 * i, 3) for i in range(17)]         # 0 … 6.304 (16 × 394 = 6304)

# ===== 박스 =====
SMALL_BOXES = {  # 이름: 중심 (x, y) [m]
    "S1": (1.468, 0.950), "S2": (3.670, 3.152), "S3": (2.936, 5.354),
    "S4": (0.734, 5.354), "S5": (2.202, 5.354), "S6": (3.670, 3.886),
    "L1": (3.670, 0.950), "L3": (3.670, 5.354),
}
BIG_CENTER = (2.202, 3.495)
BIG_BOXES = {f"B{i}": (BIG_CENTER[0] + dx * 0.205, BIG_CENTER[1] + dy * 0.205)
             for i, (dx, dy) in enumerate(((-1, -1), (1, -1), (-1, 1), (1, 1)))}

# ===== 실험 장소 =====
# heading = 로봇 +X 가 향하는 맵 방향 [deg]. 접근 방향은 주변 벽과 로봇 400×400 여유를 보고 고름 [가정]
#   L1 서쪽 접근(그림 접근점 3228,950), L3 남쪽 접근(그림 접근점 3669,4914)
#   S1·S2·S6 동쪽 접근(퍽 2개가 로봇 앞뒤 줄 → 먼 퍽이 최대 도달 시험), S5 서쪽(남쪽은 y 4987 벽), S3·S4 남쪽
GRID9 = [(dx, dy) for dy in (-0.06, 0.0, 0.06) for dx in (-0.06, 0.0, 0.06)]   # 칸 0~8 = 왼아래→오른위 (map +Y 위)
PAIR = [(-0.04, 0.0), (0.04, 0.0)]
SITES = {
    "L1": dict(box="L1", heading=0.0, cells=GRID9),
    "L3": dict(box="L3", heading=90.0, cells=GRID9),
    "S1": dict(box="S1", heading=180.0, cells=PAIR),
    "S2": dict(box="S2", heading=180.0, cells=PAIR),
    "S3": dict(box="S3", heading=90.0, cells=PAIR),
    "S4": dict(box="S4", heading=90.0, cells=PAIR),
    "S5": dict(box="S5", heading=0.0, cells=PAIR),
    "S6": dict(box="S6", heading=180.0, cells=PAIR),
    # LINE: 벽 없는 평지 + 깊은 받침(205 깊이 ×2), 퍽을 정면 0.30~0.50 m 에 10 mm 간격 → 순수 최대 도달 곡선
    "LINE": dict(box=None, heading=0.0, cells=None),
}
LINE_R = [round(0.30 + 0.01 * i, 3) for i in range(21)]     # 팔 중심(차체 중심)에서 퍽 중심까지 [m]
LINE_BOX = (0.41, 0.205, 0.070)                              # 깊이 410 (박스 2개 이어 붙인 모양)


def site_names() -> list[str]:
    return list(SITES)


def robot_pose(site: str, gap: float = NOMINAL_GAP):
    """맵 좌표 로봇 중심 (x, y) [m], heading [rad]"""
    s = SITES[site]
    psi = math.radians(s["heading"])
    if s["box"] is None:
        return (0.0, 0.0), psi
    bx, by = SMALL_BOXES[s["box"]]
    d = SMALL_BOX[0] / 2 + gap + CHASSIS_FRONT_X
    return (bx - d * math.cos(psi), by - d * math.sin(psi)), psi


def to_robot(p, site: str):
    """맵 점 (x, y) → 로봇 기준 (x, y) (기준 간격 NOMINAL_GAP)"""
    (rx, ry), psi = robot_pose(site)
    dx, dy = p[0] - rx, p[1] - ry
    c, s = math.cos(psi), math.sin(psi)
    return (c * dx + s * dy, -s * dx + c * dy)


def cells_robot(site: str):
    """퍽 칸의 '박스 중심 기준' 오프셋을 로봇 기준 (앞, 왼) 으로. 반환 [(dx_fwd, dy_left), ...]"""
    s = SITES[site]
    psi = math.radians(s["heading"])
    c, sn = math.cos(psi), math.sin(psi)
    return [(c * dx + sn * dy, -sn * dx + c * dy) for dx, dy in s["cells"]]


# ---------------------------------------------------------------------------
# 경기장 요소 목록 (맵 좌표) → 로봇 기준 직육면체 목록
# ---------------------------------------------------------------------------
def _segments():
    segs = []
    for chain in WALL_CHAINS:
        segs += list(zip(chain[:-1], chain[1:]))
    for x0, x1 in zip(OUTER_X[:-1], OUTER_X[1:]):
        segs += [((x0, 0.0), (x1, 0.0)), ((x0, FIELD[1]), (x1, FIELD[1]))]
    for y0, y1 in zip(OUTER_Y[:-1], OUTER_Y[1:]):
        segs += [((0.0, y0), (0.0, y1)), ((FIELD[0], y0), (FIELD[0], y1))]
    return segs


def _posts(segs):
    pts = {}
    for a, b in segs:
        pts[(round(a[0], 3), round(a[1], 3))] = a
        pts[(round(b[0], 3), round(b[1], 3))] = b
    return list(pts.values())


def _dist_point_seg(p, a, b):
    ax, ay, bx, by = *a, *b
    vx, vy = bx - ax, by - ay
    t = max(0.0, min(1.0, ((p[0] - ax) * vx + (p[1] - ay) * vy) / max(vx * vx + vy * vy, 1e-12)))
    return math.hypot(p[0] - (ax + t * vx), p[1] - (ay + t * vy))


def arena_cuboids(site: str, radius: float | None):
    """로봇 기준 직육면체 목록 [(이름, (cx, cy, cz), (sx, sy, sz), yaw_rad, 색, 종류)]
    radius: 로봇 중심에서 이 거리 안의 요소만 (None = 전체 맵). 실험 박스(site 의 박스)는 제외 — 따로 움직이는 물체로 넣음"""
    out = []
    if SITES[site]["box"] is None:
        return out
    (rx, ry), psi = robot_pose(site)
    keep = (lambda d: True) if radius is None else (lambda d: d <= radius)
    segs = _segments()
    for i, (a, b) in enumerate(segs):
        if not keep(_dist_point_seg((rx, ry), a, b)):
            continue
        ra, rb = to_robot(a, site), to_robot(b, site)
        cx, cy = (ra[0] + rb[0]) / 2, (ra[1] + rb[1]) / 2
        length = math.hypot(rb[0] - ra[0], rb[1] - ra[1]) - POST_W     # 기둥 사이 벽판
        yaw = math.atan2(rb[1] - ra[1], rb[0] - ra[0])
        out.append((f"wall_{i:03d}", (cx, cy, WALL_H / 2), (length, WALL_T, WALL_H), yaw, (0.85, 0.85, 0.88), "wall"))
    for i, p in enumerate(_posts(segs)):
        if not keep(math.hypot(p[0] - rx, p[1] - ry)):
            continue
        rp = to_robot(p, site)
        out.append((f"post_{i:03d}", (rp[0], rp[1], POST_H / 2), (POST_W, POST_W, POST_H), 0.0, (0.2, 0.25, 0.3), "post"))
    boxes = [(n, c, SMALL_BOX) for n, c in SMALL_BOXES.items() if n != SITES[site]["box"]]
    boxes += [(n, c, BIG_BOX) for n, c in BIG_BOXES.items()]
    for n, c, size in boxes:
        half = max(size[0], size[1]) / 2
        if not keep(max(0.0, math.hypot(c[0] - rx, c[1] - ry) - half)):
            continue
        rc = to_robot(c, site)
        out.append((f"box_{n}", (rc[0], rc[1], size[2] / 2), size, -psi, (0.75, 0.6, 0.4), "box"))
    return out


def add_arena_to_scene(scene_cfg, site: str, radius: float | None):
    """scene_cfg 에 정적 충돌 직육면체(벽·기둥·다른 박스)를 환경마다 추가. 반환: 추가한 prim 이름 목록"""
    import isaaclab.sim as sim_utils                       # 설정 단계에서만 쓰는 import (pxr 무관)
    from isaaclab.assets import AssetBaseCfg
    names = []
    for name, (cx, cy, cz), size, yaw, color, _ in arena_cuboids(site, radius):
        q = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))   # (x, y, z, w) — Isaac Lab 3.0 순서
        cfg = AssetBaseCfg(
            prim_path="{ENV_REGEX_NS}/Arena/" + name,
            spawn=sim_utils.CuboidCfg(
                size=tuple(size),
                collision_props=sim_utils.CollisionPropertiesCfg(),
                physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=0.35, dynamic_friction=0.30),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=color)),
            init_state=AssetBaseCfg.InitialStateCfg(pos=(cx, cy, cz), rot=q))
        setattr(scene_cfg, "arena_" + name, cfg)
        names.append(name)
    return names


def _rect_dist(p, c, size, yaw):
    """점 p 와 회전 직사각형(중심 c, 크기 size[:2], yaw) 사이 거리"""
    dx, dy = p[0] - c[0], p[1] - c[1]
    co, si = math.cos(-yaw), math.sin(-yaw)
    lx, ly = co * dx - si * dy, si * dx + co * dy
    ex, ey = max(abs(lx) - size[0] / 2, 0.0), max(abs(ly) - size[1] / 2, 0.0)
    return math.hypot(ex, ey)


def park_points(site: str, full_map: bool = False, radius_m: float = 1.25, clearance_m: float = 0.12):
    """쓰지 않는 퍽을 둘 바닥 위치 2개 (로봇 기준 x, y). 벽·기둥·박스에서 clearance_m 이상 떨어진 곳.
    (로봇 바로 뒤 고정 위치는 장소에 따라 벽 속에 들어가 퍽이 튕겨 나감 → 장소별로 계산)"""
    cubes = arena_cuboids(site, None if full_map else 1.6)
    out = []
    for k in range(48):
        ang = math.pi + (k // 2 + 1) * (math.pi / 24) * (1 if k % 2 == 0 else -1)   # 뒤쪽부터 좌우로 넓혀 감
        for rad in (radius_m, radius_m + 0.25, radius_m - 0.25):
            p = (rad * math.cos(ang), rad * math.sin(ang))
            if all(_rect_dist(p, c[1], c[2], c[3]) >= clearance_m for c in cubes):
                if all(math.hypot(p[0] - o[0], p[1] - o[1]) > 0.15 for o in out):
                    out.append(p)
                break
        if len(out) == 2:
            return out
    raise RuntimeError(f"{site}: 퍽 보관 위치를 못 찾음")


def env_site() -> str:
    site = os.environ.get("NZ2_SITE", "L1").upper()
    if site not in SITES:
        raise ValueError(f"NZ2_SITE={site} 는 없는 장소. 가능: {site_names()}")
    return site


if __name__ == "__main__":       # 개발 PC/클라우드에서 데이터 점검: python3 nz2_arena.py
    for s in SITES:
        (x, y), psi = robot_pose(s)
        n = len(arena_cuboids(s, 1.0))
        print(f"{s:5s} 로봇 중심 ({x:.3f},{y:.3f}) heading {math.degrees(psi):5.1f}°  반경 1 m 요소 {n}개")
    print("전체 맵 요소:", len(arena_cuboids("L1", None)))
    for s in SITES:
        print(s, "퍽 보관:", [tuple(round(v, 3) for v in p) for p in park_points(s)],
              "전체맵:", [tuple(round(v, 3) for v in p) for p in park_points(s, True)])
