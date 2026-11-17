#!/usr/bin/env python3
"""maze.usda(+geo.usd) 의 collision 프림 → Gazebo SDF 월드 (벽 = 3 mm 상자, 기둥 = 원통).
사용: python3 make_maze_sdf.py <maze.usda> <출력.sdf> [--json walls.json] [--no-boxes] [--pedestal]
(11/14) mission.pdf 수정: 중앙 받침대 삭제 → 기본 미포함. --pedestal 을 주면 예전처럼 넣음
좌표: mission.pdf 기준 (왼쪽 아래 기둥 중심 = 원점, +X 오른쪽, +Y 위, m). USD 좌표에서 그 기둥 중심만큼 평행이동
박스: mission.pdf v5 의 205×205×70 박스 8개 + 중앙 받침대 820×820×70 (중앙 위치는 [가정]) — 라이다 평면(47.5 mm)보다 높아 지도에 찍힘
시각 메시(98 MB)는 쓰지 않음 → gpu_lidar RTF 유지"""
import sys, json, math
import numpy as np
from pxr import Usd, UsdGeom

def components(nv, counts, idx):
    parent = list(range(nv))
    def f(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]; a = parent[a]
        return a
    k = 0
    for c in counts:
        vs = idx[k:k + c]; k += c
        for v in vs[1:]:
            ra, rb = f(vs[0]), f(v)
            if ra != rb: parent[ra] = rb
    groups = {}
    for v in range(nv): groups.setdefault(f(v), []).append(v)
    return list(groups.values())

def min_rect(xy):
    """2D 점들의 최소 면적 회전 사각형 → (cx, cy, length, width, yaw)"""
    best = None
    c = xy.mean(0)
    for a in np.linspace(0, math.pi, 1801)[:-1]:
        R = np.array([[math.cos(a), math.sin(a)], [-math.sin(a), math.cos(a)]])
        q = (xy - c) @ R.T
        mn, mx = q.min(0), q.max(0)
        area = (mx - mn).prod()
        if best is None or area < best[0] - 1e-12:
            best = (area, a, mn, mx)
    _, a, mn, mx = best
    mid = (mn + mx) / 2
    ctr = c + np.array([math.cos(a) * mid[0] - math.sin(a) * mid[1], math.sin(a) * mid[0] + math.cos(a) * mid[1]])
    L, W = mx - mn
    yaw = a
    if W > L: L, W, yaw = W, L, a + math.pi / 2
    yaw = (yaw + math.pi / 2) % math.pi - math.pi / 2
    return float(ctr[0]), float(ctr[1]), float(L), float(W), float(yaw)

# mission.pdf v5 (mm): 시작 박스 1~6, L1, L3 = 205×205×70
SMALL_BOXES_MM = [(1468, 950), (3670, 3152), (2936, 5354), (734, 5354), (2202, 5354), (3670, 3886),
                  (3670, 950), (3670, 5354)]
# 중앙 받침대 = 410 박스 2×2. L2 격자 중심 (2202,3700) 이 북쪽 두 박스 사이 이음매 위라고 [가정] → 받침대 중심 (2202, 3495)
CENTER_PEDESTAL_MM = (2202, 3495, 820)
BOX_H = 0.070

def main():
    src, dst = sys.argv[1], sys.argv[2]
    jpath = sys.argv[sys.argv.index('--json') + 1] if '--json' in sys.argv else None
    st = Usd.Stage.Open(src)
    xc = UsdGeom.XformCache()
    boxes, cyls = [], []
    for p in st.Traverse():
        s = p.GetPath().pathString
        if '/collision/' not in s: continue
        grp = s.split('/')[2]
        M = np.array(xc.GetLocalToWorldTransform(p))
        if p.IsA(UsdGeom.Mesh):
            m = UsdGeom.Mesh(p)
            pts = np.array(m.GetPointsAttr().Get(), dtype=float)
            P = (np.c_[pts, np.ones(len(pts))] @ M)[:, :3]
            for comp in components(len(P), m.GetFaceVertexCountsAttr().Get(), m.GetFaceVertexIndicesAttr().Get()):
                Q = P[comp]
                cx, cy, L, W, yaw = min_rect(Q[:, :2])
                z0, z1 = Q[:, 2].min(), Q[:, 2].max()
                boxes.append(dict(grp=grp, x=cx, y=cy, z=(z0 + z1) / 2, L=L, W=W, H=z1 - z0, yaw=yaw))
        elif p.GetTypeName() == 'Cylinder':
            r = p.GetAttribute('radius').Get(); h = p.GetAttribute('height').Get()
            sx = np.linalg.norm(M[0, :3]); sz = np.linalg.norm(M[2, :3])
            cyls.append(dict(grp=grp, x=M[3, 0], y=M[3, 1], z=M[3, 2], r=r * sx, h=h * sz))
        else:
            print('무시:', s, p.GetTypeName())
    # 원점 = 왼쪽 아래 기둥 중심 (mission.pdf)
    bl = min(cyls, key=lambda c: c['x'] + c['y'])
    ox, oy = bl['x'], bl['y']
    for e in boxes + cyls:
        e['x'] -= ox; e['y'] -= oy
    print(f'원점 이동: USD ({ox:.4f}, {oy:.4f}) → (0, 0)')
    if '--no-boxes' not in sys.argv:
        for (x, y) in SMALL_BOXES_MM:
            boxes.append(dict(grp='box205', x=x / 1000, y=y / 1000, z=BOX_H / 2, L=0.205, W=0.205, H=BOX_H, yaw=0.0))
        if '--pedestal' in sys.argv:
            cx, cy, a = CENTER_PEDESTAL_MM
            boxes.append(dict(grp='pedestal820', x=cx / 1000, y=cy / 1000, z=BOX_H / 2, L=a / 1000, W=a / 1000, H=BOX_H, yaw=0.0))
    # 같은 위치 기둥 중복 제거 (벽 묶음끼리 공유)
    uniq = []
    for c in cyls:
        if all(math.hypot(c['x'] - u['x'], c['y'] - u['y']) > 0.005 for u in uniq): uniq.append(c)
    print(f'벽 상자 {len(boxes)}개, 기둥 {len(uniq)}개 (중복 {len(cyls) - len(uniq)}개 제거)')
    allx = [b['x'] for b in boxes]; ally = [b['y'] for b in boxes]
    print(f'범위 x {min(allx):.3f}~{max(allx):.3f}, y {min(ally):.3f}~{max(ally):.3f}')
    walls = [b for b in boxes if b['grp'].startswith('wall')]
    print('벽 두께', sorted(set(round(b['W'] * 1000, 1) for b in walls)), 'mm, 높이', sorted(set(round(float(b['H']), 3) for b in walls)))
    print('박스', sum(1 for b in boxes if not b['grp'].startswith('wall')), '개')

    mat_w = '<material><ambient>0.92 0.92 0.92 1</ambient><diffuse>0.92 0.92 0.92 1</diffuse></material>'
    mat_b = '<material><ambient>0.85 0.65 0.35 1</ambient><diffuse>0.85 0.65 0.35 1</diffuse></material>'
    mat_p = '<material><ambient>0.3 0.3 0.35 1</ambient><diffuse>0.3 0.3 0.35 1</diffuse></material>'
    out = []
    for i, b in enumerate(boxes):
        g = f'<geometry><box><size>{b["L"]:.4f} {b["W"]:.4f} {b["H"]:.4f}</size></box></geometry>'
        pose = f'<pose>{b["x"]:.4f} {b["y"]:.4f} {b["z"]:.4f} 0 0 {b["yaw"]:.5f}</pose>'
        m = mat_w if b['grp'].startswith('wall') else mat_b
        out.append(f'      <collision name="{b["grp"]}_{i}_c">{pose}{g}</collision>\n      <visual name="{b["grp"]}_{i}_v">{pose}{g}{m}</visual>')
    for i, c in enumerate(uniq):
        g = f'<geometry><cylinder><radius>{c["r"]:.4f}</radius><length>{c["h"]:.4f}</length></cylinder></geometry>'
        pose = f'<pose>{c["x"]:.4f} {c["y"]:.4f} {c["z"]:.4f} 0 0 0</pose>'
        out.append(f'      <collision name="p{i}_c">{pose}{g}</collision>\n      <visual name="p{i}_v">{pose}{g}{mat_p}</visual>')
    body = '\n'.join(out)
    sdf = f'''<?xml version="1.0"?>
<!-- 경기장 월드: ~/git/lv2_assignment/nz2/maze/maze.usda 의 collision 프림에서 자동 생성 (tools/make_maze_sdf.py)
     좌표 = mission.pdf (왼쪽 아래 기둥 중심 원점, m). 출발 중심 (0.734, 0.950)
     벽 3 mm 상자 {len(walls)}개 (바닥 위 5~385 mm), 기둥 원통 {len(uniq)}개 (0~390 mm), 박스 {len(boxes) - len(walls)}개 (70 mm). 시각 메시 미사용(RTF) -->
<sdf version="1.9">
  <world name="maze_arena">
    <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <light type="directional" name="sun"><direction>-0.4 0.2 -1</direction></light>
    <model name="ground"><static>true</static><link name="ground">
      <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry></collision>
      <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane></geometry></visual>
    </link></model>
    <model name="maze"><static>true</static><link name="walls">
{body}
    </link></model>
  </world>
</sdf>
'''
    with open(dst, 'w', encoding='utf-8') as f: f.write(sdf)
    if jpath:
        with open(jpath, 'w') as f: json.dump(dict(boxes=boxes, posts=uniq), f, indent=1, default=float)
    print('저장:', dst)

if __name__ == '__main__':
    main()
