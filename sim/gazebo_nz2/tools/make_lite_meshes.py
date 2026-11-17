#!/usr/bin/env python3
"""시각 STL 감량 (Gazebo gpu_lidar·GUI 부하 감소). 정점 격자 병합(vertex clustering) 방식:
정점을 grid [m] 격자에 스냅 → 같은 칸 정점 합치기 → 찌그러진 삼각형 제거.
외곽 오차 ≤ grid/2·√3, 나사 같은 작은 부품도 사라지지 않고 덩어리로 남음
사용: python3 make_lite_meshes.py <meshes 폴더> <출력 폴더> [grid_mm=2.0]
의존: pip install trimesh"""
import os, sys, glob
import numpy as np
import trimesh

src, dst = sys.argv[1], sys.argv[2]
grid = (float(sys.argv[3]) if len(sys.argv) > 3 else 2.0) / 1000.0
os.makedirs(dst, exist_ok=True)
t0 = t1 = 0
for f in sorted(glob.glob(os.path.join(src, '*.stl'))):
    m = trimesh.load(f, force='mesh')
    n0 = len(m.faces)
    q = np.round(m.vertices / grid).astype(np.int64)
    uq, inv = np.unique(q, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    # 칸 대표점 = 칸 안 원래 정점 평균 (격자점보다 모양이 덜 깎임)
    cnt = np.bincount(inv, minlength=len(uq)).astype(float)
    v = np.stack([np.bincount(inv, weights=m.vertices[:, k], minlength=len(uq)) / cnt for k in range(3)], 1)
    fc = inv[m.faces]
    ok = (fc[:, 0] != fc[:, 1]) & (fc[:, 1] != fc[:, 2]) & (fc[:, 0] != fc[:, 2])
    fc = np.unique(np.sort(fc[ok], 1), axis=0, return_index=True)[1]
    fc = inv[m.faces][ok][np.sort(fc)]
    m2 = trimesh.Trimesh(v, fc, process=False)
    m2.remove_unreferenced_vertices()
    m2.export(os.path.join(dst, os.path.basename(f)))
    ext = np.abs(m2.bounds - m.bounds).max() * 1000
    print(f'{os.path.basename(f):28s} {n0:8d} → {len(m2.faces):6d} 면, 외곽 차 {ext:.2f} mm')
    t0 += n0; t1 += len(m2.faces)
print(f'합계 {t0} → {t1} 면 ({t1 / t0 * 100:.1f} %)')
