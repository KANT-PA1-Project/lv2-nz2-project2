#!/usr/bin/env python3
"""지도(map_saver yaml/pgm) 를 '지도 시작 자세 기준' → mission.pdf 좌표로 변환 (90° 배수 회전 + 평행이동)

Cartographer·slam_toolbox 의 map 원점 = 지도 작성을 시작한 로봇 자세(base_footprint).
실전 출발 자세는 (0.734, 0.950, yaw π) 이므로 shift_map_origin.py(평행이동만, yaw 0 전용)로는 안 됨 → 이 도구
  mission 좌표 p = T + R(yaw) · p_map     (T = 시작 x, y)

사용 (Jetson 또는 개발 PC):
  ros2 run nz2_swerve_nav transform_map.py ~/slam_maps/nz2_real_v1.yaml                 # 기본 0.734 0.950 3.14159
  ros2 run nz2_swerve_nav transform_map.py <yaml> <x> <y> <yaw_rad>
결과: <이름>_mission.yaml / <이름>_mission.pgm (원본은 그대로 둠). yaw 는 90° 배수만 (±0.6° 허용)
"""
import math
import os
import re
import sys

import numpy as np


def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    # 헤더: P5 <w> <h> <maxval> (주석 줄 허용)
    tokens, i = [], 0
    while len(tokens) < 4:
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while data[i:i + 1] not in (b'\n', b''):
                i += 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    if tokens[0] != b'P5':
        sys.exit(f'P5(이진 PGM) 만 지원: {path}')
    w, h, maxv = int(tokens[1]), int(tokens[2]), int(tokens[3])
    if maxv > 255:
        sys.exit('16비트 PGM 미지원')
    i += 1  # 헤더 뒤 공백 1개
    img = np.frombuffer(data[i:i + w * h], dtype=np.uint8).reshape(h, w)
    return img


def write_pgm(path, img):
    h, w = img.shape
    with open(path, 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode())
        f.write(np.ascontiguousarray(img, dtype=np.uint8).tobytes())


def transform(img, res, origin, tx, ty, yaw):
    """img: 위쪽 행 = y 최대 (map_saver 규약). 반환 (새 img, 새 origin [x, y])."""
    k = round(yaw / (math.pi / 2))
    if abs(yaw - k * math.pi / 2) > 0.01:
        sys.exit(f'yaw {yaw:.4f} rad 는 90° 배수가 아님 (지도 시작 방향을 0/90/180/270° 로 맞출 것)')
    k %= 4
    c, s = [(1, 0), (0, 1), (-1, 0), (0, -1)][k]  # cos, sin (정확값)
    h, w = img.shape
    grid = img[::-1, :]                 # grid[j, i]: j = y 방향(아래→위), i = x 방향
    ox, oy = origin
    # 원본 네 모서리 → mission 좌표 → 새 경계
    cx = np.array([ox, ox + w * res, ox, ox + w * res])
    cy = np.array([oy, oy, oy + h * res, oy + h * res])
    mx, my = tx + c * cx - s * cy, ty + s * cx + c * cy
    nox, noy = mx.min(), my.min()
    nw, nh = (w, h) if k % 2 == 0 else (h, w)
    # 새 칸 중심 → 원본 칸 (역변환). 90° 배수라 1:1 대응
    ii, jj = np.meshgrid(np.arange(nw), np.arange(nh))
    px, py = nox + (ii + 0.5) * res - tx, noy + (jj + 0.5) * res - ty
    qx, qy = c * px + s * py, -s * px + c * py      # R(-yaw)
    oi = np.floor((qx - ox) / res).astype(int)
    oj = np.floor((qy - oy) / res).astype(int)
    if oi.min() < 0 or oj.min() < 0 or oi.max() >= w or oj.max() >= h:
        sys.exit('내부 오류: 역변환 인덱스 범위 밖')
    new_grid = grid[oj, oi]
    return new_grid[::-1, :], (float(nox), float(noy))


def main():
    if len(sys.argv) not in (2, 5):
        print(__doc__)
        sys.exit(1)
    ypath = os.path.expanduser(sys.argv[1])
    tx, ty, yaw = (0.734, 0.950, math.pi) if len(sys.argv) == 2 else map(float, sys.argv[2:5])
    with open(ypath, encoding='utf-8') as f:
        text = f.read()
    m = re.search(r'origin:\s*\[\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*\]', text)
    r = re.search(r'resolution:\s*([-\d.eE+]+)', text)
    im = re.search(r'image:\s*(\S+)', text)
    if not (m and r and im):
        sys.exit('yaml 에서 image / resolution / origin 을 못 찾음')
    ox, oy, oyaw = (float(v) for v in m.groups())
    if abs(oyaw) > 1e-6:
        sys.exit(f'origin yaw = {oyaw} (0 이 아니면 미지원)')
    res = float(r.group(1))
    img_name = im.group(1).strip('"\'')
    img_path = img_name if os.path.isabs(img_name) else os.path.join(os.path.dirname(ypath), img_name)
    base = os.path.splitext(ypath)[0]
    out_yaml, out_pgm = base + '_mission.yaml', base + '_mission.pgm'
    if os.path.exists(out_yaml):
        sys.exit(f'중단: {out_yaml} 이 이미 있음 (지우거나 이름 변경 후 다시)')

    img = read_pgm(img_path)
    new, (nox, noy) = transform(img, res, (ox, oy), tx, ty, yaw)
    write_pgm(out_pgm, new)
    text = text[:m.start()] + f'origin: [{nox:.6f}, {noy:.6f}, 0.0]' + text[m.end():]
    text = re.sub(r'image:\s*\S+', f'image: {os.path.basename(out_pgm)}', text, count=1)
    with open(out_yaml, 'w', encoding='utf-8') as f:
        f.write(text)
    h, w = new.shape
    print(f'시작 자세 ({tx:.3f}, {ty:.3f}, {math.degrees(yaw):.1f}°) 기준 변환\n'
          f'  원본 {img.shape[1]}×{img.shape[0]} origin [{ox:.3f}, {oy:.3f}]\n'
          f'  결과 {w}×{h} origin [{nox:.3f}, {noy:.3f}]  범위 x {nox:.2f}~{nox + w * res:.2f}, '
          f'y {noy:.2f}~{noy + h * res:.2f} m\n  → {out_yaml}')


if __name__ == '__main__':
    main()
