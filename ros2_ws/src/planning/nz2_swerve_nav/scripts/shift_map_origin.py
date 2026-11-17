#!/usr/bin/env python3
"""map_saver 가 만든 지도 yaml 의 origin 을 평행이동 → map 프레임 = mission.pdf 좌표(왼쪽 아래 기둥 중심 원점, m)

전제: 지도 작성을 출발 중심 (0.734, 0.950), yaw 0(+X 를 봄)에서 시작 (slam_toolbox map 원점 = 시작 자세)
사용: ros2 run nz2_swerve_nav shift_map_origin.py ~/slam_maps/nz2_maze_sim.yaml            # 기본 0.734 0.950
      ros2 run nz2_swerve_nav shift_map_origin.py <yaml> <dx> <dy>
원본은 <이름>.yaml.bak 로 남김. 두 번 실행하면 두 번 이동하므로 주의(.bak 가 있으면 중단)"""
import os
import re
import shutil
import sys


def main():
    if len(sys.argv) not in (2, 4):
        print(__doc__)
        sys.exit(1)
    path = os.path.expanduser(sys.argv[1])
    dx, dy = (float(sys.argv[2]), float(sys.argv[3])) if len(sys.argv) == 4 else (0.734, 0.950)
    bak = path + '.bak'
    if os.path.exists(bak):
        sys.exit(f'중단: {bak} 가 이미 있음 (이미 이동한 지도로 보임)')
    with open(path, encoding='utf-8') as f:
        text = f.read()
    m = re.search(r'origin:\s*\[\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)\s*\]', text)
    if not m:
        sys.exit('origin: [x, y, yaw] 줄을 찾지 못함')
    x, y, yaw = (float(v) for v in m.groups())
    if abs(yaw) > 1e-6:
        sys.exit(f'중단: origin yaw = {yaw} (0 이 아니면 단순 평행이동 불가)')
    new = f'origin: [{x + dx:.6f}, {y + dy:.6f}, 0.0]'
    shutil.copy2(path, bak)
    with open(path, 'w', encoding='utf-8') as f:
        f.write(text[:m.start()] + new + text[m.end():])
    print(f'origin [{x:.3f}, {y:.3f}] → [{x + dx:.3f}, {y + dy:.3f}]  (백업 {bak})')


if __name__ == '__main__':
    main()
