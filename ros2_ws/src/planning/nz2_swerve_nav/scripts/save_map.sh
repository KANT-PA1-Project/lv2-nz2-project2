#!/usr/bin/env bash
# nz2_swerve_nav/scripts/save_map.sh — slam_toolbox 지도 저장 (점검 → map_saver → 원점 이동 → 포즈 그래프 저장)
# 사용: ros2 run nz2_swerve_nav save_map.sh [이름=nz2_maze_sim] [shift: 1=mission 좌표로 원점 이동(기본) / 0=안 함]
# 결과: ~/slam_maps/<이름>.yaml .pgm (+ .yaml.bak), ~/slam_maps/<이름>_graph.posegraph .data
set -u
NAME="${1:-nz2_maze_sim}"
SHIFT="${2:-1}"
DIR="$HOME/slam_maps"
mkdir -p "$DIR"
OUT="$DIR/$NAME"

echo "[1/4] slam_toolbox 상태 확인"
if ! ros2 node list 2>/dev/null | grep -qx '/slam_toolbox'; then
  echo "  ✗ /slam_toolbox 노드 없음 → 터미널 1 을 slam:=true 로 실행했는지, 로그에 slam_toolbox 오류가 있는지 확인"
  exit 1
fi
STATE=$(ros2 lifecycle get /slam_toolbox 2>/dev/null | awk '{print $1}')
echo "  lifecycle: ${STATE:-알 수 없음}"
if [ "${STATE:-}" = "unconfigured" ]; then
  ros2 lifecycle set /slam_toolbox configure && STATE=inactive
fi
if [ "${STATE:-}" = "inactive" ]; then
  ros2 lifecycle set /slam_toolbox activate
fi

echo "[2/4] /map 수신 확인 (최대 15 s)"
if ! timeout 15 ros2 topic echo /map --once --field info >/dev/null 2>&1; then
  echo "  ✗ /map 이 안 나옴 → ros2 topic hz /scan (≈10 Hz), ros2 run tf2_ros tf2_echo odom base_footprint 확인"
  exit 1
fi
echo "  ✓ /map 수신"

echo "[3/4] map_saver → $OUT.yaml"
if [ -e "$OUT.yaml" ]; then
  echo "  ✗ $OUT.yaml 이 이미 있음 → 다른 이름을 주거나 기존 파일을 옮긴 뒤 다시 실행"
  exit 1
fi
ros2 run nav2_map_server map_saver_cli -f "$OUT" --ros-args -p use_sim_time:=true -p save_map_timeout:=10.0 || exit 1
[ -f "$OUT.yaml" ] || { echo "  ✗ $OUT.yaml 생성 안 됨"; exit 1; }
if [ "$SHIFT" = "1" ]; then
  ros2 run nz2_swerve_nav shift_map_origin.py "$OUT.yaml" || exit 1
fi

echo "[4/4] 포즈 그래프 저장 → ${OUT}_graph (선택, 실패해도 지도는 저장됨)"
timeout 20 ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '${OUT}_graph'}" \
  || echo "  (serialize_map 실패 — 지도 yaml/pgm 은 정상)"
echo "완료: $OUT.yaml"
