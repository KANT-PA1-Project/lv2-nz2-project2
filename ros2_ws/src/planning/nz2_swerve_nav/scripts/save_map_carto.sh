#!/usr/bin/env bash
# nz2_swerve_nav/scripts/save_map_carto.sh — Cartographer 실물 지도 저장 (Jetson)
# 사용: ros2 run nz2_swerve_nav save_map_carto.sh <이름> [시작x=0.734] [시작y=0.950] [시작yaw=3.14159]
# 순서: 점검 → finish_trajectory(최종 최적화) → map_saver → pbstream 저장 → transform_map(mission 좌표)
# 결과: ~/slam_maps/<이름>.yaml/.pgm (시작 자세 기준 원본), <이름>_mission.yaml/.pgm (Nav2 용), <이름>.pbstream
set -u
NAME="${1:?사용: save_map_carto.sh <이름> [x y yaw]}"
SX="${2:-0.734}"; SY="${3:-0.950}"; SYAW="${4:-3.14159}"
DIR="$HOME/slam_maps"; mkdir -p "$DIR"; OUT="$DIR/$NAME"
[ -e "$OUT.yaml" ] && { echo "✗ $OUT.yaml 이미 있음 → 다른 이름"; exit 1; }

echo "[1/5] Cartographer 확인"
ros2 node list 2>/dev/null | grep -qx '/cartographer_node' || { echo "  ✗ /cartographer_node 없음 (mode:=mapping 으로 실행했는지)"; exit 1; }
timeout 10 ros2 topic echo /map --once --field info >/dev/null 2>&1 || { echo "  ✗ /map 안 나옴 (occupancy_grid_node, /scan, TF odom→base_footprint 확인)"; exit 1; }
echo "  ✓ /map 수신"

echo "[2/5] finish_trajectory (마지막 루프 최적화, 수 초)"
timeout 60 ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}" \
  || echo "  (finish_trajectory 실패/시간초과 — 현재 지도로 계속)"
sleep 3   # occupancy_grid_node 가 최종 서브맵으로 /map 다시 발행할 시간 (publish_period 1 s)

echo "[3/5] map_saver → $OUT.yaml"
ros2 run nav2_map_server map_saver_cli -f "$OUT" --ros-args -p use_sim_time:=false -p save_map_timeout:=10.0 || exit 1
[ -f "$OUT.yaml" ] || { echo "  ✗ $OUT.yaml 생성 안 됨"; exit 1; }

echo "[4/5] pbstream 저장 (선택: 나중에 재검토용)"
timeout 30 ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '$OUT.pbstream', include_unfinished_submaps: true}" >/dev/null || echo "  (write_state 실패 — 지도 yaml/pgm 은 정상)"

echo "[5/5] mission 좌표 변환 (시작 $SX, $SY, yaw $SYAW)"
ros2 run nz2_swerve_nav transform_map.py "$OUT.yaml" "$SX" "$SY" "$SYAW" || exit 1
echo "완료 → Nav2: ros2 launch nz2_swerve_nav real_nav.launch.py mode:=nav map:=${OUT}_mission.yaml ..."
echo "⚠ Cartographer(mapping) launch 를 먼저 Ctrl-C 로 끈 뒤 nav 실행 (map→odom 충돌)"
