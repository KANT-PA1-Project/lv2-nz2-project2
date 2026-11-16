-- nz2_swerve_nav/config/cartographer/nz2_real_2d.lua — 실물 2D 지도 작성 (6단계)
-- 입력: /scan (scan_merger, base_footprint, 4개 병합 720빔), /odom (swerve_controller_node, odom→base_footprint TF)
-- 출력: TF map→odom, /map (cartographer_occupancy_grid_node, 0.02 m)
-- ⚠ AMCL(nav2_stack) 과 동시 실행 금지 (둘 다 map→odom)
-- map 원점 = 지도 시작 자세 → 저장 후 transform_map.py 로 mission 좌표 변환 (시작 yaw π 면 180° 회전 포함)
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint",   -- IMU 안 씀 → 로봇 바닥 프레임
  published_frame = "odom",            -- odom→base_footprint 는 swerve_controller_node 가 냄
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 2          -- Jetson Orin Nano: 다른 노드 몫 남김

TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = 0.20          -- 병합 스캔은 로봇 중심 기준, 몸 제거 후 최소 ≈ 0.22 m
TRAJECTORY_BUILDER_2D.max_range = 6.0           -- 경기장 대각 약 6.9 m. 먼 거리 잡음 제외
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1   -- 이미 4개 병합된 360° 스캔
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.025
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60
-- 벽 3 mm·통로 550 mm → 실시간 상관 매칭으로 odom 오차(바퀴 미끄럼) 보정
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.10
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(15.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.)

POSE_GRAPH.optimize_every_n_nodes = 30
POSE_GRAPH.constraint_builder.min_score = 0.60
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5

return options
