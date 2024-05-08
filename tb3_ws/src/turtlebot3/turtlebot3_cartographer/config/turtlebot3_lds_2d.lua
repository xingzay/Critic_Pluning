include "map_builder.lua"
include "trajectory_builder.lua"
-- turtlebot3 --

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
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
-- 前端参数--
TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 20
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true 
-- TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)
-- 后端参数--
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.50 --0.7

-- TRAJECTORY_BUILDER.pure_localization_trimmer = {
--   max_submaps_to_keep = 3,
-- }

POSE_GRAPH.optimize_every_n_nodes = 1  -- 10 几乎不更新
POSE_GRAPH.constraint_builder.sampling_ratio = 0.2 --默认 0.3 约束采样，太多则速度慢，太少则会导致约束丢失，闭环效果不好
POSE_GRAPH.global_sampling_ratio = 0.001  -- 默认0.003

return options

--推料--
-- options = {
--   map_builder = MAP_BUILDER,
--   trajectory_builder = TRAJECTORY_BUILDER,
--   map_frame = "map",
--   tracking_frame = "imu_link",
--   published_frame = "odom",
--   odom_frame = "odom",
--   provide_odom_frame = false,
--   publish_frame_projected_to_2d = true,
--   use_odometry = true,
--   use_nav_sat = false,
--   use_landmarks = false,
--   num_laser_scans = 1,
--   num_multi_echo_laser_scans = 0,
--   num_subdivisions_per_laser_scan = 1,
--   num_point_clouds = 0,
--   lookup_transform_timeout_sec = 0.2,  --0.2
--   submap_publish_period_sec = 0.3,
--   pose_publish_period_sec = 5e-3,
--   trajectory_publish_period_sec = 30e-3,
--   rangefinder_sampling_ratio = 1.,
--   odometry_sampling_ratio = 1.,
--   fixed_frame_pose_sampling_ratio = 1.,
--   imu_sampling_ratio = 1.,
--   landmarks_sampling_ratio = 1.,
-- }

-- MAP_BUILDER.use_trajectory_builder_2d = true

-- TRAJECTORY_BUILDER_2D.min_range = 0.1
-- TRAJECTORY_BUILDER_2D.max_range = 30
-- TRAJECTORY_BUILDER_2D.missing_data_ray_length = 10 --当传感器数据超出有效范围最大值时，按此值来处理。默认配置为8m，小于最大距离30m。
-- TRAJECTORY_BUILDER_2D.use_imu_data = false  -- 如果true 时 出现激光不显示，不知道原因
-- TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
-- -- TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

-- POSE_GRAPH.constraint_builder.min_score = 0.65 --0.65
-- POSE_GRAPH.constraint_builder.global_localization_min_score = 0.50 --0.7

-- TRAJECTORY_BUILDER.pure_localization_trimmer = {
--   max_submaps_to_keep = 3,
-- }

-- POSE_GRAPH.optimize_every_n_nodes = 1  -- 10 几乎不更新
-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.2 --默认 0.3 约束采样，太多则速度慢，太少则会导致约束丢失，闭环效果不好
-- POSE_GRAPH.global_sampling_ratio = 0.001  -- 默认0.003

-- -- POSE_GRAPH.global_constraint_search_after_n_seconds = 10

-- return options

