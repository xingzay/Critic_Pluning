include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  -- 一般设置为发布频率最高的传感器的frame_id
  tracking_frame = "laser_link",  --laser_link
  -- base_link改为odom,发布map到odom之间的位姿态
  published_frame = "base_link",  -- base_link
  odom_frame = "odom",
  -- true改为false，不用提供里程计数据
  provide_odom_frame = false,
  -- false改为true，仅发布2D位资
  publish_frame_projected_to_2d = true,
  -- false改为true，使用里程计数据（IMU融合需要为true）
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  -- 0改为1,使用一个雷达
  num_laser_scans = 1,
  -- 1改为0，不使用多波雷达
  num_multi_echo_laser_scans = 0,
  -- 10改为1，1/1=1等于不分割
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


-- false改为true，启动2D SLAM
MAP_BUILDER.use_trajectory_builder_2d = true

-- TRAJECTORY_BUILDER_2D 参数说明参见 https://blog.csdn.net/c417469898/article/details/117790918
-- S2M1-R2 最小为0.05
TRAJECTORY_BUILDER_2D.min_range = 0.10
-- 30改成10,限制在雷达最大扫描范围内，越小一般越精确些
TRAJECTORY_BUILDER_2D.max_range = 20
-- 超出传感器数据有效范围（0.05 ～ 10）的最大值
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 10
-- true改成false,不使用IMU数据，大家可以开启，然后对比下效果
TRAJECTORY_BUILDER_2D.use_imu_data = false  --- 如果是true，地图无法显示
-- false改成true,使用实时回环检测来进行前端的扫描匹配
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true 
-- 1.0改成0.1,提高对运动的敏感度
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters =0.1
-- TRAJECTORY_BUILDER_2D.submaps.num_range_data = 30  注释掉后效果好点
-- TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.1
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- 0.55改成0.65,Fast csm的最低分数，高于此分数才进行优化。
POSE_GRAPH.constraint_builder.min_score = 0.55
--0.6改成0.7,全局定位最小分数，低于此分数则认为目前全局定位不准确
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
POSE_GRAPH.constraint_builder.log_matches = true
POSE_GRAPH.max_num_final_iterations = 10

-- 设置0可关闭全局SLAM
POSE_GRAPH.optimize_every_n_nodes = 30

return options