include "map_builder.lua"
include "trajectory_builder.lua"

-- Docs: https://google-cartographer-ros.readthedocs.io/en/latest/configuration.html
options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  publish_to_tf = true,
  use_pose_extrapolator = true,
  use_odometry = false,
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

-- Docs: https://github.com/cartographer-project/cartographer/blob/master/configuration_files/trajectory_builder_2d.lua
TRAJECTORY_BUILDER_2D.min_range = 0.2
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 8.5

-- One scan message contains one 360 scan
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- we don't use IMU
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- enable and tune RealTimeCorrelativeScanMatcher which seems to work better with YDLidar
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(35.)

POSE_GRAPH.optimization_problem.huber_scale = 1e2

return options
