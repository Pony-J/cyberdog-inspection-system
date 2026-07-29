include "cyberdog_slam.lua"

----------------------------------------------------------------------
-- 纯定位模式覆盖
-- 加载 .pbstream 地图，不再扩建新地图，仅做实时定位
----------------------------------------------------------------------

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

-- 更频繁优化以加快定位收敛
POSE_GRAPH.optimize_every_n_nodes = 20

-- 更大搜索窗口以支持初始重定位
POSE_GRAPH.constraint_builder.sampling_ratio = 0.5
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 10.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(60.)

POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

return options
