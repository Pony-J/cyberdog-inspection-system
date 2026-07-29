#pragma once

#include <filesystem>
#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "grpc/client/client.hpp"
#include "mstc_wrapper/types.hpp"

namespace mstc_wrapper {

class MSTCPlanner {
public:
  MSTCPlanner(
    const std::string & map_path, const YAML::Node & map_config, float robot_size,
    float min_obs_radius, int grpc_port, const std::string & debug_output_directory = "");
  virtual ~MSTCPlanner();

  bool setup_planner();
  bool plan(const RoboWorldPoses & robot_positions, PathsWorld & paths);

protected:
  PointGrid world2grid(const PointWorld & p) const;
  PointGrid world2grid(float x, float y) const;
  PointWorld grid2world(const PointGrid & p) const;
  PointWorld grid2world(int i, int j) const;
  static void grid2graph(const cv::Mat1b & grid, WeighedEdges & graph);
  bool debug_enabled() const;
  void save_debug_grid(const std::string & filename, const cv::Mat1b & grid) const;
  void save_debug_grid_with_robots(
    const std::string & filename, const cv::Mat1b & grid,
    const RoboGridPoses & robot_positions) const;
  void save_debug_edges(
    const std::string & filename, const cv::Mat1b & grid, const WeighedEdges & edges,
    const RoboGridPoses & robot_positions) const;

private:
  virtual bool preprocess_map(
    const cv::Mat1b & map, const YAML::Node & map_config,
    const RoboGridPoses & robot_positions, WeighedEdges & graph) = 0;
  virtual bool postprocess_paths(
    const cv::Mat1b & map, const YAML::Node & map_config,
    const RoboGridPoses & robot_positions, const PathsWorld & paths,
    PathsWorld & refined_paths) = 0;

  void mstc_star_wrapper(const WeighedEdges & g, const RoboGridPoses & positions, PathsGrid & paths);
  static bool load_map(const std::string & map_path, cv::Mat1b & map);
  static bool sanity_check(const YAML::Node & map_config);
  static void filter_grid(cv::Mat1b & grid, int thresh, int expansion, int min_obs_radius);

  mstc_grpc::MSTCClient grpc_client_;
  std::string map_path_;
  YAML::Node map_config_;
  cv::Mat1b original_map_;
  float robot_size_;
  float min_obs_radius_;
  std::string debug_output_directory_;
};

}  // namespace mstc_wrapper
