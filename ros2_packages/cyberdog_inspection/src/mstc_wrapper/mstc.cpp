#include "mstc_wrapper/mstc.hpp"

#include <cmath>
#include <filesystem>

namespace mstc_wrapper {

MSTCPlanner::MSTCPlanner(
  const std::string & map_path, const YAML::Node & map_config, float robot_size,
  float min_obs_radius, int grpc_port, const std::string & debug_output_directory)
: grpc_client_(grpc::CreateChannel(
      "localhost:" + std::to_string(grpc_port), grpc::InsecureChannelCredentials())),
  map_path_(map_path),
  map_config_(map_config),
  robot_size_(robot_size),
  min_obs_radius_(min_obs_radius),
  debug_output_directory_(debug_output_directory)
{
}

MSTCPlanner::~MSTCPlanner() = default;

bool MSTCPlanner::setup_planner()
{
  if (!load_map(map_path_, original_map_)) {
    return false;
  }
  save_debug_grid("00_original_map.png", original_map_);
  return sanity_check(map_config_);
}

bool MSTCPlanner::plan(const RoboWorldPoses & robot_positions, PathsWorld & paths)
{
  RoboGridPoses robot_positions_grid;
  robot_positions_grid.reserve(robot_positions.size());
  for (const auto & p : robot_positions) {
    robot_positions_grid[p.first] = world2grid(p.second);
  }

  int thresh = static_cast<int>(255 * (1 - map_config_["free_thresh"].as<float>())) + 1;
  int expansion = static_cast<int>(robot_size_ / map_config_["resolution"].as<float>() / 2.0f);
  int min_obs_radius = static_cast<int>(min_obs_radius_ / map_config_["resolution"].as<float>());
  filter_grid(original_map_, thresh, expansion, min_obs_radius);
  save_debug_grid("01_filtered_map.png", original_map_);

  WeighedEdges g;
  if (!preprocess_map(original_map_, map_config_, robot_positions_grid, g)) {
    return false;
  }

  PathsGrid paths_grid;
  mstc_star_wrapper(g, robot_positions_grid, paths_grid);

  PathsWorld coarse_paths;
  coarse_paths.reserve(paths_grid.size());
  for (const auto & p : paths_grid) {
    PathWorld path_world;
    path_world.reserve(p.second.size());
    for (const auto & wp : p.second) {
      path_world.push_back(grid2world(wp));
    }
    coarse_paths[p.first] = std::move(path_world);
  }

  return postprocess_paths(original_map_, map_config_, robot_positions_grid, coarse_paths, paths);
}

PointGrid MSTCPlanner::world2grid(const PointWorld & p) const
{
  int j = static_cast<int>(
    std::round((p.first - map_config_["origin"][0].as<float>()) /
    map_config_["resolution"].as<float>()));
  int i = static_cast<int>(
    std::round((map_config_["origin"][1].as<float>() - p.second) /
    map_config_["resolution"].as<float>()) + original_map_.rows);
  return {i, j};
}

PointGrid MSTCPlanner::world2grid(float x, float y) const
{
  return world2grid({x, y});
}

PointWorld MSTCPlanner::grid2world(const PointGrid & p) const
{
  float x = p.second * map_config_["resolution"].as<float>() + map_config_["origin"][0].as<float>();
  float y =
    (original_map_.rows - p.first) * map_config_["resolution"].as<float>() +
    map_config_["origin"][1].as<float>();
  return {x, y};
}

PointWorld MSTCPlanner::grid2world(int i, int j) const
{
  return grid2world({i, j});
}

void MSTCPlanner::grid2graph(const cv::Mat1b & grid, WeighedEdges & graph)
{
  graph.clear();
  const int dx[8] = {-1, -1, 0, 1, 1, 1, 0, -1};
  const int dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};
  const float cost[8] = {1.0f, std::sqrt(2.0f), 1.0f, std::sqrt(2.0f), 1.0f, std::sqrt(2.0f), 1.0f, std::sqrt(2.0f)};

  for (int idx = 0; idx < grid.rows * grid.cols; ++idx) {
    int i = idx / grid.cols;
    int j = idx % grid.cols;
    if (grid.at<uchar>(i, j) == 0) {
      continue;
    }
    for (int d = 0; d < 8; ++d) {
      int ni = i + dy[d];
      int nj = j + dx[d];
      if (ni < 0 || ni >= grid.rows || nj < 0 || nj >= grid.cols) {
        continue;
      }
      if (grid.at<uchar>(ni, nj) == 0) {
        continue;
      }
      graph.push_back({{{i, j}, {ni, nj}}, cost[d]});
    }
  }
}

void MSTCPlanner::mstc_star_wrapper(
  const WeighedEdges & g, const RoboGridPoses & positions, PathsGrid & paths)
{
  paths = grpc_client_.Plan(positions, g);
}

bool MSTCPlanner::load_map(const std::string & map_path, cv::Mat1b & map)
{
  map = cv::imread(map_path, cv::IMREAD_GRAYSCALE);
  if (map.empty()) {
    std::cerr << "[ERROR] Cannot load map from " << map_path << std::endl;
    return false;
  }
  return true;
}

bool MSTCPlanner::sanity_check(const YAML::Node & map_config)
{
  if (!map_config["negate"] || map_config["negate"].as<int>()) {
    std::cerr << "[ERROR] Only negate=0 map is supported" << std::endl;
    return false;
  }
  if (!map_config["origin"] || std::fabs(map_config["origin"][2].as<float>()) > 1e-3f) {
    std::cerr << "[ERROR] Invalid origin in map config, angle must be 0.0" << std::endl;
    return false;
  }
  return true;
}

void MSTCPlanner::filter_grid(cv::Mat1b & grid, int thresh, int expansion, int min_obs_radius)
{
  cv::threshold(grid, grid, thresh, 255, cv::THRESH_BINARY);
  expansion += min_obs_radius;
  if (expansion <= 0) {
    return;
  }
  cv::Mat1b element = cv::getStructuringElement(
    cv::MORPH_ELLIPSE, cv::Size(2 * expansion + 1, 2 * expansion + 1));
  cv::erode(grid, grid, element);
}

bool MSTCPlanner::debug_enabled() const
{
  return !debug_output_directory_.empty();
}

void MSTCPlanner::save_debug_grid(const std::string & filename, const cv::Mat1b & grid) const
{
  if (!debug_enabled() || grid.empty()) {
    return;
  }
  std::filesystem::create_directories(debug_output_directory_);
  cv::imwrite((std::filesystem::path(debug_output_directory_) / filename).string(), grid);
}

void MSTCPlanner::save_debug_grid_with_robots(
  const std::string & filename, const cv::Mat1b & grid,
  const RoboGridPoses & robot_positions) const
{
  if (!debug_enabled() || grid.empty()) {
    return;
  }

  cv::Mat3b canvas;
  cv::cvtColor(grid, canvas, cv::COLOR_GRAY2BGR);
  for (const auto & [robot_id, pose] : robot_positions) {
    cv::circle(canvas, cv::Point(pose.second, pose.first), 5, cv::Scalar(0, 0, 255), -1);
    cv::putText(
      canvas, robot_id, cv::Point(pose.second + 6, pose.first - 6),
      cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255), 1);
  }

  std::filesystem::create_directories(debug_output_directory_);
  cv::imwrite((std::filesystem::path(debug_output_directory_) / filename).string(), canvas);
}

void MSTCPlanner::save_debug_edges(
  const std::string & filename, const cv::Mat1b & grid, const WeighedEdges & edges,
  const RoboGridPoses & robot_positions) const
{
  if (!debug_enabled() || grid.empty()) {
    return;
  }

  cv::Mat3b canvas;
  cv::cvtColor(grid, canvas, cv::COLOR_GRAY2BGR);
  for (const auto & edge : edges) {
    const auto & from = edge.first.first;
    const auto & to = edge.first.second;
    cv::line(
      canvas, cv::Point(from.second, from.first), cv::Point(to.second, to.first),
      cv::Scalar(0, 180, 0), 1);
  }
  for (const auto & [robot_id, pose] : robot_positions) {
    cv::circle(canvas, cv::Point(pose.second, pose.first), 5, cv::Scalar(0, 0, 255), -1);
    cv::putText(
      canvas, robot_id, cv::Point(pose.second + 6, pose.first - 6),
      cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255), 1);
  }

  std::filesystem::create_directories(debug_output_directory_);
  cv::imwrite((std::filesystem::path(debug_output_directory_) / filename).string(), canvas);
}

}  // namespace mstc_wrapper
