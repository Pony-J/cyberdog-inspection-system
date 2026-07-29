#pragma once

#include <cmath>
#include <set>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "mstc_wrapper/graph.hpp"
#include "mstc_wrapper/mstc.hpp"

namespace mstc_wrapper {

class AreaCoveragePlanner : public MSTCPlanner {
public:
  AreaCoveragePlanner(
    const std::string & map_path, const YAML::Node & map_config, float robot_size,
    float min_obs_radius, int grpc_port, float downsample_interval, float coverage_interval,
    const std::string & debug_output_directory = "")
  : MSTCPlanner(
      map_path, map_config, robot_size, min_obs_radius, grpc_port, debug_output_directory),
    downsample_interval_(downsample_interval),
    coverage_interval_(coverage_interval) {}

private:
  bool preprocess_map(
    const cv::Mat1b & map, const YAML::Node & map_config, const RoboGridPoses & robot_positions,
    WeighedEdges & graph) override
  {
    grid2graph(map, graph);
    WeighedEdges full_graph = graph;
    float resolution = map_config["resolution"].as<float>();
    int grid_interval = std::max(1, static_cast<int>(coverage_interval_ / resolution));
    build_sampled_graph(map, grid_interval, graph);
    save_debug_edges("02_coverage_sampled_graph.png", map, graph, robot_positions);
    const bool ok = add_robots_to_sampled(robot_positions, full_graph, graph, grid_interval);
    if (ok) {
      save_debug_edges("03_coverage_graph_with_robot.png", map, graph, robot_positions);
    }
    return ok;
  }

  bool postprocess_paths(
    const cv::Mat1b &, const YAML::Node &, const RoboGridPoses &, const PathsWorld & paths,
    PathsWorld & refined_paths) override
  {
    downsample_paths(paths, downsample_interval_, refined_paths);
    return true;
  }

  static void build_sampled_graph(const cv::Mat1b & map, int grid_interval, WeighedEdges & graph)
  {
    graph.clear();
    using NodePair = std::pair<int, int>;
    std::set<NodePair> sampled_set;
    for (int r = 0; r < map.rows; r += grid_interval) {
      for (int c = 0; c < map.cols; c += grid_interval) {
        if (map.at<uchar>(r, c) == 255) {
          sampled_set.insert({r, c});
        }
      }
    }
    const int dr[8] = {-grid_interval, -grid_interval, 0, grid_interval, grid_interval, grid_interval, 0, -grid_interval};
    const int dc[8] = {0, grid_interval, grid_interval, grid_interval, 0, -grid_interval, -grid_interval, -grid_interval};
    for (const auto & [r, c] : sampled_set) {
      for (int d = 0; d < 8; ++d) {
        int nr = r + dr[d];
        int nc = c + dc[d];
        if (sampled_set.count({nr, nc})) {
          float weight = std::sqrt(static_cast<float>(dr[d] * dr[d] + dc[d] * dc[d]));
          graph.push_back({{{r, c}, {nr, nc}}, weight});
        }
      }
    }
  }

  static bool add_robots_to_sampled(
    const RoboGridPoses & robot_positions, const WeighedEdges & full_graph,
    WeighedEdges & sampled_graph, int grid_interval)
  {
    std::set<std::pair<int, int>> sampled_set;
    for (const auto & ew : sampled_graph) {
      const auto & e = ew.first;
      sampled_set.insert({e.first.first, e.first.second});
      sampled_set.insert({e.second.first, e.second.second});
    }

    Graph g(full_graph);
    for (const auto & [id, pos] : robot_positions) {
      if (!g.has_node(pos)) {
        return false;
      }
      int range = std::max(25, grid_interval * 2);
      std::vector<PointGrid> candidates;
      for (int dr = -range; dr <= range; ++dr) {
        for (int dc = -range; dc <= range; ++dc) {
          PointGrid candidate = {pos.first + dr, pos.second + dc};
          if (sampled_set.count({candidate.first, candidate.second}) && g.has_node(candidate)) {
            candidates.push_back(candidate);
          }
        }
      }
      if (candidates.empty()) {
        return false;
      }
      std::vector<Node> path;
      Node target;
      double dist = g.batch_dijkstra(pos, candidates, path, target);
      if (dist < 0 || path.empty()) {
        return false;
      }
      for (size_t i = 0; i + 1 < path.size(); ++i) {
        bool diagonal = (path[i].first != path[i + 1].first && path[i].second != path[i + 1].second);
        float w = diagonal ? std::sqrt(2.0f) : 1.0f;
        sampled_graph.push_back({{path[i], path[i + 1]}, w});
        sampled_graph.push_back({{path[i + 1], path[i]}, w});
      }
    }
    return true;
  }

  void downsample_single_path(const PathWorld & path, float interval, PathWorld & downsampled)
  {
    if (path.empty()) {
      return;
    }
    downsampled.clear();
    downsampled.reserve(path.size());
    downsampled.push_back(path.front());
    float cum_dist = 0.0f;
    for (size_t i = 1; i < path.size(); ++i) {
      float dx = path[i].first - path[i - 1].first;
      float dy = path[i].second - path[i - 1].second;
      cum_dist += std::sqrt(dx * dx + dy * dy);
      if (cum_dist >= interval) {
        downsampled.push_back(path[i]);
        cum_dist = 0.0f;
      }
    }
    if (downsampled.back() != path.back()) {
      downsampled.push_back(path.back());
    }
  }

  void downsample_paths(const PathsWorld & paths, float interval, PathsWorld & result)
  {
    result.clear();
    result.reserve(paths.size());
    for (const auto & kv : paths) {
      PathWorld downsampled;
      downsample_single_path(kv.second, interval, downsampled);
      result[kv.first] = downsampled;
    }
  }

  float downsample_interval_;
  float coverage_interval_;
};

}  // namespace mstc_wrapper
