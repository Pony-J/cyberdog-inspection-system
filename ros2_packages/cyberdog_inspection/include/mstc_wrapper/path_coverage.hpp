#pragma once

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "mstc_wrapper/graph.hpp"
#include "mstc_wrapper/mstc.hpp"

namespace mstc_wrapper {

class PathCoveragePlanner : public MSTCPlanner {
public:
  PathCoveragePlanner(
    const std::string & map_path, const YAML::Node & map_config, float robot_size,
    float min_obs_radius, int grpc_port, float downsample_interval,
    const std::string & debug_output_directory = "")
  : MSTCPlanner(
      map_path, map_config, robot_size, min_obs_radius, grpc_port, debug_output_directory),
    downsample_interval_(downsample_interval) {}

private:
  bool preprocess_map(
    const cv::Mat1b & map, const YAML::Node &, const RoboGridPoses & robot_positions,
    WeighedEdges & graph) override
  {
    grid2graph(map, graph);
    cv::Mat1b skel;
    extract_skeleton(map, skel);
    save_debug_grid("02_path_skeleton.png", skel);
    if (!add_robot_to_skeleton(skel, robot_positions, graph)) {
      return false;
    }
    save_debug_grid_with_robots("03_path_skeleton_with_robot.png", skel, robot_positions);
    grid2graph(skel, graph);
    save_debug_edges("04_path_graph.png", skel, graph, robot_positions);
    return true;
  }

  bool postprocess_paths(
    const cv::Mat1b &, const YAML::Node &, const RoboGridPoses &,
    const PathsWorld & paths, PathsWorld & refined_paths) override
  {
    downsample_paths(paths, downsample_interval_, refined_paths);
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

  static void extract_skeleton(const cv::Mat1b & gray_map, cv::Mat1b & skel)
  {
    cv::Mat1b input = (gray_map.data == skel.data) ? gray_map.clone() : gray_map;
    cv::threshold(input, skel, 127, 255, cv::THRESH_BINARY);
    thinning(skel);
  }

  static bool add_robot_to_skeleton(
    cv::Mat1b & skel, const RoboGridPoses & robot_positions, const WeighedEdges & ref_graph,
    int search_radius = 25)
  {
    Graph g(ref_graph);
    for (const auto & p : robot_positions) {
      const auto & pos = p.second;
      if (!g.has_node(pos)) {
        return false;
      }
      std::vector<PointGrid> neighbors;
      for (int i = -search_radius; i <= search_radius; ++i) {
        for (int j = -search_radius; j <= search_radius; ++j) {
          PointGrid neighbor = {pos.first + i, pos.second + j};
          if (g.has_node(neighbor) && skel.at<uchar>(neighbor.first, neighbor.second) == 255) {
            neighbors.push_back(neighbor);
          }
        }
      }
      if (neighbors.empty()) {
        return false;
      }
      std::vector<Node> min_path;
      Node out_target;
      if (g.batch_dijkstra(pos, neighbors, min_path, out_target) < 0 || min_path.empty()) {
        return false;
      }
      for (const auto & q : min_path) {
        skel.at<uchar>(q.first, q.second) = 255;
      }
    }
    return true;
  }

  static void thinningIteration(cv::Mat1b & img, int iter)
  {
    CV_Assert(img.type() == CV_8UC1);
    cv::Mat1b marker = cv::Mat1b::zeros(img.size());
    for (int i = 1; i < img.rows - 1; ++i) {
      for (int j = 1; j < img.cols - 1; ++j) {
        uchar p2 = img.at<uchar>(i - 1, j);
        uchar p3 = img.at<uchar>(i - 1, j + 1);
        uchar p4 = img.at<uchar>(i, j + 1);
        uchar p5 = img.at<uchar>(i + 1, j + 1);
        uchar p6 = img.at<uchar>(i + 1, j);
        uchar p7 = img.at<uchar>(i + 1, j - 1);
        uchar p8 = img.at<uchar>(i, j - 1);
        uchar p9 = img.at<uchar>(i - 1, j - 1);
        int A = (p2 == 0 && p3 == 1) + (p3 == 0 && p4 == 1) +
          (p4 == 0 && p5 == 1) + (p5 == 0 && p6 == 1) +
          (p6 == 0 && p7 == 1) + (p7 == 0 && p8 == 1) +
          (p8 == 0 && p9 == 1) + (p9 == 0 && p2 == 1);
        int B = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
        if (img.at<uchar>(i, j) == 1 && A == 1 && (B >= 2 && B <= 6)) {
          if (iter == 0) {
            if ((p2 * p4 * p6 == 0) && (p4 * p6 * p8 == 0)) {
              marker.at<uchar>(i, j) = 1;
            }
          } else {
            if ((p2 * p4 * p8 == 0) && (p2 * p6 * p8 == 0)) {
              marker.at<uchar>(i, j) = 1;
            }
          }
        }
      }
    }
    cv::subtract(img, marker, img);
  }

  static void thinning(cv::Mat1b & src)
  {
    CV_Assert(src.type() == CV_8UC1);
    cv::Mat1b img;
    src.copyTo(img);
    cv::threshold(img, img, 127, 1, cv::THRESH_BINARY);
    cv::Mat1b prev = cv::Mat1b::zeros(img.size());
    cv::Mat1b diff;
    do {
      thinningIteration(img, 0);
      thinningIteration(img, 1);
      cv::absdiff(img, prev, diff);
      img.copyTo(prev);
    } while (cv::countNonZero(diff) > 0);
    img *= 255;
    img.copyTo(src);
  }

  float downsample_interval_;
};

}  // namespace mstc_wrapper
