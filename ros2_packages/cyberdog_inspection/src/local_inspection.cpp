#include "cyberdog_inspection/local_inspection.hpp"

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <set>
#include <sstream>
#include <stdexcept>
#include <sys/wait.h>
#include <unistd.h>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <yaml-cpp/yaml.h>

#include "cyberdog_inspection/http_control_server.hpp"
#include "utils/gif_writer.hpp"
#include "utils/utils.hpp"

namespace cyberdog_inspection {

namespace {

std::string resolve_path(const std::filesystem::path & base_dir, const std::string & value) {
  if (value.empty()) {
    return value;
  }

  const std::filesystem::path path(value);
  if (path.is_absolute()) {
    return path.lexically_normal().string();
  }
  return (base_dir / path).lexically_normal().string();
}

std::string get_default_python_wrapper(const std::filesystem::path & config_dir) {
  try {
    const auto prefix = ament_index_cpp::get_package_prefix("cyberdog_inspection");
    const auto installed =
      std::filesystem::path(prefix) / "lib" / "cyberdog_inspection" / "run_mstc_server.py";
    if (std::filesystem::exists(installed)) {
      return installed.string();
    }
  } catch (...) {
  }

  const auto source_script =
    (config_dir / "../scripts/run_mstc_server.py").lexically_normal();
  return source_script.string();
}

std::vector<std::string> collect_scene_names(const std::string & maps_directory) {
  std::vector<std::string> scene_names;
  std::set<std::string> seen;
  const std::filesystem::path root(maps_directory);
  if (!std::filesystem::exists(root) || !std::filesystem::is_directory(root)) {
    return scene_names;
  }

  for (const auto & entry : std::filesystem::directory_iterator(root)) {
    if (entry.is_regular_file() && entry.path().extension() == ".yaml") {
      const auto scene_name = entry.path().stem().string();
      if (seen.insert(scene_name).second) {
        scene_names.push_back(scene_name);
      }
      continue;
    }

    if (!entry.is_directory()) {
      continue;
    }

    const auto scene_name = entry.path().filename().string();
    const auto nested_yaml = entry.path() / (scene_name + ".yaml");
    if (std::filesystem::exists(nested_yaml) && seen.insert(scene_name).second) {
      scene_names.push_back(scene_name);
    }
  }

  std::sort(scene_names.begin(), scene_names.end());
  return scene_names;
}

std::string find_latest_matching_file(
  const std::filesystem::path & directory, const std::string & prefix,
  const std::string & extension)
{
  if (!std::filesystem::exists(directory) || !std::filesystem::is_directory(directory)) {
    return "";
  }

  std::filesystem::file_time_type latest_time;
  std::filesystem::path latest_path;
  bool found = false;

  for (const auto & entry : std::filesystem::directory_iterator(directory)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    const auto file_name = entry.path().filename().string();
    if (!prefix.empty() && file_name.rfind(prefix, 0) != 0) {
      continue;
    }
    if (!extension.empty() && entry.path().extension() != extension) {
      continue;
    }

    const auto mtime = std::filesystem::last_write_time(entry.path());
    if (!found || mtime > latest_time) {
      latest_time = mtime;
      latest_path = entry.path();
      found = true;
    }
  }

  return found ? latest_path.string() : "";
}

}  // namespace

InspectionConfig load_config(const std::string & config_path) {
  const auto yaml = YAML::LoadFile(config_path);
  const auto config_dir = std::filesystem::path(config_path).parent_path();

  InspectionConfig cfg;
  const char * ns_env = std::getenv("CYBERDOG_NS");
  cfg.ns = (ns_env && *ns_env) ? std::string(ns_env) : yaml["ns"].as<std::string>();
  cfg.http_port = yaml["http"] && yaml["http"]["port"] ? yaml["http"]["port"].as<uint16_t>() : 8083;

  cfg.maps_directory = resolve_path(config_dir, yaml["maps_directory"].as<std::string>());
  cfg.visualization_enabled = yaml["visualization"] && yaml["visualization"]["enabled"] ?
    yaml["visualization"]["enabled"].as<bool>() : false;
  cfg.gif_output_directory = yaml["visualization"] && yaml["visualization"]["gif_output_directory"] ?
    resolve_path(config_dir, yaml["visualization"]["gif_output_directory"].as<std::string>()) :
    resolve_path(config_dir, "../test_output/gifs");
  cfg.debug_map_output_directory =
    yaml["visualization"] && yaml["visualization"]["debug_map_output_directory"] ?
    resolve_path(config_dir, yaml["visualization"]["debug_map_output_directory"].as<std::string>()) :
    resolve_path(config_dir, "../test_output/maps");
  cfg.gif_frame_delay_ms = yaml["visualization"] && yaml["visualization"]["gif_frame_delay_ms"] ?
    yaml["visualization"]["gif_frame_delay_ms"].as<int>() : 80;
  cfg.gif_draw_interval = yaml["visualization"] && yaml["visualization"]["gif_draw_interval"] ?
    yaml["visualization"]["gif_draw_interval"].as<int>() : 5;

  cfg.python_server_script = resolve_path(
    config_dir, yaml["python_server"]["script"].as<std::string>());
  if (cfg.python_server_script.empty()) {
    cfg.python_server_script = get_default_python_wrapper(config_dir);
  }

  cfg.forward_step = yaml["carrot_chaser"]["forward_step"].as<float>();
  cfg.tol = yaml["carrot_chaser"]["tol"].as<float>();

  cfg.mstc_config.robot_size = yaml["mstc"]["robot_size"].as<float>();
  cfg.mstc_config.min_obs_radius = yaml["mstc"]["min_obs_radius"].as<float>();
  cfg.mstc_config.grpc_port = yaml["mstc"]["grpc_port"].as<int>();
  cfg.mstc_config.downsample_interval = yaml["mstc"]["downsample_interval"].as<float>();
  cfg.mstc_config.mode = yaml["mstc"]["mode"].as<std::string>();
  cfg.mstc_config.coverage_interval = yaml["mstc"]["coverage_interval"].as<float>();

  cfg.nav2_action_name = yaml["nav2"]["action_name"].as<std::string>();
  cfg.map_frame = yaml["nav2"]["frame_id"].as<std::string>();
  cfg.robot_frame = yaml["nav2"]["robot_frame"].as<std::string>();
  cfg.nav2_wait_for_server_timeout_sec = yaml["nav2"]["wait_for_server_timeout_sec"].as<double>();

  cfg.navigation_enabled = yaml["navigation_enabled"].as<bool>();
  return cfg;
}

LocalInspectionService::LocalInspectionService(const InspectionConfig & config)
: config_(config),
  pose_provider_(config.map_frame, config.robot_frame),
  nav2_client_(
    config.nav2_action_name,
    config.map_frame,
    config.nav2_wait_for_server_timeout_sec)
{
  if (!pose_provider_.start()) {
    throw std::runtime_error("Failed to start pose provider");
  }

  if (config_.navigation_enabled && !nav2_client_.start()) {
    throw std::runtime_error("Failed to connect to Nav2 action server");
  }

  http_server_ = std::make_unique<HttpControlServer>(
    *this, config_.http_port, std::vector<std::string>{"127.0.0.1"});
  if (!http_server_->start()) {
    throw std::runtime_error("Failed to start local HTTP control server");
  }

  execution_thread_ = std::thread(&LocalInspectionService::execution_loop, this);
}

LocalInspectionService::~LocalInspectionService() {
  shutdown();
  if (execution_thread_.joinable()) {
    execution_thread_.join();
  }
  if (http_server_) {
    http_server_->stop();
  }
  nav2_client_.stop();
  pose_provider_.stop();
  stop_python_server();
}

void LocalInspectionService::run() {
  while (running_) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

void LocalInspectionService::shutdown() {
  running_ = false;
  execute_requested_ = false;
  cv_.notify_all();
}

bool LocalInspectionService::initialize(const std::string & scene_name, std::string * message) {
  const std::string requested_scene = scene_name.empty() ? "map" : scene_name;
  {
    std::shared_lock<std::shared_mutex> lock(status_mutex_);
    if (status_ == InspectionStatus::INITIALIZING ||
      status_ == InspectionStatus::INSPECTION_IN_PROGRESS ||
      status_ == InspectionStatus::INSPECTION_PAUSED)
    {
      if (message) {
        *message = "Inspection service is busy";
      }
      return false;
    }
  }

  std::thread(&LocalInspectionService::initialize_impl, this, requested_scene).detach();
  if (message) {
    *message = "Initialization started";
  }
  return true;
}

bool LocalInspectionService::start_inspection(std::string * message) {
  std::shared_ptr<algorithm::carrot_chaser::CarrotChaser> chaser;
  {
    std::shared_lock<std::shared_mutex> lock(status_mutex_);
    if (status_ != InspectionStatus::READY) {
      if (message) {
        *message = "Inspection is not ready";
      }
      return false;
    }
  }

  std::vector<std::vector<float>> path_copy;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    if (planned_path_.empty()) {
      if (message) {
        *message = "No planned path available";
      }
      return false;
    }
    path_copy = planned_path_;
  }

  chaser = std::make_shared<algorithm::carrot_chaser::CarrotChaser>(
    path_copy, config_.forward_step, config_.tol);
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    carrot_chaser_ = chaser;
  }

  execute_requested_ = true;
  set_status(InspectionStatus::INSPECTION_IN_PROGRESS, "Inspection started");
  cv_.notify_one();
  if (message) {
    *message = "Inspection started";
  }
  return true;
}

bool LocalInspectionService::pause_inspection(std::string * message) {
  {
    std::shared_lock<std::shared_mutex> lock(status_mutex_);
    if (status_ != InspectionStatus::INSPECTION_IN_PROGRESS) {
      if (message) {
        *message = "Inspection is not running";
      }
      return false;
    }
  }

  execute_requested_ = false;
  nav2_client_.cancel_current_goal();
  set_status(InspectionStatus::INSPECTION_PAUSED, "Inspection paused");
  if (message) {
    *message = "Inspection paused";
  }
  return true;
}

bool LocalInspectionService::resume_inspection(std::string * message) {
  {
    std::shared_lock<std::shared_mutex> lock(status_mutex_);
    if (status_ != InspectionStatus::INSPECTION_PAUSED) {
      if (message) {
        *message = "Inspection is not paused";
      }
      return false;
    }
  }

  execute_requested_ = true;
  set_status(InspectionStatus::INSPECTION_IN_PROGRESS, "Inspection resumed");
  cv_.notify_one();
  if (message) {
    *message = "Inspection resumed";
  }
  return true;
}

bool LocalInspectionService::stop_inspection(std::string * message) {
  execute_requested_ = false;
  nav2_client_.cancel_current_goal();

  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    carrot_chaser_.reset();
    planned_path_.clear();
  }

  active_scene_name_.clear();
  set_status(InspectionStatus::IDLE, "Inspection stopped");
  if (message) {
    *message = "Inspection stopped";
  }
  return true;
}

bool LocalInspectionService::reset(std::string * message) {
  return stop_inspection(message);
}

void LocalInspectionService::initialize_impl(const std::string & scene_name) {
  set_status(InspectionStatus::INITIALIZING, "Planning inspection path");

  try {
    auto pose = pose_provider_.get_pose();
    if (!pose.valid) {
      throw std::runtime_error("Robot pose is not available from TF");
    }

    start_python_server();

    const auto map_yaml_path = resolve_map_yaml(scene_name);
    const auto map_config = YAML::LoadFile(map_yaml_path);
    std::string map_pgm_path;
    if (utils::get_pgm_from_yaml_ros(map_yaml_path, map_pgm_path) != 0) {
      throw std::runtime_error("Failed to resolve map PGM from YAML");
    }

    std::unique_ptr<mstc_wrapper::MSTCPlanner> planner;
    const auto debug_dir = config_.visualization_enabled ?
      (std::filesystem::path(config_.debug_map_output_directory) / scene_name).string() :
      std::string();
    if (config_.mstc_config.mode == "coverage") {
      planner = std::make_unique<mstc_wrapper::AreaCoveragePlanner>(
        map_pgm_path,
        map_config,
        config_.mstc_config.robot_size,
        config_.mstc_config.min_obs_radius,
        config_.mstc_config.grpc_port,
        config_.mstc_config.downsample_interval,
        config_.mstc_config.coverage_interval,
        debug_dir);
    } else {
      planner = std::make_unique<mstc_wrapper::PathCoveragePlanner>(
        map_pgm_path,
        map_config,
        config_.mstc_config.robot_size,
        config_.mstc_config.min_obs_radius,
        config_.mstc_config.grpc_port,
        config_.mstc_config.downsample_interval,
        debug_dir);
    }

    if (!planner->setup_planner()) {
      throw std::runtime_error("Planner setup failed");
    }

    mstc_wrapper::RoboWorldPoses robot_poses;
    robot_poses[config_.ns] = {pose.x, pose.y};

    mstc_wrapper::PathsWorld paths_world;
    if (!planner->plan(robot_poses, paths_world)) {
      throw std::runtime_error("MSTC planning failed");
    }

    const auto it = paths_world.find(config_.ns);
    if (it == paths_world.end() || it->second.empty()) {
      throw std::runtime_error("No path generated for the local robot");
    }

    std::vector<std::vector<float>> local_path;
    local_path.reserve(it->second.size());
    for (const auto & point : it->second) {
      local_path.push_back({point.first, point.second});
    }

    std::vector<std::vector<float>> gif_path_points;
    {
      std::lock_guard<std::mutex> lock(path_mutex_);
      planned_path_ = local_path;
      carrot_chaser_.reset();
      active_scene_name_ = scene_name;
      gif_path_points = planned_path_;
    }
    if (config_.visualization_enabled) {
      generate_path_gif(scene_name, map_yaml_path, gif_path_points);
    }
    set_status(InspectionStatus::READY, "Inspection path ready");
  } catch (const std::exception & e) {
    set_status(InspectionStatus::ERROR, e.what());
  }
}

void LocalInspectionService::execution_loop() {
  int resend_counter = 0;
  constexpr int kResendInterval = 8;

  while (running_) {
    {
      std::unique_lock<std::mutex> lock(cv_mutex_);
      cv_.wait(lock, [this] { return execute_requested_ || !running_; });
    }

    resend_counter = 0;
    while (running_ && execute_requested_) {
      std::shared_ptr<algorithm::carrot_chaser::CarrotChaser> chaser;
      {
        std::lock_guard<std::mutex> lock(path_mutex_);
        chaser = carrot_chaser_;
      }

      if (!chaser) {
        set_status(InspectionStatus::ERROR, "Execution requested without a path");
        execute_requested_ = false;
        break;
      }

      if (chaser->is_completed()) {
        nav2_client_.cancel_current_goal();
        execute_requested_ = false;
        set_status(InspectionStatus::READY, "Inspection completed");
        break;
      }

      auto pose = pose_provider_.get_pose();
      if (pose.valid) {
        auto update_result = chaser->update({pose.x, pose.y});
        ++resend_counter;
        if (!execute_requested_) {
          break;
        }
        if (update_result.first || (resend_counter % kResendInterval == 0)) {
          if (config_.navigation_enabled) {
            RCLCPP_INFO(
              rclcpp::get_logger("cyberdog_inspection"),
              "Sending Nav2 goal to x=%.3f y=%.3f",
              update_result.second[0], update_result.second[1]);
            nav2_client_.send_goal(update_result.second[0], update_result.second[1], 0.0f);
          }
        }
      }

      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

void LocalInspectionService::start_python_server() {
  if (python_server_pid_ > 0) {
    return;
  }

  const std::string port_str = std::to_string(config_.mstc_config.grpc_port);
  python_server_pid_ = fork();
  if (python_server_pid_ == 0) {
    execlp(
      "python3",
      "python3",
      config_.python_server_script.c_str(),
      "--port",
      port_str.c_str(),
      static_cast<char *>(nullptr));
    _exit(127);
  }

  if (python_server_pid_ < 0) {
    throw std::runtime_error("Failed to fork MSTC Python server");
  }

  for (int i = 0; i < 20; ++i) {
    int status = 0;
    const auto result = waitpid(python_server_pid_, &status, WNOHANG);
    if (result > 0) {
      python_server_pid_ = -1;
      throw std::runtime_error("MSTC Python server exited during startup");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void LocalInspectionService::stop_python_server() {
  if (python_server_pid_ <= 0) {
    return;
  }

  kill(python_server_pid_, SIGTERM);
  for (int i = 0; i < 30; ++i) {
    int status = 0;
    const auto result = waitpid(python_server_pid_, &status, WNOHANG);
    if (result != 0) {
      python_server_pid_ = -1;
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  kill(python_server_pid_, SIGKILL);
  int status = 0;
  waitpid(python_server_pid_, &status, 0);
  python_server_pid_ = -1;
}

std::string LocalInspectionService::resolve_map_yaml(const std::string & scene_name) const {
  const std::string resolved_name = scene_name.empty() ? "map" : scene_name;
  const auto direct = std::filesystem::path(config_.maps_directory) / (resolved_name + ".yaml");
  if (std::filesystem::exists(direct)) {
    return direct.string();
  }

  const auto nested =
    std::filesystem::path(config_.maps_directory) / resolved_name / (resolved_name + ".yaml");
  if (std::filesystem::exists(nested)) {
    return nested.string();
  }

  throw std::runtime_error("Cannot find map yaml for scene: " + resolved_name);
}

void LocalInspectionService::set_status(InspectionStatus status, const std::string & msg) {
  std::unique_lock<std::shared_mutex> lock(status_mutex_);
  status_ = status;
  status_msg_ = msg;
}

void LocalInspectionService::generate_path_gif(
  const std::string & scene_name, const std::string & map_yaml_path,
  const std::vector<std::vector<float>> & path) const
{
  if (path.empty()) {
    return;
  }

  const auto map_config = YAML::LoadFile(map_yaml_path);
  std::string map_pgm_path;
  if (utils::get_pgm_from_yaml_ros(map_yaml_path, map_pgm_path) != 0) {
    throw std::runtime_error("Failed to resolve map PGM for GIF generation");
  }

  cv::Mat original_map = cv::imread(map_pgm_path, cv::IMREAD_GRAYSCALE);
  if (original_map.empty()) {
    throw std::runtime_error("Failed to load map image for GIF generation");
  }

  const float resolution = map_config["resolution"].as<float>();
  const auto origin = map_config["origin"].as<std::vector<float>>();
  const float origin_x = origin[0];
  const float origin_y = origin[1];

  auto world2grid = [&](float x, float y) -> std::pair<int, int> {
    const int col = static_cast<int>((x - origin_x) / resolution);
    const int row = original_map.rows - 1 - static_cast<int>((y - origin_y) / resolution);
    return {row, col};
  };

  std::filesystem::create_directories(config_.gif_output_directory);

  const auto now = std::chrono::system_clock::now();
  const auto time_t_now = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&time_t_now), "%Y%m%d_%H%M%S");
  const auto gif_path =
    std::filesystem::path(config_.gif_output_directory) /
    ("path_" + scene_name + "_" + ss.str() + ".gif");

  utils::GifWriterWrapper writer;
  const int delay_cs = std::max(1, config_.gif_frame_delay_ms / 10);
  if (!writer.begin(gif_path.string(), original_map.cols, original_map.rows, delay_cs)) {
    throw std::runtime_error("Failed to open GIF writer");
  }

  const size_t draw_interval = static_cast<size_t>(std::max(1, config_.gif_draw_interval));
  for (size_t frame = 0; frame < path.size(); frame += draw_interval) {
    cv::Mat3b canvas;
    cv::cvtColor(original_map, canvas, cv::COLOR_GRAY2BGR);

    const size_t end_idx = std::min(frame, path.size() - 1);
    for (size_t i = 1; i <= end_idx; ++i) {
      const auto [row1, col1] = world2grid(path[i - 1][0], path[i - 1][1]);
      const auto [row2, col2] = world2grid(path[i][0], path[i][1]);
      cv::line(canvas, cv::Point(col1, row1), cv::Point(col2, row2), cv::Scalar(30, 144, 255), 2);
    }

    const auto [row, col] = world2grid(path[end_idx][0], path[end_idx][1]);
    cv::circle(canvas, cv::Point(col, row), 5, cv::Scalar(0, 69, 255), -1);
    cv::circle(canvas, cv::Point(col, row), 7, cv::Scalar(255, 255, 255), 1);
    writer.addFrame(canvas);
  }

  cv::Mat3b final_canvas;
  cv::cvtColor(original_map, final_canvas, cv::COLOR_GRAY2BGR);
  for (size_t i = 1; i < path.size(); ++i) {
    const auto [row1, col1] = world2grid(path[i - 1][0], path[i - 1][1]);
    const auto [row2, col2] = world2grid(path[i][0], path[i][1]);
    cv::line(final_canvas, cv::Point(col1, row1), cv::Point(col2, row2), cv::Scalar(30, 144, 255), 2);
  }
  const auto [start_row, start_col] = world2grid(path.front()[0], path.front()[1]);
  const auto [end_row, end_col] = world2grid(path.back()[0], path.back()[1]);
  cv::circle(final_canvas, cv::Point(start_col, start_row), 6, cv::Scalar(0, 0, 255), -1);
  cv::circle(final_canvas, cv::Point(end_col, end_row), 6, cv::Scalar(0, 255, 0), -1);
  for (int i = 0; i < 12; ++i) {
    writer.addFrame(final_canvas);
  }
  writer.end();

  RCLCPP_INFO(
    rclcpp::get_logger("cyberdog_inspection"),
    "Saved inspection path GIF to %s", gif_path.c_str());
}

InspectionStatus LocalInspectionService::status() const {
  std::shared_lock<std::shared_mutex> lock(status_mutex_);
  return status_;
}

std::string LocalInspectionService::status_name() const {
  switch (status()) {
    case InspectionStatus::IDLE:
      return "IDLE";
    case InspectionStatus::INITIALIZING:
      return "INITIALIZING";
    case InspectionStatus::READY:
      return "READY";
    case InspectionStatus::INSPECTION_IN_PROGRESS:
      return "INSPECTION_IN_PROGRESS";
    case InspectionStatus::INSPECTION_PAUSED:
      return "INSPECTION_PAUSED";
    case InspectionStatus::ERROR:
      return "ERROR";
    default:
      return "UNKNOWN";
  }
}

std::string LocalInspectionService::status_message() const {
  std::shared_lock<std::shared_mutex> lock(status_mutex_);
  return status_msg_;
}

std::string LocalInspectionService::nav2_status() const {
  return nav2_client_.last_status();
}

std::string LocalInspectionService::active_scene_name() const {
  std::lock_guard<std::mutex> lock(path_mutex_);
  return active_scene_name_;
}

std::vector<std::vector<float>> LocalInspectionService::planned_path() const {
  std::lock_guard<std::mutex> lock(path_mutex_);
  return planned_path_;
}

std::vector<InspectionMapInfo> LocalInspectionService::list_maps() const {
  std::vector<InspectionMapInfo> maps;
  for (const auto & scene_name : collect_scene_names(config_.maps_directory)) {
    InspectionMapInfo info;
    info.scene_name = scene_name;

    try {
      info.yaml_path = resolve_map_yaml(scene_name);
      const auto map_config = YAML::LoadFile(info.yaml_path);

      if (utils::get_pgm_from_yaml_ros(info.yaml_path, info.pgm_path) != 0) {
        throw std::runtime_error("Failed to resolve map image from YAML");
      }

      if (!map_config["resolution"] || !map_config["origin"]) {
        throw std::runtime_error("Map YAML is missing resolution or origin");
      }

      info.resolution = map_config["resolution"].as<float>();
      info.origin = map_config["origin"].as<std::vector<float>>();

      const cv::Mat map_image = cv::imread(info.pgm_path, cv::IMREAD_GRAYSCALE);
      if (map_image.empty()) {
        throw std::runtime_error("Failed to load map image");
      }

      info.width = map_image.cols;
      info.height = map_image.rows;
      info.available = true;
    } catch (const std::exception & e) {
      info.available = false;
      info.error_message = e.what();
    }

    maps.push_back(std::move(info));
  }

  return maps;
}

InspectionArtifacts LocalInspectionService::map_artifacts(const std::string & scene_name) const {
  InspectionArtifacts artifacts;
  artifacts.scene_name = scene_name.empty() ? active_scene_name() : scene_name;
  if (artifacts.scene_name.empty()) {
    return artifacts;
  }

  const auto debug_dir = std::filesystem::path(config_.debug_map_output_directory) / artifacts.scene_name;
  if (std::filesystem::exists(debug_dir) && std::filesystem::is_directory(debug_dir)) {
    for (const auto & entry : std::filesystem::directory_iterator(debug_dir)) {
      if (!entry.is_regular_file()) {
        continue;
      }
      const auto extension = entry.path().extension().string();
      if (extension == ".png" || extension == ".jpg" || extension == ".jpeg") {
        artifacts.debug_map_paths.push_back(entry.path().string());
      }
    }
    std::sort(artifacts.debug_map_paths.begin(), artifacts.debug_map_paths.end());
  }

  artifacts.latest_gif_path = find_latest_matching_file(
    config_.gif_output_directory, "path_" + artifacts.scene_name + "_", ".gif");
  return artifacts;
}

}  // namespace cyberdog_inspection
