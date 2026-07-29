#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <sys/types.h>
#include <thread>
#include <vector>

#include "algorithm/carrot_chaser.hpp"
#include "communication/ros2/pose_provider.hpp"
#include "mstc_wrapper/area_coverage.hpp"
#include "mstc_wrapper/path_coverage.hpp"

#include "cyberdog_inspection/nav2_client.hpp"

namespace cyberdog_inspection {

class HttpControlServer;

enum class InspectionStatus : int {
  IDLE = 0,
  INITIALIZING = 1,
  READY = 2,
  INSPECTION_IN_PROGRESS = 3,
  INSPECTION_PAUSED = 4,
  ERROR = 5
};

struct InspectionMapInfo {
  std::string scene_name;
  std::string yaml_path;
  std::string pgm_path;
  bool available{false};
  float resolution{0.0f};
  std::vector<float> origin;
  int width{0};
  int height{0};
  std::string error_message;
};

struct InspectionArtifacts {
  std::string scene_name;
  std::string latest_gif_path;
  std::vector<std::string> debug_map_paths;
};

struct InspectionConfig {
  struct MstcConfig {
    float robot_size{0.5f};
    float min_obs_radius{0.1f};
    int grpc_port{50051};
    float downsample_interval{0.1f};
    std::string mode{"coverage"};
    float coverage_interval{1.1f};
  };

  std::string ns;
  uint16_t http_port{8083};
  std::string maps_directory;
  std::string python_server_script;
  bool visualization_enabled{false};
  std::string gif_output_directory;
  std::string debug_map_output_directory;
  int gif_frame_delay_ms{80};
  int gif_draw_interval{5};

  float forward_step{2.0f};
  float tol{0.6f};
  bool navigation_enabled{true};

  MstcConfig mstc_config;

  std::string nav2_action_name{"navigate_to_pose"};
  std::string map_frame{"map"};
  std::string robot_frame{"base_footprint"};
  double nav2_wait_for_server_timeout_sec{10.0};
};

InspectionConfig load_config(const std::string & config_path);

class LocalInspectionService {
public:
  explicit LocalInspectionService(const InspectionConfig & config);
  ~LocalInspectionService();

  void run();
  void shutdown();

  bool initialize(const std::string & scene_name, std::string * message = nullptr);
  bool start_inspection(std::string * message = nullptr);
  bool pause_inspection(std::string * message = nullptr);
  bool resume_inspection(std::string * message = nullptr);
  bool stop_inspection(std::string * message = nullptr);
  bool reset(std::string * message = nullptr);

  InspectionStatus status() const;
  std::string status_name() const;
  std::string status_message() const;
  std::string nav2_status() const;
  std::string active_scene_name() const;
  std::vector<std::vector<float>> planned_path() const;
  std::vector<InspectionMapInfo> list_maps() const;
  InspectionArtifacts map_artifacts(const std::string & scene_name = "") const;

private:
  void initialize_impl(const std::string & scene_name);
  void execution_loop();
  void start_python_server();
  void stop_python_server();
  std::string resolve_map_yaml(const std::string & scene_name) const;
  void set_status(InspectionStatus status, const std::string & msg);
  void generate_path_gif(
    const std::string & scene_name, const std::string & map_yaml_path,
    const std::vector<std::vector<float>> & path) const;

  InspectionConfig config_;
  communication::ros2::PoseProvider pose_provider_;
  Nav2Client nav2_client_;
  std::unique_ptr<HttpControlServer> http_server_;

  std::atomic<bool> running_{true};
  std::atomic<bool> execute_requested_{false};
  std::thread execution_thread_;
  std::condition_variable cv_;
  std::mutex cv_mutex_;

  mutable std::shared_mutex status_mutex_;
  InspectionStatus status_{InspectionStatus::IDLE};
  std::string status_msg_{"Idle"};

  mutable std::mutex path_mutex_;
  std::vector<std::vector<float>> planned_path_;
  std::shared_ptr<algorithm::carrot_chaser::CarrotChaser> carrot_chaser_;
  std::string active_scene_name_;

  pid_t python_server_pid_{-1};
};

}  // namespace cyberdog_inspection
