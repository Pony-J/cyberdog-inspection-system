#pragma once

#include <atomic>
#include <shared_mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace communication::ros2 {

struct Pose2D {
  float x = 0.0f;
  float y = 0.0f;
  bool valid = false;
};

class PoseProvider {
public:
  PoseProvider(
    const std::string & map_frame = "map",
    const std::string & robot_frame = "base_footprint");
  ~PoseProvider();

  bool start();
  void stop();

  Pose2D get_pose();
  bool is_valid();
  std::string get_ns() const { return ns_; }

private:
  void spin_thread();
  void update_pose();

  std::string ns_;
  std::string map_frame_;
  std::string robot_frame_;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::thread spin_thread_;
  std::atomic<bool> running_{false};

  std::shared_mutex pose_mutex_;
  Pose2D current_pose_;
};

}  // namespace communication::ros2
