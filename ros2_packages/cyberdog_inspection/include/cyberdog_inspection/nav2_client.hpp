#pragma once

#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace cyberdog_inspection {

class Nav2Client {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  Nav2Client(
    const std::string & action_name,
    const std::string & frame_id,
    double wait_for_server_timeout_sec);
  ~Nav2Client();

  bool start();
  void stop();

  bool send_goal(float x, float y, float yaw = 0.0f);
  void cancel_current_goal();

  std::string last_status() const;

private:
  void spin_loop();
  void set_status(const std::string & status);
  bool has_active_goal_locked() const;
  static bool same_goal(float lhs_x, float lhs_y, float lhs_yaw, float rhs_x, float rhs_y, float rhs_yaw);

  std::string action_name_;
  std::string frame_id_;
  double wait_for_server_timeout_sec_;

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  std::thread spin_thread_;
  std::atomic<bool> running_{false};

  mutable std::mutex mutex_;
  GoalHandle::SharedPtr current_goal_handle_;
  std::string last_status_{"IDLE"};
  bool has_last_goal_{false};
  float last_goal_x_{0.0f};
  float last_goal_y_{0.0f};
  float last_goal_yaw_{0.0f};
};

}  // namespace cyberdog_inspection
