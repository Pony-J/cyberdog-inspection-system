#include "cyberdog_inspection/nav2_client.hpp"

#include <chrono>
#include <utility>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace cyberdog_inspection {

Nav2Client::Nav2Client(
  const std::string & action_name,
  const std::string & frame_id,
  double wait_for_server_timeout_sec)
: action_name_(action_name),
  frame_id_(frame_id),
  wait_for_server_timeout_sec_(wait_for_server_timeout_sec) {}

Nav2Client::~Nav2Client() {
  stop();
}

bool Nav2Client::start() {
  if (running_) {
    return true;
  }

  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }

  node_ = rclcpp::Node::make_shared("cyberdog_inspection_nav2_client");
  client_ = rclcpp_action::create_client<NavigateToPose>(node_, action_name_);

  if (!client_->wait_for_action_server(
      std::chrono::duration<double>(wait_for_server_timeout_sec_)))
  {
    RCLCPP_ERROR(
      node_->get_logger(), "Nav2 action server '%s' not available",
      action_name_.c_str());
    set_status("SERVER_UNAVAILABLE");
    node_.reset();
    client_.reset();
    return false;
  }

  running_ = true;
  spin_thread_ = std::thread(&Nav2Client::spin_loop, this);
  set_status("READY");
  return true;
}

void Nav2Client::stop() {
  if (!running_) {
    return;
  }

  cancel_current_goal();
  running_ = false;
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  client_.reset();
  node_.reset();
}

bool Nav2Client::send_goal(float x, float y, float yaw) {
  if (!client_) {
    return false;
  }

  bool cancel_existing_goal = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (has_active_goal_locked()) {
      if (has_last_goal_ && same_goal(last_goal_x_, last_goal_y_, last_goal_yaw_, x, y, yaw)) {
        return true;
      }
      cancel_existing_goal = true;
    }
    has_last_goal_ = true;
    last_goal_x_ = x;
    last_goal_y_ = y;
    last_goal_yaw_ = yaw;
  }

  if (cancel_existing_goal) {
    cancel_current_goal();
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "Dispatching NavigateToPose goal: x=%.3f y=%.3f yaw=%.3f",
    static_cast<double>(x), static_cast<double>(y), static_cast<double>(yaw));

  NavigateToPose::Goal goal_msg;
  goal_msg.pose.header.frame_id = frame_id_;
  goal_msg.pose.header.stamp = node_->now();
  goal_msg.pose.pose.position.x = static_cast<double>(x);
  goal_msg.pose.pose.position.y = static_cast<double>(y);
  goal_msg.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, static_cast<double>(yaw));
  goal_msg.pose.pose.orientation = tf2::toMsg(q);

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
  options.goal_response_callback =
    [this](const GoalHandle::SharedPtr & goal_handle) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        current_goal_handle_ = goal_handle;
      }
      set_status(goal_handle ? "ACCEPTED" : "REJECTED");
    };
  options.feedback_callback =
    [this](GoalHandle::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback>) {
      set_status("EXECUTING");
    };
  options.result_callback =
    [this](const GoalHandle::WrappedResult & result) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        current_goal_handle_.reset();
        has_last_goal_ = false;
      }
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          set_status("SUCCEEDED");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          set_status("ABORTED");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          set_status("CANCELED");
          break;
        default:
          set_status("UNKNOWN_RESULT");
          break;
      }
    };

  client_->async_send_goal(goal_msg, options);
  set_status("GOAL_SENT");
  return true;
}

void Nav2Client::cancel_current_goal() {
  GoalHandle::SharedPtr goal_handle;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!client_ || !current_goal_handle_) {
      return;
    }
    goal_handle = current_goal_handle_;
    current_goal_handle_.reset();
    has_last_goal_ = false;
  }
  if (!goal_handle) {
    return;
  }
  RCLCPP_INFO(node_->get_logger(), "Canceling current NavigateToPose goal");
  client_->async_cancel_goal(goal_handle);
  set_status("CANCEL_REQUESTED");
}

std::string Nav2Client::last_status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return last_status_;
}

void Nav2Client::spin_loop() {
  while (running_ && rclcpp::ok()) {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

void Nav2Client::set_status(const std::string & status) {
  std::lock_guard<std::mutex> lock(mutex_);
  last_status_ = status;
}

bool Nav2Client::has_active_goal_locked() const
{
  return static_cast<bool>(current_goal_handle_);
}

bool Nav2Client::same_goal(
  float lhs_x, float lhs_y, float lhs_yaw, float rhs_x, float rhs_y, float rhs_yaw)
{
  constexpr float kEpsilon = 1e-3f;
  return std::fabs(lhs_x - rhs_x) < kEpsilon &&
    std::fabs(lhs_y - rhs_y) < kEpsilon &&
    std::fabs(lhs_yaw - rhs_yaw) < kEpsilon;
}

}  // namespace cyberdog_inspection
