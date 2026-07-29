#include "communication/ros2/pose_provider.hpp"

#include <chrono>
#include <cstdlib>
#include <functional>

#include <tf2/exceptions.h>

namespace communication::ros2 {

PoseProvider::PoseProvider(const std::string & map_frame, const std::string & robot_frame)
{
  const char * env_ns = std::getenv("CYBERDOG_NS");
  ns_ = (env_ns != nullptr) ? std::string(env_ns) : "";
  // The current cyberdog_nav2_lidar stack exposes a Jetson-side cleaned TF tree:
  // map -> odom_fixed -> base_footprint_fixed -> base_link_fixed -> laser_frame_fixed.
  // Only CyberDog-specific topics like /<ns>/odom_out and /<ns>/body_cmd remain namespaced.
  map_frame_ = map_frame;
  robot_frame_ = robot_frame;
}

PoseProvider::~PoseProvider()
{
  stop();
}

bool PoseProvider::start()
{
  if (running_) {
    return false;
  }

  try {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = rclcpp::Node::make_shared("cyberdog_inspection_pose_provider");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&PoseProvider::update_pose, this));
    running_ = true;
    spin_thread_ = std::thread(&PoseProvider::spin_thread, this);
    return true;
  } catch (const std::exception & e) {
    std::cerr << "[PoseProvider] Start failed: " << e.what() << std::endl;
    return false;
  }
}

void PoseProvider::stop()
{
  if (!running_) {
    return;
  }
  running_ = false;
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  timer_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  node_.reset();
}

void PoseProvider::spin_thread()
{
  while (running_ && rclcpp::ok()) {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void PoseProvider::update_pose()
{
  try {
    auto transform = tf_buffer_->lookupTransform(map_frame_, robot_frame_, tf2::TimePointZero);
    std::unique_lock<std::shared_mutex> lock(pose_mutex_);
    current_pose_.x = static_cast<float>(transform.transform.translation.x);
    current_pose_.y = static_cast<float>(transform.transform.translation.y);
    current_pose_.valid = true;
  } catch (const tf2::TransformException & e) {
    std::unique_lock<std::shared_mutex> lock(pose_mutex_);
    current_pose_.valid = false;
    if (node_) {
      RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 5000,
        "TF failed: %s -> %s: %s",
        robot_frame_.c_str(), map_frame_.c_str(), e.what());
    }
  }
}

Pose2D PoseProvider::get_pose()
{
  std::shared_lock<std::shared_mutex> lock(pose_mutex_);
  return current_pose_;
}

bool PoseProvider::is_valid()
{
  std::shared_lock<std::shared_mutex> lock(pose_mutex_);
  return current_pose_.valid;
}

}  // namespace communication::ros2
