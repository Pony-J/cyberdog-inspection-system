#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace cyberdog_inspection {

class OriginTfPublisher : public rclcpp::Node {
public:
  OriginTfPublisher()
  : Node("cyberdog_inspection_origin_tf_publisher")
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    robot_frame_ = declare_parameter<std::string>("robot_frame", "base_footprint");
    x_ = declare_parameter<double>("x", 0.0);
    y_ = declare_parameter<double>("y", 0.0);
    z_ = declare_parameter<double>("z", 0.0);
    rate_hz_ = declare_parameter<double>("rate_hz", 10.0);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / std::max(1.0, rate_hz_))),
      std::bind(&OriginTfPublisher::publish_transform, this));

    RCLCPP_INFO(
      get_logger(),
      "Publishing static-like TF %s -> %s at (%.3f, %.3f, %.3f)",
      map_frame_.c_str(), robot_frame_.c_str(), x_, y_, z_);
  }

private:
  void publish_transform()
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = now();
    tf_msg.header.frame_id = map_frame_;
    tf_msg.child_frame_id = robot_frame_;
    tf_msg.transform.translation.x = x_;
    tf_msg.transform.translation.y = y_;
    tf_msg.transform.translation.z = z_;
    tf_msg.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(tf_msg);
  }

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string map_frame_;
  std::string robot_frame_;
  double x_{0.0};
  double y_{0.0};
  double z_{0.0};
  double rate_hz_{10.0};
};

}  // namespace cyberdog_inspection

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cyberdog_inspection::OriginTfPublisher>());
  rclcpp::shutdown();
  return 0;
}
