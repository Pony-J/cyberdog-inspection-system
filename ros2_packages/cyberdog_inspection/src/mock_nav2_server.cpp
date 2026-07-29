#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace cyberdog_inspection {

class MockNav2Server : public rclcpp::Node {
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  MockNav2Server()
  : Node("cyberdog_inspection_mock_nav2_server")
  {
    const auto action_name = declare_parameter<std::string>("action_name", "navigate_to_pose");
    server_ = rclcpp_action::create_server<NavigateToPose>(
      this,
      action_name,
      std::bind(&MockNav2Server::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MockNav2Server::handle_cancel, this, std::placeholders::_1),
      std::bind(&MockNav2Server::handle_accepted, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Mock Nav2 server listening on action '%s'", action_name.c_str());
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const NavigateToPose::Goal> goal)
  {
    RCLCPP_INFO(
      get_logger(),
      "Received NavigateToPose goal: x=%.3f y=%.3f",
      goal->pose.pose.position.x,
      goal->pose.pose.position.y);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(get_logger(), "Received NavigateToPose cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    rclcpp::Rate rate(2.0);
    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        auto result = std::make_shared<NavigateToPose::Result>();
        goal_handle->canceled(result);
        RCLCPP_INFO(get_logger(), "Goal canceled");
        return;
      }

      auto feedback = std::make_shared<NavigateToPose::Feedback>();
      feedback->distance_remaining = 1.0;
      goal_handle->publish_feedback(feedback);
      rate.sleep();
    }
  }

  rclcpp_action::Server<NavigateToPose>::SharedPtr server_;
};

}  // namespace cyberdog_inspection

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cyberdog_inspection::MockNav2Server>());
  rclcpp::shutdown();
  return 0;
}
