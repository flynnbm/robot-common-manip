#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include "robot_common_manip/srv/move_to_pose.hpp"

class PosePlanner : public rclcpp::Node
{
public:
  PosePlanner()
    : Node("move_to_pose_service")
  {
    this->declare_parameter<std::string>("planning_group", "arm");

    std::string group_name;
    this->get_parameter("planning_group", group_name);

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){}), group_name);

    service_ = this->create_service<robot_common_manip::srv::MoveToPose>(
      "move_to_pose",
      std::bind(&PosePlanner::handle_request, this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(this->get_logger(), "MoveToPose service ready (group: %s)", group_name.c_str());
  }

private:
  rclcpp::Service<robot_common_manip::srv::MoveToPose>::SharedPtr service_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  void handle_request(
    const std::shared_ptr<robot_common_manip::srv::MoveToPose::Request> req,
    std::shared_ptr<robot_common_manip::srv::MoveToPose::Response> res)
  {
    move_group_->setPoseTarget(req->target_pose);

    auto result = move_group_->move();
    res->success = (result == moveit::core::MoveItErrorCode::SUCCESS);

    if (res->success)
      RCLCPP_INFO(this->get_logger(), "Motion to pose succeeded.");
    else
      RCLCPP_WARN(this->get_logger(), "Motion to pose failed.");
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PosePlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}