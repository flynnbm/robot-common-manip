#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include "robot_common_manip/srv/move_to_named_pose.hpp"

class NamedPoseMover : public rclcpp::Node
{
public:
NamedPoseMover()
    : Node("named_pose_mover")
{
    // Declare and read the planning group name parameter
    this->declare_parameter<std::string>("planning_group", "arm");
    
    std::string group_name;
    this->get_parameter("planning_group", group_name);
    
    // Initialize MoveGroupInterface using the current node as context
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){}), group_name);
    
    // Setup the service
    service_ = this->create_service<robot_common_manip::srv::MoveToNamedPose>(
        "move_to_named_pose",
        std::bind(&NamedPoseMover::handle_move_request, this, std::placeholders::_1, std::placeholders::_2)
    );
    
    RCLCPP_INFO(this->get_logger(), "MoveToNamedPose service ready with group: %s", group_name.c_str());
    }

private:
  rclcpp::Service<robot_common_manip::srv::MoveToNamedPose>::SharedPtr service_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  void handle_move_request(
    const std::shared_ptr<robot_common_manip::srv::MoveToNamedPose::Request> req,
    std::shared_ptr<robot_common_manip::srv::MoveToNamedPose::Response> res)
  {
    RCLCPP_INFO(this->get_logger(), "Moving to named target: '%s'", req->target_name.c_str());

    move_group_->setNamedTarget(req->target_name);

    auto result = move_group_->move();
    res->success = (result == moveit::core::MoveItErrorCode::SUCCESS);

    if (res->success)
      RCLCPP_INFO(this->get_logger(), "Motion to '%s' succeeded", req->target_name.c_str());
    else
      RCLCPP_WARN(this->get_logger(), "Motion to '%s' failed", req->target_name.c_str());
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NamedPoseMover>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}