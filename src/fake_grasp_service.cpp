#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <robot_common_manip/srv/fake_grasp.hpp>
#include <random>

// TF2 Includes
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class FakeGraspPlanner : public rclcpp::Node
{
public:
  FakeGraspPlanner()
    : Node("fake_grasp_planner"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
  {
    this->declare_parameter<std::string>("planning_group", "arm");

    std::string group_name;
    this->get_parameter("planning_group", group_name);

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){}), group_name);

    service_ = this->create_service<robot_common_manip::srv::FakeGrasp>(
      "fake_grasp_pose",
      std::bind(&FakeGraspPlanner::handle_request, this, std::placeholders::_1, std::placeholders::_2)
    );

    RCLCPP_INFO(this->get_logger(), "FakeGraspPlanner service ready (group: %s)", group_name.c_str());
  }

private:
  rclcpp::Service<robot_common_manip::srv::FakeGrasp>::SharedPtr service_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  void handle_request(
    const std::shared_ptr<robot_common_manip::srv::FakeGrasp::Request> /*req*/,
    std::shared_ptr<robot_common_manip::srv::FakeGrasp::Response> res)
  {
    geometry_msgs::msg::Pose current_pose;

    try
    {
      // Replace "base_link" with your robot's base frame if different
      geometry_msgs::msg::TransformStamped transformStamped =
        tf_buffer_.lookupTransform("panda_link0", "panda_tcp", tf2::TimePointZero);

      current_pose.position.x = transformStamped.transform.translation.x;
      current_pose.position.y = transformStamped.transform.translation.y;
      current_pose.position.z = transformStamped.transform.translation.z;
      current_pose.orientation = transformStamped.transform.rotation;

      // Add small random perturbation
      std::random_device rd;
      std::mt19937 gen(rd());
      std::uniform_real_distribution<> dist(-0.02, 0.02);  // +/- 2 cm

      current_pose.position.x += dist(gen);
      current_pose.position.y += dist(gen);
      current_pose.position.z += dist(gen);

      res->grasp_pose = current_pose;

      RCLCPP_INFO(this->get_logger(), "Fake grasp pose (from TF) generated and returned.");
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
      // Optionally return a default pose or error code
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FakeGraspPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}