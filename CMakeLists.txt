cmake_minimum_required(VERSION 3.8)
project(robot_common_manip)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CartesianMoveToPose.srv"
  "srv/MoveToNamedPose.srv"
  "srv/MoveToPose.srv"
  DEPENDENCIES geometry_msgs
)

# Create executable
add_executable(cartesian_move_to_pose_service src/cartesian_move_to_pose_service.cpp)
ament_target_dependencies(cartesian_move_to_pose_service
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
)

add_executable(move_to_named_pose_service src/move_to_named_pose_service.cpp)
ament_target_dependencies(move_to_named_pose_service
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
)

add_executable(move_to_pose_service src/move_to_pose_service.cpp)
ament_target_dependencies(move_to_pose_service
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
)

# Link generated service headers
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(cartesian_move_to_pose_service ${cpp_typesupport_target})
target_link_libraries(move_to_named_pose_service ${cpp_typesupport_target})
target_link_libraries(move_to_pose_service ${cpp_typesupport_target})

install(TARGETS 
  cartesian_move_to_pose_service
  move_to_named_pose_service
  move_to_pose_service
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
