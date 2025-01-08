cmake_minimum_required(VERSION 3.8)
project(kinova_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(gripper_node src/gripper_test.cpp)
add_executable(home_position src/home.cpp)
add_executable(joint_node src/joint_test.cpp)
add_executable(joint_mode_node src/joint_mode.cpp)
add_executable(twist_node src/twist_test.cpp)
add_executable(twist_mode_node src/twist_mode.cpp)
add_executable(packaging_position src/packaging.cpp)
add_executable(zero_position src/zero.cpp)

ament_target_dependencies(gripper_node rclcpp rclcpp_action control_msgs)
ament_target_dependencies(home_position rclcpp trajectory_msgs)
ament_target_dependencies(joint_node rclcpp trajectory_msgs)
ament_target_dependencies(joint_mode_node rclcpp controller_manager_msgs)
ament_target_dependencies(twist_node rclcpp geometry_msgs)
ament_target_dependencies(twist_mode_node rclcpp controller_manager_msgs)
ament_target_dependencies(packaging_position rclcpp rclcpp_action control_msgs trajectory_msgs)
ament_target_dependencies(zero_position rclcpp trajectory_msgs)

install(TARGETS
  gripper_node
  home_position
  joint_node
  joint_mode_node
  twist_node
  twist_mode_node
  packaging_position
  zero_position
  DESTINATION lib/${PROJECT_NAME}
)

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
