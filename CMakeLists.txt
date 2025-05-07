cmake_minimum_required(VERSION 3.8)
project(quadruped_skate_mpc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


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

install(DIRECTORY
  a1_description/
  DESTINATION share/${PROJECT_NAME}/a1_description
)

install(DIRECTORY
  include
  launch
  scripts
  src
  msg
  DESTINATION share/${PROJECT_NAME}
)

include_directories(
  include
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/States.msg"
  "msg/FootPosition.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

add_executable(foot_trajectory_planner_node
  src/foot_trajectory_planner_node.cpp
  src/foot_trajectory_planner.cpp
)

ament_target_dependencies(foot_trajectory_planner_node
  rclcpp
  std_msgs
  geometry_msgs
)

install(TARGETS
  foot_trajectory_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

rosidl_target_interfaces(foot_trajectory_planner_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)



ament_export_dependencies(rosidl_default_runtime)


ament_package()
