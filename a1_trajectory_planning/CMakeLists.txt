cmake_minimum_required(VERSION 3.8)
project(a1_trajectory_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(a1_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  include
)

add_executable(foot_trajectory_planner_node
  src/foot_trajectory_planner_node.cpp
  src/foot_trajectory_planner.cpp
)

ament_target_dependencies(foot_trajectory_planner_node
  rclcpp
  std_msgs
  geometry_msgs
  a1_msgs
)

install(TARGETS
  foot_trajectory_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()