cmake_minimum_required(VERSION 3.8)
project(a1_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  meshes/
  assets/
  DESTINATION share/${PROJECT_NAME}/
)

install(FILES
  a1.xml
  scene.xml
  skate.xml
  a1.png
  DESTINATION share/${PROJECT_NAME}
)

ament_package()