cmake_minimum_required(VERSION 3.5)
project(tf2_tools)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
  scripts/view_frames.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
