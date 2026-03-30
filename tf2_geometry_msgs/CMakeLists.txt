cmake_minimum_required(VERSION 3.5)
project(tf2_geometry_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

# Figure out Python3 debug/release before anything else can find_package it
if(WIN32 AND CMAKE_BUILD_TYPE STREQUAL "Debug")
  find_package(python_cmake_module REQUIRED)
  find_package(PythonExtra REQUIRED)
  # Force FindPython3 to use the debug interpretter where ROS 2 expects it
  set(Python3_EXECUTABLE "${PYTHON_EXECUTABLE_DEBUG}")
elseif(NOT WIN32 AND NOT Python3_EXECUTABLE)
  # We expect that Python dependencies are met for whatever Python interpreter
  # is invoked by the "python3" executable, however this may not be the latest
  # Python version installed on the system. The default behavior of
  # find_package(Python3) is to use the latest version (e.x. python3.12), so we
  # specifically look for a "python3" executable and if found, instruct
  # find_package(Python3) to use that.
  # On Windows, the find_package(Python3) logic is different and doesn't
  # appear to prefer specific versions (e.x. python3.12) over plain
  # "python.exe" so this extra logic is unnecessary there.
  find_program(Python3_EXECUTABLE python3)
endif()
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

find_package(geometry_msgs REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR src/${PROJECT_NAME})

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} INTERFACE
  ${geometry_msgs_TARGETS}
  orocos-kdl
  tf2::tf2
  tf2_ros::tf2_ros)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_cppcheck
  )
  ament_lint_auto_find_test_dependencies()

  ament_cppcheck(LANGUAGE "c++")

  find_package(ament_cmake_gtest REQUIRED)
  find_package(rclcpp REQUIRED)

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_tf2_geometry_msgs_py test/test_tf2_geometry_msgs.py)

  ament_add_gtest(test_tf2_geometry_msgs test/test_tf2_geometry_msgs.cpp)
  if(TARGET test_tf2_geometry_msgs)
    target_link_libraries(test_tf2_geometry_msgs
      ${PROJECT_NAME}
      rclcpp::rclcpp
      # Used, but not linked to test ${PROJECT_NAME}'s exports:
      #   tf2_ros::tf2_ros
    )
  endif()
endif()

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  "geometry_msgs"
  "orocos_kdl_vendor"
  "orocos_kdl"
  "tf2"
  "tf2_ros")

ament_package()
