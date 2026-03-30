cmake_minimum_required(VERSION 3.5)
project(tf2_sensor_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

<<<<<<< HEAD
find_package(ament_cmake_auto REQUIRED)
set(required_dependencies
  "sensor_msgs"
  "tf2"
  "tf2_ros"
)
=======
# By default, without the settings below, find_package(Python3) will attempt
# to find the newest python version it can, and additionally will find the
# most specific version.  For instance, on a system that has
# /usr/bin/python3.10, /usr/bin/python3.11, and /usr/bin/python3, it will find
# /usr/bin/python3.11, even if /usr/bin/python3 points to /usr/bin/python3.10.
# The behavior we want is to prefer the "system" installed version unless the
# user specifically tells us otherwise through the Python3_EXECUTABLE hint.
# Setting CMP0094 to NEW means that the search will stop after the first
# python version is found.  Setting Python3_FIND_UNVERSIONED_NAMES means that
# the search will prefer /usr/bin/python3 over /usr/bin/python3.11.  And that
# latter functionality is only available in CMake 3.20 or later, so we need
# at least that version.
cmake_policy(SET CMP0094 NEW)
set(Python3_FIND_UNVERSIONED_NAMES FIRST)
>>>>>>> febd947 (fix typos (#921))

ament_auto_find_build_dependencies(REQUIRED ${required_dependencies})

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)
ament_export_dependencies(tf2)
ament_export_dependencies(tf2_ros)

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  # TODO(ros2/geometry2#259) Remove once headers
  # are renamed to .hpp
  set(ament_cmake_uncrustify_ADDITIONAL_ARGS --language CPP)
  set(ament_cmake_cppcheck_LANGUAGE c++)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_tf2_sensor_msgs_py test/test_tf2_sensor_msgs.py)

  ament_add_gtest(test_tf2_sensor_msgs_cpp test/test_tf2_sensor_msgs.cpp)
  if(TARGET test_tf2_sensor_msgs_cpp)
    target_include_directories(test_tf2_sensor_msgs_cpp PUBLIC include)
    ament_target_dependencies(test_tf2_sensor_msgs_cpp
      "Eigen3"
      "rclcpp"
      "sensor_msgs"
      "tf2"
      "tf2_ros"
    )
  endif()
endif()

ament_auto_package()
