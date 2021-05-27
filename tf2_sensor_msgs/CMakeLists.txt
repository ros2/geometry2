cmake_minimum_required(VERSION 3.5)
project(tf2_sensor_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

find_package(ament_cmake_auto REQUIRED)
set(required_dependencies
  "sensor_msgs"
  #"python_orocos_kdl"
  "tf2"
  "tf2_ros"
)

ament_auto_find_build_dependencies(REQUIRED ${required_dependencies})

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)
ament_export_dependencies(tf2)
ament_export_dependencies(tf2_ros)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  # TODO(ros2/geometry2#259) Remove once headers
  # are renamed to .hpp
  set(ament_cmake_uncrustify_ADDITIONAL_ARGS --language CPP)
  set(ament_cmake_cppcheck_LANGUAGE c++)
  ament_lint_auto_find_test_dependencies()

  # TODO(ros2/orocos_kinematics_dynamics): reenable when PyKDL is ready to use
  #find_package(ament_cmake_pytest REQUIRED)
  #ament_add_pytest_test(test_tf2_sensor_msgs_py test/test_tf2_sensor_msgs.py)

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
