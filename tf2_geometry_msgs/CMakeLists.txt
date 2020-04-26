cmake_minimum_required(VERSION 3.5)
project(tf2_geometry_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake_auto REQUIRED)
set(required_dependencies
  "geometry_msgs"
  "orocos_kdl"
  "tf2"
  "tf2_ros"
)
ament_auto_find_build_dependencies(REQUIRED ${required_dependencies})

ament_python_install_package(${PROJECT_NAME}
     PACKAGE_DIR src/${PROJECT_NAME})

# TODO (ahcorde) Port python once https://github.com/ros2/orocos_kinematics_dynamics/pull/4 is merged
# install(PROGRAMS scripts/test.py
#    DESTINATION lib/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rclcpp REQUIRED)
  ament_add_gtest(test_tf2_geometry_msgs test/test_tf2_geometry_msgs.cpp)
  if(TARGET test_tf2_geometry_msgs)
    target_include_directories(test_tf2_geometry_msgs PUBLIC include)
    ament_target_dependencies(test_tf2_geometry_msgs
      "geometry_msgs"
      "orocos_kdl"
      "rclcpp"
      "tf2"
      "tf2_ros"
    )
  endif()
endif()

ament_auto_package()
