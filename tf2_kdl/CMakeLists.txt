cmake_minimum_required(VERSION 3.5)
project(tf2_kdl)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

ament_python_install_package(${PROJECT_NAME}
     PACKAGE_DIR src/${PROJECT_NAME})

add_library(tf2_kdl INTERFACE)
target_link_libraries(tf2_kdl INTERFACE
  ${builtin_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  orocos-kdl
  tf2::tf2
  tf2_ros::tf2_ros)
target_include_directories(tf2_kdl INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

install(TARGETS tf2_kdl EXPORT export_tf2_kdl)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

# TODO(ahcorde): Port python once https://github.com/ros2/orocos_kinematics_dynamics/pull/4 is merged
# install(PROGRAMS scripts/test.py
#     DESTINATION lib/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rclcpp REQUIRED)

  ament_add_gtest(test_kdl test/test_tf2_kdl.cpp)
  target_link_libraries(test_kdl
    tf2_kdl
    rclcpp::rclcpp
    # Used, but not linked to test tf2_kdl's exports:
    #   tf2::tf2
    #   tf2_ros::tf2_ros
  )
endif()

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")

# Export modern CMake targets
ament_export_targets(export_tf2_kdl)

ament_export_dependencies(builtin_interfaces geometry_msgs orocos_kdl_vendor orocos_kdl tf2 tf2_ros)

ament_package()
