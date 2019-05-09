cmake_minimum_required(VERSION 3.5)
project(tf2_kdl)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# TODO(clalancette): Port python once https://github.com/ros2/geometry2/pull/99 is merged
# ament_python_install_package(${PROJECT_NAME}
#      PACKAGE_DIR src/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}
)

# TODO(clalancette): Port python tests once https://github.com/ros2/geometry/pull/99 is merged
# install(PROGRAMS scripts/test.py
#     DESTINATION lib/${PROJECT_NAME}
# )


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(tf2_msgs REQUIRED)

  ament_add_gtest(test_kdl test/test_tf2_kdl.cpp)
  target_include_directories(test_kdl PUBLIC
    include
  )
  ament_target_dependencies(test_kdl
    builtin_interfaces
    orocos_kdl
    rclcpp
    tf2
    tf2_ros
    tf2_msgs)
endif()

ament_export_include_directories(include)
ament_export_dependencies(builtin_interfaces Eigen3 geometry_msgs orocos_kdl tf2 tf2_ros)
ament_package()
