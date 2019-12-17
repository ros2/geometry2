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

include_directories(include
   ${orocos_kdl_INCLUDE_DIRS}
   ${tf2_INCLUDE_DIRS}
   ${tf2_ros_INCLUDE_DIRS}
   ${geometry_msgs_INCLUDE_DIRS}
)

link_directories(${orocos_kdl_LIBRARY_DIRS})

ament_python_install_package(${PROJECT_NAME}
     PACKAGE_DIR src/${PROJECT_NAME})

# TODO (ahcorde) Port python once https://github.com/ros2/orocos_kinematics_dynamics/pull/4 is merged
# install(PROGRAMS scripts/test.py
#    DESTINATION lib/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_tf2_geometry_msgs test/test_tf2_geometry_msgs.cpp)
  if(TARGET test_tf2_geometry_msgs)
    target_link_libraries(test_tf2_geometry_msgs
      ${geometry_msgs_LIBRARIES}
      ${orocos_kdl_LIBRARIES}
      ${tf2_LIBRARIES}
      ${tf2_ros_LIBRARIES}
      ${GTEST_LIBRARIES}
    )
  endif()
endif()

ament_auto_package()
