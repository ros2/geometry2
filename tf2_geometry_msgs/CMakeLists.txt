cmake_minimum_required(VERSION 3.5)
project(tf2_geometry_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rclcpp REQUIRED)
<<<<<<< HEAD
=======

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_tf2_geometry_msgs_py test/test_tf2_geometry_msgs.py)

>>>>>>> 6b18a400 (Drop PyKDL dependency in tf2_geometry_msgs (#509))
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

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} INTERFACE ${required_dependencies})

install(
  TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
  INCLUDES DESTINATION include
)

ament_export_targets(${PROJECT_NAME})
ament_export_dependencies(${required_dependencies})

ament_auto_package()
