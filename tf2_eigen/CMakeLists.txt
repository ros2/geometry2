cmake_minimum_required(VERSION 3.5)
project(tf2_eigen)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Work around broken find module in AlmaLinux/RHEL eigen3-devel from PowerTools repo
find_package(Eigen3 QUIET NO_MODULE)
if(NOT Eigen3_FOUND)
  find_package(Eigen3 REQUIRED)
endif()

add_library(tf2_eigen INTERFACE)
target_link_libraries(tf2_eigen INTERFACE
  ${geometry_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros)
target_include_directories(tf2_eigen INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
if(TARGET Eigen3::Eigen)
  # TODO(sloretz) require target to exist when https://github.com/ros2/choco-packages/issues/19 is addressed
  target_link_libraries(tf2_eigen INTERFACE Eigen3::Eigen)
else()
  target_include_directories(tf2_eigen INTERFACE ${Eigen3_INCLUDE_DIRS})
endif()

install(TARGETS tf2_eigen EXPORT export_tf2_eigen)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)

  ament_cppcheck(LANGUAGE "c++")
  ament_cpplint()
  ament_lint_cmake()
  ament_uncrustify(LANGUAGE "c++")

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(tf2_eigen-test test/tf2_eigen-test.cpp)
  if(TARGET tf2_eigen-test)
    target_link_libraries(tf2_eigen-test
      tf2_eigen
      # Used, but not linked to test tf2_eigen's exports:
      #   Eigen3::Eigen
      #   ${geometry_msgs_TARGETS}
      #   tf2::tf2
      #   tf2_ros::tf2_ros
    )
  endif()
endif()

ament_export_dependencies(
  Eigen3
  tf2
  tf2_ros)

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")

# Export modern CMake targets
ament_export_targets(export_tf2_eigen)

ament_package()
