cmake_minimum_required(VERSION 3.5)
project(tf2_eigen)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)

  # The version of Eigen in Ubuntu 22.04 has a bug that causes Wmem-access warnings on aarch64.
  # Upstream has already fixed this in https://gitlab.com/libeigen/eigen/-/merge_requests/645 .
  # There is a fix in progress for this in Debian: https://salsa.debian.org/science-team/eigen3/-/merge_requests/1
  # However, it is not clear that that fix is going to make it into Ubuntu 22.04 before it
  # freezes, so disable the warning here.
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    add_compile_options(-Wno-mem-access)
  endif()
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME})

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
    target_include_directories(tf2_eigen-test PUBLIC
      include)
    ament_target_dependencies(tf2_eigen-test
      "tf2"
      "tf2_ros")
    target_include_directories(tf2_eigen-test PUBLIC
      ${Eigen3_INCLUDE_DIRS})
  endif()
endif()

ament_export_include_directories(include)
ament_export_dependencies(
  eigen3_cmake_module
  Eigen3
  tf2
  tf2_ros)
ament_package()
