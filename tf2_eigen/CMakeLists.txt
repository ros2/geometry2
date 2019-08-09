cmake_minimum_required(VERSION 3.5)
project(tf2_eigen)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
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
  tf2_ros)
ament_package()
