cmake_minimum_required(VERSION 3.5)
project(tf2_eigen)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIR "${EIGEN3_INCLUDE_DIR}")
message("Using Eigen3 include dirs: ${Eigen_INCLUDE_DIR}")

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(tf2_eigen-test test/tf2_eigen-test.cpp)
  if(TARGET tf2_eigen-test)
    target_include_directories(tf2_eigen-test PUBLIC
      include
      ${tf2_INCLUDE_DIRS}
      ${tf2_ros_INCLUDE_DIRS}
      ${Eigen_INCLUDE_DIR}
    )
  endif()
endif()

ament_export_include_directories(include)
ament_export_dependencies(tf2_ros)
ament_package()
