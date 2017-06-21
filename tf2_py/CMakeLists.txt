cmake_minimum_required(VERSION 3.5)
project(tf2_py)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(PythonLibs 3 REQUIRED)
include_directories(${PYTHON_INCLUDE_PATH})

link_libraries(${PYTHON_LIBRARIES})
add_library(${PROJECT_NAME} src/tf2_py.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME tf2 PREFIX "_" SUFFIX ".so")

ament_target_dependencies(${PROJECT_NAME} "rclcpp" "rclpy" "geometry_msgs" "tf2_msgs" "python")
ament_export_include_directories("include")
ament_export_libraries(${PROJECT_NAME})

ament_package()
