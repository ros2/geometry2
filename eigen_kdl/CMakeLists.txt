cmake_minimum_required(VERSION 3.5)
project(eigen_kdl)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/eigen_kdl.cpp
)

ament_target_dependencies(${PROJECT_NAME}
    orocos_kdl
    eigen3_cmake_module
    Eigen3
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib)

ament_export_include_directories(include)
ament_export_dependencies(
  eigen3_cmake_module
  Eigen3
  orocos_kdl)
ament_export_libraries(${PROJECT_NAME})
ament_package()
