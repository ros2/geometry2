cmake_minimum_required(VERSION 3.5)
project(tf2_eigen)

find_package(ament_cmake REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

include_directories(include)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME})

ament_export_include_directories(include)
ament_package()
