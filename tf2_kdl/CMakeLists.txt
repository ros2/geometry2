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
find_package(orocos_kdl REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)

# ament_python_install_package(${PROJECT_NAME}
#     PACKAGE_DIR src/${PROJECT_NAME})

link_directories(${orocos_kdl_LIBRARY_DIRS})
include_directories(include ${rclpy_INCLUDE_DIRS}
                    ${rmw_implementation_INCLUDE_DIRS}
                    ${std_msgs_INCLUDE_DIRS}
                    ${Eigen3_INCLUDE_DIRS}
                    ${GTEST_INCLUDE_DIRS})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/${PROJECT_NAME}
)

install(PROGRAMS scripts/test.py
         DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package (Threads)
  find_package(Eigen3 REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(tf2_msgs REQUIRED)
  
  ament_add_gtest(test_kdl test/test_tf2_kdl.cpp)
  target_include_directories(test_kdl PUBLIC
                                include
                                ${tf2_INCLUDE_DIRS}
                                ${tf2_ros_INCLUDE_DIRS}
                                ${EIGEN3_INCLUDE_DIRS}
                                ${rclcpp_INCLUDE_DIRS}
                                ${orocos_kdl_INCLUDE_DIRS}
  )
    
  target_link_libraries(test_kdl ${rclpy_LIBRARIES}
  		                          ${rclcpp_LIBRARIES}
                                ${rmw_implementation_LIBRARIES}
                                ${GTEST_LIBRARIES}
                                ${orocos_kdl_LIBRARIES}
                                ${CMAKE_THREAD_LIBS_INIT})

  ament_target_dependencies(test_kdl
    tf2
    tf2_ros
    tf2_msgs)    
endif()

ament_export_include_directories(include ${Eigen3_INCLUDE_DIRS})
ament_export_dependencies(Eigen3 orocos_kdl)
ament_package()
