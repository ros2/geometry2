cmake_minimum_required(VERSION 3.5)
project(tf2_sensor_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake_auto REQUIRED)
set(required_dependencies
  "sensor_msgs"
  #"python_orocos_kdl"
  "tf2"
  "tf2_ros"
)

ament_auto_find_build_dependencies(REQUIRED ${required_dependencies})

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

# TODO enable tests
#if(BUILD_TESTING)
#  catkin_add_nosetests(test/test_tf2_sensor_msgs.py)

#find_package(catkin REQUIRED COMPONENTS
#  sensor_msgs
#  rostest
#  tf2_ros
#  tf2
#)
#include_directories(${EIGEN_INCLUDE_DIRS})
#add_executable(test_tf2_sensor_msgs_cpp EXCLUDE_FROM_ALL test/test_tf2_sensor_msgs.cpp)
#target_link_libraries(test_tf2_sensor_msgs_cpp ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
#if(TARGET tests)
#  add_dependencies(tests test_tf2_sensor_msgs_cpp)
#endif()
#add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test.launch)
#endif()

ament_auto_package()
