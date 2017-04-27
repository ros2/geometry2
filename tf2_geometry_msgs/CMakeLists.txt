cmake_minimum_required(VERSION 3.5)
project(tf2_geometry_msgs)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")
endif()

find_package(ament_cmake_auto REQUIRED)
set(required_dependencies
  "geometry_msgs"
  "orocos_kdl"
  "tf2"
  "tf2_ros"
)
ament_auto_find_build_dependencies(REQUIRED ${required_dependencies})

include_directories(include
   ${orocos_kdl_INCLUDE_DIRS}
   ${tf2_INCLUDE_DIRS}
   ${tf2_ros_INCLUDE_DIRS}
   ${geometry_msgs_INCLUDE_DIRS}
)

link_directories(${orocos_kdl_LIBRARY_DIRS})


# TODO(dhood): enable python support once ported to ROS 2
# catkin_python_setup()

# TODO(dhood): enable tests once ported to ROS 2
# if(CATKIN_ENABLE_TESTING)
#
# find_package(catkin REQUIRED COMPONENTS geometry_msgs rostest tf2_ros tf2)
#
# add_executable(test_geometry_msgs EXCLUDE_FROM_ALL test/test_tf2_geometry_msgs.cpp)
# target_link_libraries(test_geometry_msgs ${catkin_LIBRARIES} ${GTEST_LIBRARIES} ${orocos_kdl_LIBRARIES})
# add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test.launch)
# add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_python.launch)
#
#
# if(TARGET tests)
#   add_dependencies(tests test_geometry_msgs)
# endif()
#
#
# endif()
ament_auto_package()
