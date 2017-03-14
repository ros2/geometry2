cmake_minimum_required(VERSION 3.5)
project(tf2_geometry_msgs)

if(NOT WIN32)
  add_definitions(-std=c++14)
endif()

find_package(orocos_kdl)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)

# Issue #53
find_library(KDL_LIBRARY REQUIRED NAMES orocos-kdl HINTS ${orocos_kdl_LIBRARY_DIRS})

#catkin_package(
#   LIBRARIES ${KDL_LIBRARY}
#   INCLUDE_DIRS include
#   DEPENDS orocos_kdl
#   CATKIN_DEPENDS  geometry_msgs tf2_ros tf2)


include_directories(include
   ${tf2_INCLUDE_DIRS}
   ${tf2_ros_INCLUDE_DIRS}
   ${geometry_msgs_INCLUDE_DIRS}
)

link_directories(${orocos_kdl_LIBRARY_DIRS})



install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

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
ament_export_include_directories(include)
ament_package()
