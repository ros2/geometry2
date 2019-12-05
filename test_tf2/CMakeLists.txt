cmake_minimum_required(VERSION 3.5)

if(NOT BUILD_TESTING)
  return()
endif()

project(test_tf2)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_kdl REQUIRED)

ament_add_gtest(buffer_core_test test/buffer_core_test.cpp)
if(TARGET buffer_core_test)
  ament_target_dependencies(buffer_core_test
    tf2
    rclcpp
  )
endif()

ament_add_gtest(test_message_filter test/test_message_filter.cpp)
if(TARGET test_message_filter)
  ament_target_dependencies(test_message_filter
    tf2
    tf2_ros
    rclcpp
    geometry_msgs
  )
endif()

ament_add_gtest(test_utils test/test_utils.cpp)
if(TARGET test_utils)
  ament_target_dependencies(test_utils
    tf2
    tf2_kdl
  )
endif()

add_executable(test_buffer_server test/test_buffer_server.cpp)
ament_target_dependencies(test_buffer_server
    rclcpp
    rclcpp_action
    tf2_ros
  )

# add_executable(test_buffer_client EXCLUDE_FROM_ALL test/test_buffer_client.cpp)
# target_link_libraries(test_buffer_client ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${GTEST_LIBRARIES} ${orocos_kdl_LIBRARIES})
#
#
# add_rostest(test/buffer_client_tester.launch)
#
# add_executable(test_static_publisher EXCLUDE_FROM_ALL test/test_static_publisher.cpp)
# target_link_libraries(test_static_publisher ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
#
# add_rostest(test/static_publisher.launch)
#
#
# add_executable(test_tf2_bullet EXCLUDE_FROM_ALL test/test_tf2_bullet.cpp)
# target_link_libraries(test_tf2_bullet ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
#
# add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_tf2_bullet.launch)
#
#
# if(TARGET tests)
#   add_dependencies(tests test_buffer_server test_buffer_client test_static_publisher test_tf2_bullet)
# endif()
#
#
# # used as a test fixture
# if(TARGET tf2_ros_static_transform_publisher)
#   add_dependencies(tests tf2_ros_static_transform_publisher test_static_publisher)
# endif()

ament_package()
