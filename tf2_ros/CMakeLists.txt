cmake_minimum_required(VERSION 3.5)
project(tf2_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT ANDROID)
  set(TF2_PY tf2_py)
endif()

find_package(ament_cmake REQUIRED)
# TODO(tfoote) uncomment when reenabling server client interfaces
# find_package(actionlib REQUIRED)
# find_package(actionlib_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)

include_directories(include
  ${geometry_msgs_INCLUDE_DIRS}
  ${message_filters_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_msgs_INCLUDE_DIRS}
  )

# tf2_ros library
add_library(${PROJECT_NAME} SHARED
  src/buffer.cpp
  src/transform_listener.cpp
  # src/buffer_client.cpp
  # src/buffer_server.cpp
  src/transform_broadcaster.cpp
  src/static_transform_broadcaster.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${geometry_msgs_LIBRARIES}
  ${message_filters_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_msgs_LIBRARIES}
  )

target_compile_definitions(${PROJECT_NAME} PRIVATE "TF2_ROS_BUILDING_DLL")

# TODO(tfoote) port server client interfaces
# # buffer_server executable
# add_executable(buffer_server src/buffer_server_main.cpp)
# add_dependencies(${PROJECT_NAME}_buffer_server tf2_msgs_gencpp)
# target_link_libraries(${PROJECT_NAME}_buffer_server
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )


# static_transform_publisher
add_executable(static_transform_publisher
  src/static_transform_broadcaster_program.cpp
)
target_link_libraries(static_transform_publisher
  ${PROJECT_NAME}
  ${geometry_msgs_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_msgs_LIBRARIES}
  )

add_executable(tf2_echo
  src/tf2_echo.cpp
)
target_link_libraries(tf2_echo
  ${PROJECT_NAME}
  ${geometry_msgs_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_msgs_LIBRARIES}
  )

  add_executable(tf2_monitor
    src/tf2_monitor.cpp
  )
  target_link_libraries(tf2_monitor
    ${PROJECT_NAME}
    ${geometry_msgs_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${tf2_LIBRARIES}
    ${tf2_msgs_LIBRARIES}
    )

# Install rules
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install executables
install(TARGETS
  # buffer_server
  static_transform_publisher
  tf2_echo
  tf2_monitor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)


# Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_buffer test/test_buffer.cpp)
  target_link_libraries(test_buffer ${PROJECT_NAME})

  # Adds a tf2_ros message_filter unittest that uses
  # multiple target frames and a non-zero time tolerance
  ament_add_gtest(${PROJECT_NAME}_test_message_filter test/message_filter_test.cpp)
  target_link_libraries(${PROJECT_NAME}_test_message_filter ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_test_transform_listener test/test_transform_listener.cpp)
  target_link_libraries(${PROJECT_NAME}_test_transform_listener ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_test_static_transform_broadcaster test/test_static_transform_broadcaster.cpp)
  target_link_libraries(${PROJECT_NAME}_test_static_transform_broadcaster ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_test_transform_broadcaster test/test_transform_broadcaster.cpp)
  target_link_libraries(${PROJECT_NAME}_test_transform_broadcaster ${PROJECT_NAME})
# TODO(tfoote) port tests to use ROS2 instead of ROS1 api.
if (false)

# new requirements for testing
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_filters
  roscpp
  rosgraph
  rospy
  rostest
  tf2
  tf2_msgs
  ${TF2_PY}
)

# tf2_ros_test_listener
add_executable(${PROJECT_NAME}_test_listener EXCLUDE_FROM_ALL test/listener_unittest.cpp)
add_dependencies(${PROJECT_NAME}_test_listener tf2_msgs_gencpp)
add_executable(${PROJECT_NAME}_test_time_reset EXCLUDE_FROM_ALL test/time_reset_test.cpp)
add_dependencies(${PROJECT_NAME}_test_time_reset tf2_msgs_gencpp)
target_link_libraries(${PROJECT_NAME}_test_listener
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_test_time_reset
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}
)

add_dependencies(tests ${PROJECT_NAME}_test_listener)
add_dependencies(tests ${PROJECT_NAME}_test_time_reset)

add_rostest(test/transform_listener_unittest.launch)
add_rostest(test/transform_listener_time_reset_test.launch)

endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp)
ament_export_dependencies(tf2_msgs)
ament_export_dependencies(tf2)
ament_package()
