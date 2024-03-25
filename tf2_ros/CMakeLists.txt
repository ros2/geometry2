cmake_minimum_required(VERSION 3.5)
project(tf2_ros)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)

# tf2_ros library
add_library(${PROJECT_NAME} SHARED
  src/buffer.cpp
  src/create_timer_ros.cpp
  src/transform_listener.cpp
  src/buffer_client.cpp
  src/buffer_server.cpp
  src/transform_broadcaster.cpp
  src/static_transform_broadcaster.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${builtin_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  message_filters::message_filters
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  tf2::tf2
  ${tf2_msgs_TARGETS})

target_compile_definitions(${PROJECT_NAME} PRIVATE "TF2_ROS_BUILDING_DLL")

# buffer_server executable
add_executable(buffer_server src/buffer_server_main.cpp)
target_link_libraries(buffer_server
  ${PROJECT_NAME}
  rclcpp::rclcpp
)

add_library(static_transform_broadcaster_node SHARED
  src/static_transform_broadcaster_node.cpp
)
target_compile_definitions(static_transform_broadcaster_node PRIVATE "STATIC_TRANSFORM_BROADCASTER_BUILDING_DLL")
target_include_directories(static_transform_broadcaster_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(static_transform_broadcaster_node PUBLIC
  ${PROJECT_NAME}
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
  ${tf2_msgs_TARGETS})
target_link_libraries(static_transform_broadcaster_node PRIVATE
  rclcpp_components::component)
rclcpp_components_register_nodes(static_transform_broadcaster_node "tf2_ros::StaticTransformBroadcasterNode")

# static_transform_publisher
add_executable(static_transform_publisher
  src/static_transform_broadcaster_program.cpp
)
target_link_libraries(static_transform_publisher
  ${PROJECT_NAME}
  rclcpp::rclcpp
  static_transform_broadcaster_node
  tf2::tf2
)

add_executable(tf2_echo
  src/tf2_echo.cpp
)
target_link_libraries(tf2_echo
  ${PROJECT_NAME}
  rclcpp::rclcpp
)

add_executable(tf2_monitor
  src/tf2_monitor.cpp
)
target_link_libraries(tf2_monitor
  ${PROJECT_NAME}
  rclcpp::rclcpp
  ${tf2_msgs_TARGETS}
)

# Install rules
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS static_transform_broadcaster_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install executables
install(TARGETS
  buffer_server
  static_transform_publisher
  tf2_echo
  tf2_monitor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

# Tests
if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(rosgraph_msgs REQUIRED)

  ament_cppcheck(LANGUAGE "c++")
  ament_cpplint()
  ament_lint_cmake()
  ament_uncrustify(LANGUAGE "c++")

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_buffer test/test_buffer.cpp)
  target_link_libraries(test_buffer
    ${PROJECT_NAME}
    # Used, but not linked to test tf2_ros's exports:
    #   rclcpp::rclcpp
  )

  ament_add_gtest(test_buffer_server test/test_buffer_server.cpp)
  target_link_libraries(test_buffer_server
    ${PROJECT_NAME}
    # Used, but not linked to test tf2_ros's exports:
    #   rclcpp::rclcpp
    #   rclcpp_action::rclcpp_action
    #   ${tf2_msgs_TARGETS}
  )

  ament_add_gtest(test_buffer_client test/test_buffer_client.cpp)
  target_link_libraries(test_buffer_client
    ${PROJECT_NAME}
    # Used, but not linked to test tf2_ros's exports:
    #   rclcpp::rclcpp
    #   rclcpp_action::rclcpp_action
    #   ${tf2_msgs_TARGETS}
  )

  # Adds a tf2_ros message_filter unittest that uses
  # multiple target frames and a non-zero time tolerance
  ament_add_gtest(${PROJECT_NAME}_test_message_filter test/message_filter_test.cpp)
  target_link_libraries(${PROJECT_NAME}_test_message_filter
    ${PROJECT_NAME}
    # Used, but not linked to test tf2_ros's exports:
    #   ${geometry_msgs_TARGETS}
    #   message_filters::message_filters
    #   rclcpp::rclcpp
  )

  ament_add_gtest(${PROJECT_NAME}_test_transform_listener test/test_transform_listener.cpp)
  target_link_libraries(${PROJECT_NAME}_test_transform_listener
    ${PROJECT_NAME}
    # Used, but not linked to test tf2_ros's exports:
    #   rclcpp::rclcpp
  )

  ament_add_gtest(${PROJECT_NAME}_test_static_transform_broadcaster test/test_static_transform_broadcaster.cpp)
  target_link_libraries(${PROJECT_NAME}_test_static_transform_broadcaster
    ${PROJECT_NAME}
    # Used, but not linked to test tf2_ros's exports:
    #   rclcpp::rclcpp
  )

  ament_add_gtest(${PROJECT_NAME}_test_transform_broadcaster test/test_transform_broadcaster.cpp)
  target_link_libraries(${PROJECT_NAME}_test_transform_broadcaster
    ${PROJECT_NAME}
    # Used, but not linked to test tf2_ros's exports:
    #   rclcpp::rclcpp
  )

  ament_add_gtest(${PROJECT_NAME}_test_time_reset test/time_reset_test.cpp)
  target_link_libraries(${PROJECT_NAME}_test_time_reset
    ${PROJECT_NAME}
    ${rosgraph_msgs_TARGETS}
    # Used, but not linked to test tf2_ros's exports:
    #   ${builtin_interfaces_TARGETS}
    #   rclcpp::rclcpp
  )

  ament_add_gtest(${PROJECT_NAME}_test_velocity test/velocity_test.cpp)
  target_link_libraries(${PROJECT_NAME}_test_velocity
    ${PROJECT_NAME}
    # Used, but not linked to test tf2_ros's exports:
    #   ${builtin_interfaces_TARGETS}
    #   rclcpp::rclcpp
  )

  ament_add_gtest(${PROJECT_NAME}_test_listener test/listener_unittest.cpp)
  target_link_libraries(${PROJECT_NAME}_test_listener
    ${PROJECT_NAME}
    # Used, but not linked to test tf2_ros's exports:
    #   ${builtin_interfaces_TARGETS}
    #   rclcpp::rclcpp
  )

endif()

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})

# Export modern CMake targets
ament_export_targets(export_${PROJECT_NAME})

ament_export_dependencies(
  rcl_interfaces
  rclcpp_components
  builtin_interfaces
  geometry_msgs
  message_filters
  rclcpp
  rclcpp_action
  tf2
  tf2_msgs
)
ament_package()
