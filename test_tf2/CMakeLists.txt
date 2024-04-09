cmake_minimum_required(VERSION 3.5)

project(test_tf2)

find_package(ament_cmake REQUIRED)

if(NOT BUILD_TESTING)
  ament_package()
  return()
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

if (BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(builtin_interfaces REQUIRED)
# Work around broken find module in AlmaLinux/RHEL eigen3-devel from PowerTools repo
  find_package(Eigen3 QUIET NO_MODULE)
  if(NOT Eigen3_FOUND)
    find_package(Eigen3 REQUIRED)
  endif()
  find_package(geometry_msgs REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_bullet REQUIRED)
  find_package(tf2_eigen REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  find_package(tf2_kdl REQUIRED)
  find_package(tf2_ros REQUIRED)

  ament_find_gtest()

  ament_add_gtest(buffer_core_test test/buffer_core_test.cpp)
  if(TARGET buffer_core_test)
    target_link_libraries(buffer_core_test
      ${builtin_interfaces_TARGETS}
      ${geometry_msgs_TARGETS}
      rclcpp::rclcpp
      tf2::tf2
      ${tf2_geometry_msgs_TARGETS}
      tf2_ros::tf2_ros)
  endif()

  ament_add_gtest(test_message_filter test/test_message_filter.cpp)
  if(TARGET test_message_filter)
    target_link_libraries(test_message_filter
      ${builtin_interfaces_TARGETS}
      ${geometry_msgs_TARGETS}
      rclcpp::rclcpp
      tf2::tf2
      tf2_ros::tf2_ros)
  endif()

  ament_add_gtest(test_convert test/test_convert.cpp)
  if(TARGET test_convert)
    target_link_libraries(test_convert
      ${geometry_msgs_TARGETS}
      tf2::tf2
      tf2_bullet::tf2_bullet
      tf2_eigen::tf2_eigen
      ${tf2_geometry_msgs_TARGETS}
      tf2_kdl::tf2_kdl)
    if(TARGET Eigen3::Eigen)
      # TODO(sloretz) require target to exist when https://github.com/ros2/choco-packages/issues/19 is addressed
      target_link_libraries(test_convert Eigen3::Eigen)
    else()
      target_include_directories(test_convert PRIVATE ${Eigen3_INCLUDE_DIRS})
    endif()
  endif()

  ament_add_gtest(test_utils test/test_utils.cpp)
  if(TARGET test_utils)
    target_link_libraries(test_utils
      ${geometry_msgs_TARGETS}
      tf2::tf2
      ${tf2_geometry_msgs_TARGETS}
      tf2_kdl::tf2_kdl)
  endif()

  add_executable(test_buffer_server test/test_buffer_server.cpp)
  if(TARGET test_buffer_server)
    target_link_libraries(test_buffer_server
      rclcpp::rclcpp
      tf2_ros::tf2_ros)
  endif()

  add_executable(test_buffer_client test/test_buffer_client.cpp)
  if(TARGET test_buffer_client)
    target_link_libraries(test_buffer_client
      ${GTEST_LIBRARIES}
      rclcpp::rclcpp
      tf2_bullet::tf2_bullet
      ${tf2_geometry_msgs_TARGETS}
      tf2_kdl::tf2_kdl
      tf2_ros::tf2_ros)
    add_launch_test(test/buffer_client_tester.launch.py)
  endif()

  add_executable(test_static_publisher test/test_static_publisher.cpp)
  if(TARGET test_static_publisher)
    target_link_libraries(test_static_publisher
      ${GTEST_LIBRARIES}
      ${geometry_msgs_TARGETS}
      rclcpp::rclcpp
      tf2::tf2
      tf2_ros::tf2_ros)
    add_launch_test(test/static_publisher.launch.py)
  endif()

  ament_add_gtest(test_tf2_bullet test/test_tf2_bullet.cpp)
  if(TARGET test_tf2_bullet)
    target_link_libraries(test_tf2_bullet
      rclcpp::rclcpp
      tf2_bullet::tf2_bullet
      tf2_ros::tf2_ros
      tf2::tf2)
  endif()

# TODO(ahcorde): enable once python part of tf2_geometry_msgs is working
# add_launch_test(test/test_buffer_client.launch.py)

# install executables
  install(TARGETS
    test_buffer_client
    test_buffer_server
    test_static_publisher
    DESTINATION lib/${PROJECT_NAME}
  )
  install(PROGRAMS
    test/test_buffer_client.py
    DESTINATION lib/${PROJECT_NAME}
  )

endif()

ament_package()
